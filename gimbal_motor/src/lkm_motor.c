/* SPDX-License-Identifier: Apache-2.0 */

#include "lkm_motor.h"

#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "app_config.h"

#define LKM_CMD_MOTOR_OFF             0x80U
#define LKM_CMD_MOTOR_STOP            0x81U
#define LKM_CMD_MOTOR_ON              0x88U
#define LKM_CMD_READ_ENCODER          0x90U
#define LKM_CMD_READ_STATE_2          0x9cU
#define LKM_CMD_INCREMENTAL_POSITION_SPEED 0xa8U

static void put_u16_le(uint8_t *dst, uint16_t value)
{
	dst[0] = (uint8_t)value;
	dst[1] = (uint8_t)(value >> 8);
}

static void put_i32_le(uint8_t *dst, int32_t value)
{
	uint32_t raw = (uint32_t)value;
	dst[0] = (uint8_t)raw;
	dst[1] = (uint8_t)(raw >> 8);
	dst[2] = (uint8_t)(raw >> 16);
	dst[3] = (uint8_t)(raw >> 24);
}

static uint16_t get_u16_le(const uint8_t *src)
{
	return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static int16_t get_i16_le(const uint8_t *src)
{
	return (int16_t)get_u16_le(src);
}

static int32_t wrapped_delta(uint16_t from, uint16_t to)
{
	int32_t delta = (int32_t)to - (int32_t)from;

	if (delta > (int32_t)LKM_ENCODER_COUNTS_PER_REV / 2) {
		delta -= LKM_ENCODER_COUNTS_PER_REV;
	} else if (delta < -(int32_t)LKM_ENCODER_COUNTS_PER_REV / 2) {
		delta += LKM_ENCODER_COUNTS_PER_REV;
	}
	return delta;
}

static float counts_to_deg(const struct lkm_motor *motor, int32_t delta)
{
	float counts_per_deg = motor->map.counts_per_deg;

	if ((int32_t)motor->map.direction * delta < 0 &&
	    motor->map.negative_counts_per_deg > 0.0f) {
		counts_per_deg = motor->map.negative_counts_per_deg;
	}

	return motor->map.center_deg +
	       (float)motor->map.direction * (float)delta /
	       counts_per_deg;
}

static float track_raw_angle(struct lkm_motor *motor, uint16_t raw)
{
	float angle;
	int32_t centered_counts;

	/* Pitch uses the previously validated single-turn mapping.  Only yaw
	 * needs continuous unwrapping because its travel crosses encoder zero. */
	if (!motor->map.continuous_tracking) {
		centered_counts = wrapped_delta(motor->map.center_raw, raw);
		motor->unwrapped_counts = centered_counts;
		motor->previous_encoder_raw = raw;
		motor->encoder_tracking_valid = true;
		return counts_to_deg(motor, centered_counts);
	}

	/* The yaw travel is almost one complete encoder revolution.  Recomputing
	 * every sample as the shortest path from center is ambiguous close to the
	 * two end stops and used to turn roughly 345 degrees into roughly 29
	 * degrees.  Resolve the encoder wrap from consecutive samples instead. */
	if (!motor->encoder_tracking_valid) {
		motor->unwrapped_counts = wrapped_delta(motor->map.center_raw, raw);
		motor->encoder_tracking_valid = true;
	} else {
		int32_t delta = wrapped_delta(motor->previous_encoder_raw, raw);
		motor->unwrapped_counts += delta;
	}
	motor->previous_encoder_raw = raw;
	/* Never integrate a clamped angle: doing so loses position at the stop.
	 * Feedback refresh (CENTER/CAL) must not change the coordinate origin.
	 * Expose out-of-range measurements to the existing limit fault check. */
	angle = counts_to_deg(motor, motor->unwrapped_counts);
	return angle;
}

static void tx_complete(const struct device *dev, int error, void *user_data)
{
	struct lkm_motor *motor = user_data;

	ARG_UNUSED(dev);
	motor->last_tx_error = error;
	motor->tx_pending = false;
}

static int send_frame(struct lkm_motor *motor, const uint8_t data[8])
{
	struct can_frame frame = {
		.id = motor->can_id,
		.dlc = 8,
		.flags = 0U,
	};

	int ret;
	int64_t now_ms;

	/* Keep at most one outstanding frame per motor. This bounds the total
	 * number of FDCAN TX buffers used when a node is still booting or absent. */
	if (motor->tx_pending) {
		return -EBUSY;
	}
	memcpy(frame.data, data, 8U);
	/* A NULL callback makes Zephyr wait for bus ACK. Keep the UI loop alive
	 * when the bus is disconnected by always using asynchronous transmission.
	 */
	motor->tx_pending = true;
	motor->tx_sent_ms = k_uptime_get();
	ret = can_send(motor->can_dev, &frame, K_NO_WAIT, tx_complete, motor);
	if (ret != 0) {
		motor->tx_pending = false;
	}
	now_ms = k_uptime_get();
	if (ret != 0 && (ret != motor->last_tx_error ||
	    now_ms - motor->last_error_log_ms >= 1000)) {
		printk("CAN TX ERROR motor=%s id=0x%03x cmd=0x%02x ret=%d (%s)\n",
		       motor->name, motor->can_id, data[0], ret,
		       lkm_can_error_name(ret));
		motor->last_error_log_ms = now_ms;
	}
	if (ret == 0 && motor->last_tx_error != 0) {
		printk("CAN TX RECOVERED motor=%s id=0x%03x cmd=0x%02x\n",
		       motor->name, motor->can_id, data[0]);
	}
	motor->last_tx_error = ret;
	return ret;
}

const char *lkm_can_error_name(int error)
{
	switch (error) {
	case 0: return "OK";
	case -EAGAIN: return "NO_TX_BUFFER";
	case -EINVAL: return "INVALID_FRAME";
	case -ENETDOWN: return "CAN_STOPPED";
	case -ENETUNREACH: return "CAN_BUS_OFF";
	case -ETIMEDOUT: return "TX_TIMEOUT_OR_NO_ACK";
	case -EBUSY: return "TX_ALREADY_PENDING";
	case -EIO: return "TX_FAILED_OR_NO_ACK";
	case -ENODEV: return "CAN_DEVICE_NOT_READY";
	case -ENOTSUP: return "MODE_NOT_SUPPORTED";
	default: return "CAN_DRIVER_ERROR";
	}
}

static int send_simple_command(struct lkm_motor *motor, uint8_t command)
{
	uint8_t data[8] = {0};
	data[0] = command;
	return send_frame(motor, data);
}

void lkm_motor_init(struct lkm_motor *motor, const struct device *can_dev,
		    uint16_t can_id, const char *name,
		    float hard_min_deg, float hard_max_deg,
		    const struct lkm_axis_map *map)
{
	memset(motor, 0, sizeof(*motor));
	motor->can_dev = can_dev;
	motor->can_id = can_id;
	motor->name = name;
	motor->hard_min_deg = hard_min_deg;
	motor->hard_max_deg = hard_max_deg;
	if (map != NULL) {
		motor->map = *map;
	}
}

int lkm_motor_on(struct lkm_motor *motor)
{
	return send_simple_command(motor, LKM_CMD_MOTOR_ON);
}

int lkm_motor_off(struct lkm_motor *motor)
{
	return send_simple_command(motor, LKM_CMD_MOTOR_OFF);
}

int lkm_motor_stop(struct lkm_motor *motor)
{
	motor->command_reference_valid = false;
	return send_simple_command(motor, LKM_CMD_MOTOR_STOP);
}

int lkm_motor_read_encoder(struct lkm_motor *motor)
{
	return send_simple_command(motor, LKM_CMD_READ_ENCODER);
}

int lkm_motor_read_state(struct lkm_motor *motor)
{
	return send_simple_command(motor, LKM_CMD_READ_STATE_2);
}

int lkm_motor_set_position(struct lkm_motor *motor, float angle_deg,
			   uint16_t max_speed_dps)
{
	uint8_t data[8] = {0};
	int32_t angle_control;
	float physical_delta_deg;
	float motor_delta_deg;
	float motor_speed_dps;
	uint16_t speed_control;

	if (!GIMBAL_RELATIVE_CALIBRATION_VALID) {
		return -EACCES;
	}

	if (angle_deg < motor->hard_min_deg || angle_deg > motor->hard_max_deg ||
	    max_speed_dps == 0U) {
		return -ERANGE;
	}
	if (!motor->feedback_valid || motor->map.direction == 0 ||
	    motor->map.command_scale <= 0.0f) {
		return -EAGAIN;
	}

	/* 0xA8 is an incremental motor-side position loop. Converting the
	 * physical target to a delta avoids relying on the controller's
	 * persisted multi-turn zero, which previously drove directly to a
	 * mechanical stop when a physical angle was sent as an A4 absolute
	 * angle. Encoder direction maps physical-positive to motor-positive. */
	/* A8 offsets are accumulated by the motor controller.  While a joystick
	 * is dragged, feedback lags behind the moving command.  Recomputing every
	 * new offset from that lagging feedback stacks large offsets and causes
	 * overshoot.  Advance from the last accepted command target instead. */
	if (!motor->map.command_target_tracking) {
		motor->commanded_angle_deg = motor->angle_deg;
		motor->command_reference_valid = true;
	} else if (!motor->command_reference_valid ||
	    (fabsf(angle_deg - motor->commanded_angle_deg) < 0.05f &&
	     fabsf(angle_deg - motor->angle_deg) > GIMBAL_HOLD_REARM_DEG)) {
		motor->commanded_angle_deg = motor->angle_deg;
		motor->command_reference_valid = true;
	}
	physical_delta_deg = angle_deg - motor->commanded_angle_deg;
	/* Send the complete relative displacement once. The motor's internal
	 * position loop then produces continuous motion without 2-degree pauses. */
	motor_delta_deg = (float)motor->map.direction * physical_delta_deg *
		motor->map.command_scale;
	motor_speed_dps = (float)max_speed_dps * motor->map.command_scale;
	if (motor_speed_dps > 65535.0f) {
		motor_speed_dps = 65535.0f;
	}
	speed_control = (uint16_t)(motor_speed_dps + 0.5f);
	angle_control = (int32_t)(motor_delta_deg * 100.0f +
		(motor_delta_deg >= 0.0f ? 0.5f : -0.5f));
	if (angle_control == 0) {
		return 0;
	}
	data[0] = LKM_CMD_INCREMENTAL_POSITION_SPEED;
	put_u16_le(&data[2], speed_control);
	put_i32_le(&data[4], angle_control);
	printk("POSITION REL motor=%s current=%.1f target=%.1f physical_delta=%.1f motor_delta=%.1f scale=%.2f speed=%u/%u cmd=0xA8\n",
	       motor->name, (double)motor->angle_deg, (double)angle_deg,
	       (double)physical_delta_deg, (double)motor_delta_deg,
	       (double)motor->map.command_scale, max_speed_dps, speed_control);
	int ret = send_frame(motor, data);
	if (ret == 0) {
		motor->commanded_angle_deg = angle_deg;
	}
	return ret;
}

bool lkm_motor_process_frame(struct lkm_motor *motor,
			     const struct can_frame *frame, int64_t now_ms)
{
	uint16_t raw;

	if ((frame->flags & CAN_FRAME_IDE) != 0U || frame->id != motor->can_id ||
	    can_dlc_to_bytes(frame->dlc) != 8U) {
		return false;
	}

	motor->last_command = frame->data[0];
	if (frame->data[0] == LKM_CMD_READ_ENCODER) {
		/* 0x90 reply per LK protocol: DATA[2:3] = encoder (raw - offset),
		 * DATA[4:5] = encoderRaw, DATA[6:7] = encoderOffset. */
		motor->encoder_count = get_u16_le(&frame->data[2]);
		motor->encoder_raw = get_u16_le(&frame->data[4]);
		motor->encoder_offset = get_u16_le(&frame->data[6]);
		raw = motor->encoder_raw;
	} else {
		/* 0x9C and motion replies carry an offset-relative encoder in
		 * DATA[6:7], not encoderRaw. The V3 motors under test also return
		 * an encoderOffset field which changes with rotor position and
		 * therefore cannot safely reconstruct an absolute coordinate.
		 * Keep these replies as telemetry only; periodic 0x90 replies are
		 * the sole source of the mechanical angle. */
		motor->temperature_c = frame->data[1];
		motor->power_raw = get_i16_le(&frame->data[2]);
		motor->speed_dps = (float)get_i16_le(&frame->data[4]);
		motor->encoder_count = get_u16_le(&frame->data[6]);
		motor->rx_count++;
		return true;
	}

	if (raw > LKM_ENCODER_MAX_COUNT) {
		return false;
	}
	motor->angle_deg = track_raw_angle(motor, raw);
	motor->last_feedback_ms = now_ms;
	motor->feedback_valid = true;
	motor->rx_count++;
	if (motor->rx_count == 1U) {
		printk("CAN RX OK motor=%s id=0x%03x data=%02x.%02x.%02x.%02x.%02x.%02x.%02x.%02x raw=%u angle=%.1f\n",
		       motor->name, motor->can_id, frame->data[0], frame->data[1],
		       frame->data[2], frame->data[3], frame->data[4], frame->data[5],
		       frame->data[6], frame->data[7], raw,
		       (double)motor->angle_deg);
	}
	return true;
}

bool lkm_motor_feedback_fresh(const struct lkm_motor *motor, int64_t now_ms,
			      int64_t max_age_ms)
{
	return motor->feedback_valid && now_ms - motor->last_feedback_ms <= max_age_ms;
}
