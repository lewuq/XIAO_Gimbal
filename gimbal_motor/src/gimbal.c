/* SPDX-License-Identifier: Apache-2.0 */

#include "gimbal.h"

#include <errno.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "app_config.h"

static bool inside_hard_limits(const struct lkm_motor *motor)
{
	return motor->angle_deg >=
		       motor->hard_min_deg - GIMBAL_LIMIT_FEEDBACK_MARGIN_DEG &&
	       motor->angle_deg <=
		       motor->hard_max_deg + GIMBAL_LIMIT_FEEDBACK_MARGIN_DEG;
}

static bool pitch_inside_recovery_limits(const struct lkm_motor *motor)
{
	return motor->angle_deg >= GIMBAL_PITCH_RECOVERY_MIN_DEG &&
	       motor->angle_deg <=
		GIMBAL_PITCH_HARD_MAX_DEG + GIMBAL_LIMIT_FEEDBACK_MARGIN_DEG;
}

static void enter_fault(struct gimbal *gimbal, const char *reason)
{
	if (gimbal->state != GIMBAL_FAULT) {
		/* A fault must release holding torque. 0x81 only stops motion and may
		 * leave the controller holding its last position; 0x80 powers it off. */
		(void)lkm_motor_off(&gimbal->yaw);
		(void)lkm_motor_off(&gimbal->pitch);
		gimbal->yaw_enabled = false;
		gimbal->pitch_enabled = false;
		printk("Gimbal fault: %s\n", reason);
	}
	gimbal->state = GIMBAL_FAULT;
	gimbal->fault_reason = reason;
}

void gimbal_set_fault(struct gimbal *gimbal, const char *reason)
{
	enter_fault(gimbal, reason);
}

static bool send_targets(struct gimbal *gimbal, uint16_t speed_dps)
{
	int yaw_ret = gimbal->yaw_target_dirty ?
		lkm_motor_set_position(&gimbal->yaw,
			gimbal->yaw_target_deg, speed_dps) : 0;
	int pitch_ret = gimbal->pitch_target_dirty ?
		lkm_motor_set_position(&gimbal->pitch,
			gimbal->pitch_target_deg, speed_dps) : 0;
	return yaw_ret == 0 && pitch_ret == 0;
}

void gimbal_init(struct gimbal *gimbal, const struct device *can_dev)
{
	static const struct lkm_axis_map yaw_map = {
		.center_raw = GIMBAL_YAW_MAP_CENTER_RAW,
		.direction = GIMBAL_YAW_MAP_DIRECTION,
		.counts_per_deg = GIMBAL_YAW_MAP_COUNTS_PER_DEG,
		.negative_counts_per_deg = GIMBAL_YAW_MAP_NEG_COUNTS_PER_DEG,
		.center_deg = GIMBAL_YAW_MAP_CENTER_DEG,
		.command_scale = GIMBAL_YAW_COMMAND_SCALE,
		.continuous_tracking = true,
		.command_target_tracking = true,
	};
	static const struct lkm_axis_map pitch_map = {
		.center_raw = GIMBAL_PITCH_MAP_CENTER_RAW,
		.direction = GIMBAL_PITCH_MAP_DIRECTION,
		.counts_per_deg = GIMBAL_PITCH_MAP_COUNTS_PER_DEG,
		.negative_counts_per_deg = GIMBAL_PITCH_MAP_NEG_COUNTS_PER_DEG,
		.center_deg = GIMBAL_PITCH_MAP_CENTER_DEG,
		.command_scale = GIMBAL_PITCH_COMMAND_SCALE,
		.continuous_tracking = true,
		.command_target_tracking = true,
	};

	lkm_motor_init(&gimbal->yaw, can_dev, GIMBAL_YAW_CAN_ID, "YAW MS3506",
		GIMBAL_YAW_HARD_MIN_DEG, GIMBAL_YAW_HARD_MAX_DEG, &yaw_map);
	lkm_motor_init(&gimbal->pitch, can_dev, GIMBAL_PITCH_CAN_ID, "PITCH MS3008",
		GIMBAL_PITCH_HARD_MIN_DEG, GIMBAL_PITCH_HARD_MAX_DEG, &pitch_map);
	gimbal->state = GIMBAL_BOOT;
	gimbal->yaw_target_deg = GIMBAL_YAW_CENTER_DEG;
	gimbal->pitch_target_deg = GIMBAL_PITCH_CENTER_DEG;
	gimbal->yaw_target_dirty = true;
	gimbal->pitch_target_dirty = true;
	gimbal->start_requested = false;
	gimbal->calibrate_requested = false;
	gimbal->yaw_enabled = false;
	gimbal->pitch_enabled = false;
	gimbal->yaw_motion_active = false;
	gimbal->pitch_motion_active = false;
	gimbal->idle_stop_pending = false;
	gimbal->state_started_ms = k_uptime_get();
	gimbal->last_motion_command_ms = 0;
	gimbal->last_feedback_query_ms = 0;
	gimbal->last_probe_ms = gimbal->state_started_ms;
	gimbal->probe_pitch_next = false;
	gimbal->fault_reason = NULL;
}

int gimbal_start(struct gimbal *gimbal, int64_t now_ms)
{
	int ret;

#if !GIMBAL_RELATIVE_CALIBRATION_VALID
	ARG_UNUSED(now_ms);
	printk("MOTION BLOCKED: relative encoder calibration is not confirmed; run can_driver_ms_calibration first\n");
	return -EACCES;
#endif

	/* A retry must be based on fresh replies, never cached pre-fault angles. */
	/* Mechanical home is immutable. Slider targets are transient and must
	 * never become the next START/CALIBRATE center. */
	gimbal->yaw_target_deg = GIMBAL_YAW_CENTER_DEG;
	gimbal->pitch_target_deg = GIMBAL_PITCH_CENTER_DEG;
	gimbal->yaw_target_dirty = true;
	gimbal->pitch_target_dirty = true;
	gimbal->yaw.feedback_valid = false;
	gimbal->pitch.feedback_valid = false;
	gimbal->fault_reason = NULL;
	ret = lkm_motor_stop(&gimbal->yaw);
	if (ret == 0) ret = lkm_motor_stop(&gimbal->pitch);
	if (ret != 0) {
		enter_fault(gimbal, "initial CAN transmit failed");
		return ret;
	}

	gimbal->state = GIMBAL_WAITING_FOR_FEEDBACK;
	gimbal->yaw_enabled = false;
	gimbal->pitch_enabled = false;
	gimbal->yaw_motion_active = false;
	gimbal->pitch_motion_active = false;
	gimbal->idle_stop_pending = false;
	gimbal->state_started_ms = now_ms;
	gimbal->last_feedback_query_ms = now_ms - GIMBAL_FEEDBACK_QUERY_GAP_MS;
	gimbal->probe_pitch_next = false;
	printk("Waiting for MS3506/MS3008 encoder feedback before motion\n");
	return 0;
}

int gimbal_calibrate(struct gimbal *gimbal, int64_t now_ms)
{
	printk("Manual calibration requested\n");
	return gimbal_start(gimbal, now_ms);
}

void gimbal_request_start(struct gimbal *gimbal)
{
	gimbal->start_requested = true;
}

void gimbal_request_calibration(struct gimbal *gimbal)
{
	gimbal->calibrate_requested = true;
}

void gimbal_process_frame(struct gimbal *gimbal,
			  const struct can_frame *frame, int64_t now_ms)
{
	(void)lkm_motor_process_frame(&gimbal->yaw, frame, now_ms);
	(void)lkm_motor_process_frame(&gimbal->pitch, frame, now_ms);
}

void gimbal_tick(struct gimbal *gimbal, int64_t now_ms)
{
	if (gimbal->calibrate_requested) {
		gimbal->calibrate_requested = false;
		gimbal->start_requested = false;
		(void)gimbal_calibrate(gimbal, now_ms);
		return;
	}
	if (gimbal->start_requested) {
		gimbal->start_requested = false;
		(void)gimbal_start(gimbal, now_ms);
		return;
	}

	/* A 0x90 encoder query is read-only; it cannot enable or move a motor. */
	if (gimbal->state == GIMBAL_BOOT &&
	    now_ms - gimbal->state_started_ms < GIMBAL_MOTOR_BOOT_DELAY_MS) {
		return;
	}

	/* A fault keeps both motors disabled, but feedback must remain live for
	 * diagnosis and manual endpoint measurement. 0x90 is read-only. */
	if (gimbal->state == GIMBAL_FAULT &&
	    now_ms - gimbal->last_probe_ms >= GIMBAL_DISARM_PROBE_PERIOD_MS) {
		struct lkm_motor *motor = gimbal->probe_pitch_next ?
			&gimbal->pitch : &gimbal->yaw;
		if (!motor->tx_pending) {
			(void)lkm_motor_read_encoder(motor);
		}
		gimbal->probe_pitch_next = !gimbal->probe_pitch_next;
		gimbal->last_probe_ms = now_ms;
		return;
	}

	if (gimbal->state == GIMBAL_BOOT &&
	    now_ms - gimbal->last_probe_ms >= GIMBAL_DISARM_PROBE_PERIOD_MS) {
		struct lkm_motor *motor = gimbal->probe_pitch_next ?
			&gimbal->pitch : &gimbal->yaw;
		int ret = motor->tx_pending ? -EBUSY : lkm_motor_read_encoder(motor);

		printk("CAN PROBE 0x90 tx: %s(0x%03x)=%d (%s); read-only, motors disabled\n",
		       motor->name, motor->can_id, ret, lkm_can_error_name(ret));
		gimbal->probe_pitch_next = !gimbal->probe_pitch_next;
		gimbal->last_probe_ms = now_ms;
		return;
	}

	if (gimbal->state == GIMBAL_WAITING_FOR_FEEDBACK) {
		/* Once both absolute raw positions have arrived, stop issuing new
		 * probes and allow their TX callbacks to drain before enable. */
		if (lkm_motor_feedback_fresh(&gimbal->yaw, now_ms,
			GIMBAL_START_FEEDBACK_MAX_AGE_MS) &&
		    lkm_motor_feedback_fresh(&gimbal->pitch, now_ms,
			GIMBAL_START_FEEDBACK_MAX_AGE_MS)) {
			if (gimbal->yaw.tx_pending || gimbal->pitch.tx_pending) {
				if (now_ms - gimbal->state_started_ms >
				    GIMBAL_FEEDBACK_TIMEOUT_MS) {
					enter_fault(gimbal, "CAN transmit completion timeout");
				}
				return;
			}
			if (!inside_hard_limits(&gimbal->yaw) ||
			    !pitch_inside_recovery_limits(&gimbal->pitch)) {
				enter_fault(gimbal, "encoder angle outside mechanical limits");
				return;
			}
			int yaw_ret = lkm_motor_on(&gimbal->yaw);
			int pitch_ret = lkm_motor_on(&gimbal->pitch);
			if (yaw_ret != 0 || pitch_ret != 0) {
				printk("MOTOR ENABLE ERROR: yaw=%d (%s) pitch=%d (%s)\n",
				       yaw_ret, lkm_can_error_name(yaw_ret),
				       pitch_ret, lkm_can_error_name(pitch_ret));
				enter_fault(gimbal, "failed to start centering");
				return;
			}
			printk("Motor enable OK: sent 0x88 to 0x%03x and 0x%03x\n",
			       gimbal->yaw.can_id, gimbal->pitch.can_id);
			printk("CENTER FEEDBACK BASELINE: yaw_age=%lld ms pitch_age=%lld ms\n",
			       now_ms - gimbal->yaw.last_feedback_ms,
			       now_ms - gimbal->pitch.last_feedback_ms);
			gimbal->state = GIMBAL_CENTERING;
			gimbal->yaw_enabled = true;
			gimbal->pitch_enabled = true;
			gimbal->state_started_ms = now_ms;
			gimbal->last_motion_command_ms = now_ms;
			gimbal->yaw_target_dirty = true;
			gimbal->pitch_target_dirty = true;
			printk("Safe centering: yaw %.1f deg, pitch %.1f deg at %u dps\n",
			       (double)gimbal->yaw_target_deg,
			       (double)gimbal->pitch_target_deg, GIMBAL_HOME_SPEED_DPS);
			return;
		}
		if (now_ms - gimbal->last_feedback_query_ms >= GIMBAL_FEEDBACK_QUERY_GAP_MS) {
			struct lkm_motor *motor = gimbal->probe_pitch_next ?
				&gimbal->pitch : &gimbal->yaw;

			if (!motor->tx_pending) {
				(void)lkm_motor_read_encoder(motor);
			}
			gimbal->probe_pitch_next = !gimbal->probe_pitch_next;
			gimbal->last_feedback_query_ms = now_ms;
		}
		if (now_ms - gimbal->state_started_ms > GIMBAL_FEEDBACK_TIMEOUT_MS) {
			printk("CAN RX TIMEOUT: yaw_feedback=%d pitch_feedback=%d; check 1M bitrate, IDs, power, CANH/CANL and 120-ohm termination\n",
			       gimbal->yaw.feedback_valid, gimbal->pitch.feedback_valid);
			enter_fault(gimbal, "motor feedback timeout; no motion attempted");
		}
		return;
	}

	if (gimbal->state != GIMBAL_CENTERING && gimbal->state != GIMBAL_READY) {
		return;
	}

	if (!lkm_motor_feedback_fresh(&gimbal->yaw, now_ms,
		GIMBAL_FEEDBACK_STALE_MS) ||
	    !lkm_motor_feedback_fresh(&gimbal->pitch, now_ms,
		GIMBAL_FEEDBACK_STALE_MS)) {
		printk("CAN FEEDBACK STALE: yaw_valid=%d age=%lld ms pitch_valid=%d age=%lld ms limit=%u ms\n",
		       gimbal->yaw.feedback_valid,
		       now_ms - gimbal->yaw.last_feedback_ms,
		       gimbal->pitch.feedback_valid,
		       now_ms - gimbal->pitch.last_feedback_ms,
		       GIMBAL_FEEDBACK_STALE_MS);
		enter_fault(gimbal, "motor feedback lost");
		return;
	}

	if (!inside_hard_limits(&gimbal->yaw) ||
	    (gimbal->state == GIMBAL_CENTERING ?
		!pitch_inside_recovery_limits(&gimbal->pitch) :
		!inside_hard_limits(&gimbal->pitch))) {
		enter_fault(gimbal, "mechanical limit exceeded");
		return;
	}

	/* Keep both axes enabled after reaching their targets.  The previous yaw
	 * 0x81 STOP removed holding torque, so a manually displaced camera could
	 * never return to its target. */
	if (gimbal->state == GIMBAL_READY && gimbal->idle_stop_pending) {
		gimbal->yaw_motion_active = false;
		gimbal->pitch_motion_active = false;
		gimbal->idle_stop_pending = false;
		gimbal->last_motion_command_ms = now_ms;
		printk("Gimbal idle: both axes remain enabled for closed-loop holding\n");
		return;
	}

	if (gimbal->state == GIMBAL_READY) {
		if (gimbal->yaw_motion_active && !gimbal->yaw_target_dirty &&
		    fabsf(gimbal->yaw.angle_deg - gimbal->yaw_target_deg) <=
			GIMBAL_IDLE_STOP_TOLERANCE_DEG) {
			gimbal->yaw_motion_active = false;
			printk("YAW target reached; closed-loop holding retained\n");
		}
		if (gimbal->pitch_motion_active && !gimbal->pitch_target_dirty &&
		    fabsf(gimbal->pitch.angle_deg - gimbal->pitch_target_deg) <=
			GIMBAL_IDLE_STOP_TOLERANCE_DEG) {
			gimbal->pitch_motion_active = false;
			printk("PITCH target reached; closed-loop holding retained\n");
		}
	}

	if (gimbal->state == GIMBAL_CENTERING &&
	    (gimbal->yaw_target_dirty || gimbal->pitch_target_dirty) &&
	    now_ms - gimbal->last_motion_command_ms >= APP_CONTROL_PERIOD_MS) {
		if (gimbal->yaw.tx_pending || gimbal->pitch.tx_pending) {
			return;
		}
		if (!send_targets(gimbal, GIMBAL_HOME_SPEED_DPS)) {
			enter_fault(gimbal, "position command transmit failed");
			return;
		}
		gimbal->yaw_target_dirty = false;
		gimbal->pitch_target_dirty = false;
		gimbal->yaw_motion_active = true;
		gimbal->pitch_motion_active = true;
		gimbal->centering_start_yaw_error_deg =
			fabsf(gimbal->yaw.angle_deg - GIMBAL_YAW_CENTER_DEG);
		gimbal->centering_start_pitch_error_deg =
			fabsf(gimbal->pitch.angle_deg - GIMBAL_PITCH_CENTER_DEG);
		gimbal->last_motion_command_ms = now_ms;
	} else if (gimbal->state == GIMBAL_READY &&
		   (gimbal->yaw_target_dirty || gimbal->pitch_target_dirty) &&
		   now_ms - gimbal->last_motion_command_ms >= APP_CONTROL_PERIOD_MS) {
		int yaw_ret = 0;
		int pitch_ret = 0;
		bool enabled_now = false;

		if ((gimbal->yaw_target_dirty && gimbal->yaw.tx_pending) ||
		    (gimbal->pitch_target_dirty && gimbal->pitch.tx_pending)) {
			return;
		}
		if (gimbal->yaw_target_dirty && !gimbal->yaw_enabled) {
			yaw_ret = lkm_motor_on(&gimbal->yaw);
			if (yaw_ret == 0) {
				gimbal->yaw_enabled = true;
				enabled_now = true;
			}
		}
		if (gimbal->pitch_target_dirty && !gimbal->pitch_enabled) {
			pitch_ret = lkm_motor_on(&gimbal->pitch);
			if (pitch_ret == 0) {
				gimbal->pitch_enabled = true;
				enabled_now = true;
			}
		}
		if (yaw_ret != 0 || pitch_ret != 0) {
			enter_fault(gimbal, "motor re-enable transmit failed");
			return;
		}
		if (enabled_now) {
			gimbal->last_motion_command_ms = now_ms;
			return;
		}
		if (gimbal->yaw_target_dirty && !gimbal->yaw.tx_pending) {
			yaw_ret = lkm_motor_set_position(&gimbal->yaw,
				gimbal->yaw_target_deg, GIMBAL_UI_SPEED_DPS);
			if (yaw_ret == 0) gimbal->yaw_motion_active = true;
		}
		if (gimbal->pitch_target_dirty && !gimbal->pitch.tx_pending) {
			pitch_ret = lkm_motor_set_position(&gimbal->pitch,
				gimbal->pitch_target_deg, GIMBAL_UI_SPEED_DPS);
			if (pitch_ret == 0) gimbal->pitch_motion_active = true;
		}
		if (yaw_ret != 0 || pitch_ret != 0) {
			enter_fault(gimbal, "position command transmit failed");
			return;
		}
		gimbal->yaw_target_dirty = false;
		gimbal->pitch_target_dirty = false;
		gimbal->last_motion_command_ms = now_ms;
	} else if ((gimbal->state == GIMBAL_CENTERING ||
		    gimbal->state == GIMBAL_READY) &&
		   now_ms - gimbal->last_feedback_query_ms >= GIMBAL_FEEDBACK_POLL_MS) {
		int yaw_ret = gimbal->yaw.tx_pending ? 0 :
			lkm_motor_read_encoder(&gimbal->yaw);
		int pitch_ret = gimbal->pitch.tx_pending ? 0 :
			lkm_motor_read_encoder(&gimbal->pitch);

		if (yaw_ret != 0 || pitch_ret != 0) {
			printk("CAN FEEDBACK QUERY ERROR: yaw=%d (%s) pitch=%d (%s)\n",
			       yaw_ret, lkm_can_error_name(yaw_ret),
			       pitch_ret, lkm_can_error_name(pitch_ret));
			enter_fault(gimbal, "state query transmit failed");
			return;
		}
		gimbal->last_feedback_query_ms = now_ms;
	}

	if (gimbal->state == GIMBAL_CENTERING) {
		float yaw_error =
			fabsf(gimbal->yaw.angle_deg - GIMBAL_YAW_CENTER_DEG);
		float pitch_error =
			fabsf(gimbal->pitch.angle_deg - GIMBAL_PITCH_CENTER_DEG);

		if (!gimbal->yaw_target_dirty && !gimbal->pitch_target_dirty &&
		    now_ms - gimbal->last_motion_command_ms >=
			GIMBAL_CENTER_DIVERGENCE_GRACE_MS &&
		    (yaw_error > gimbal->centering_start_yaw_error_deg +
			 GIMBAL_CENTER_DIVERGENCE_DEG ||
		     pitch_error > gimbal->centering_start_pitch_error_deg +
			 GIMBAL_CENTER_DIVERGENCE_DEG)) {
			printk("CENTER DIVERGENCE: yaw_error=%.1f start=%.1f pitch_error=%.1f start=%.1f\n",
			       (double)yaw_error,
			       (double)gimbal->centering_start_yaw_error_deg,
			       (double)pitch_error,
			       (double)gimbal->centering_start_pitch_error_deg);
			enter_fault(gimbal, "centering moved away from target");
		} else if (yaw_error <=
			GIMBAL_CENTER_TOLERANCE_DEG &&
		    pitch_error <=
			GIMBAL_CENTER_TOLERANCE_DEG) {
			gimbal->state = GIMBAL_READY;
			gimbal->yaw_target_dirty = false;
			gimbal->pitch_target_dirty = false;
			gimbal->idle_stop_pending = true;
			gimbal->state_started_ms = now_ms;
			printk("Gimbal centered; touch control enabled; releasing holding PID\n");
		} else if (now_ms - gimbal->state_started_ms > GIMBAL_CENTER_TIMEOUT_MS) {
			enter_fault(gimbal, "safe centering timeout");
		}
	}

	if (gimbal->state == GIMBAL_READY &&
	    !gimbal->yaw_target_dirty && !gimbal->pitch_target_dirty &&
	    now_ms - gimbal->last_motion_command_ms >=
		GIMBAL_POSITION_STEP_INTERVAL_MS) {
		/* Re-arm only an idle holding axis displaced by an external force.
		 * Never overwrite an active internal position-loop command. */
		if (!gimbal->yaw_motion_active &&
		    fabsf(gimbal->yaw.angle_deg - gimbal->yaw_target_deg) >
			GIMBAL_HOLD_REARM_DEG) {
			gimbal->yaw_target_dirty = true;
		}
		if (!gimbal->pitch_motion_active &&
		    fabsf(gimbal->pitch.angle_deg - gimbal->pitch_target_deg) >
			GIMBAL_HOLD_REARM_DEG) {
			gimbal->pitch_target_dirty = true;
		}
	}
}

bool gimbal_set_yaw_target(struct gimbal *gimbal, float angle_deg)
{
	if (gimbal->state != GIMBAL_READY || angle_deg < GIMBAL_YAW_CONTROL_MIN_DEG ||
	    angle_deg > GIMBAL_YAW_CONTROL_MAX_DEG) {
		return false;
	}
	if (fabsf(angle_deg - gimbal->yaw_target_deg) < GIMBAL_TARGET_DEADBAND_DEG) {
		return false;
	}
	gimbal->yaw_target_deg = angle_deg;
	gimbal->yaw_target_dirty = true;
	return true;
}

bool gimbal_set_pitch_target(struct gimbal *gimbal, float angle_deg)
{
	if (gimbal->state != GIMBAL_READY ||
	    angle_deg < GIMBAL_PITCH_CONTROL_MIN_DEG ||
	    angle_deg > GIMBAL_PITCH_CONTROL_MAX_DEG) {
		return false;
	}
	if (fabsf(angle_deg - gimbal->pitch_target_deg) < GIMBAL_TARGET_DEADBAND_DEG) {
		return false;
	}
	gimbal->pitch_target_deg = angle_deg;
	gimbal->pitch_target_dirty = true;
	return true;
}

const char *gimbal_state_name(enum gimbal_state state)
{
	switch (state) {
	case GIMBAL_BOOT: return "DISARM";
	case GIMBAL_WAITING_FOR_FEEDBACK: return "CHECK";
	case GIMBAL_CENTERING: return "CENTER";
	case GIMBAL_READY: return "READY";
	case GIMBAL_FAULT: return "FAULT";
	default: return "UNKNOWN";
	}
}
