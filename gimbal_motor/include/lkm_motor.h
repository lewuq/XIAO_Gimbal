#ifndef LKM_MOTOR_H
#define LKM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#define LKM_ENCODER_MAX_COUNT 32767U
#define LKM_ENCODER_COUNTS_PER_REV 32768U

/* Relative raw-encoder -> degrees mapping measured by can_driver_ms_calibration.
 * direction is +1 when raw counts increase with physical angle, -1 otherwise. */
struct lkm_axis_map {
	uint16_t center_raw;
	int8_t direction;
	float counts_per_deg;
	float negative_counts_per_deg;
	float center_deg;
	float command_scale;
	bool continuous_tracking;
	bool command_target_tracking;
};

struct lkm_motor {
	const struct device *can_dev;
	uint16_t can_id;
	const char *name;
	float hard_min_deg;
	float hard_max_deg;
	struct lkm_axis_map map;
	float angle_deg;
	float commanded_angle_deg;
	float speed_dps;
	int16_t power_raw;
	uint16_t encoder_count;   /* 0x90 reply: encoder = raw - offset */
	uint16_t encoder_raw;     /* 15-bit raw single-turn position */
	uint16_t encoder_offset;  /* 0x90 reply: motor stored zero point */
	uint16_t previous_encoder_raw;
	int32_t unwrapped_counts;
	uint8_t temperature_c;
	uint8_t last_command;
	int64_t last_feedback_ms;
	bool feedback_valid;
	bool encoder_tracking_valid;
	bool command_reference_valid;
	volatile bool tx_pending;
	int64_t tx_sent_ms;
	volatile int last_tx_error;
	int64_t last_error_log_ms;
	uint32_t rx_count;
};

void lkm_motor_init(struct lkm_motor *motor, const struct device *can_dev,
		    uint16_t can_id, const char *name,
		    float hard_min_deg, float hard_max_deg,
		    const struct lkm_axis_map *map);
int lkm_motor_on(struct lkm_motor *motor);
int lkm_motor_off(struct lkm_motor *motor);
int lkm_motor_stop(struct lkm_motor *motor);
int lkm_motor_read_encoder(struct lkm_motor *motor);
int lkm_motor_read_state(struct lkm_motor *motor);
int lkm_motor_set_position(struct lkm_motor *motor, float angle_deg,
			   uint16_t max_speed_dps);
bool lkm_motor_process_frame(struct lkm_motor *motor,
			     const struct can_frame *frame, int64_t now_ms);
bool lkm_motor_feedback_fresh(const struct lkm_motor *motor, int64_t now_ms,
			      int64_t max_age_ms);
const char *lkm_can_error_name(int error);

#endif /* LKM_MOTOR_H */
