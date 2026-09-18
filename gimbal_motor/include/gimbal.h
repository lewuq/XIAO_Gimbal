#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include "lkm_motor.h"

enum gimbal_state {
	GIMBAL_BOOT,
	GIMBAL_WAITING_FOR_FEEDBACK,
	GIMBAL_CENTERING,
	GIMBAL_READY,
	GIMBAL_FAULT,
};

struct gimbal {
	struct lkm_motor yaw;
	struct lkm_motor pitch;
	enum gimbal_state state;
	float yaw_target_deg;
	float pitch_target_deg;
	float centering_start_yaw_error_deg;
	float centering_start_pitch_error_deg;
	bool yaw_target_dirty;
	bool pitch_target_dirty;
	bool start_requested;
	bool calibrate_requested;
	bool yaw_enabled;
	bool pitch_enabled;
	bool yaw_motion_active;
	bool pitch_motion_active;
	bool idle_stop_pending;
	bool probe_pitch_next;
	int64_t state_started_ms;
	int64_t last_motion_command_ms;
	int64_t last_feedback_query_ms;
	int64_t last_probe_ms;
	const char *fault_reason;
};

void gimbal_init(struct gimbal *gimbal, const struct device *can_dev);
int gimbal_start(struct gimbal *gimbal, int64_t now_ms);
int gimbal_calibrate(struct gimbal *gimbal, int64_t now_ms);
void gimbal_request_start(struct gimbal *gimbal);
void gimbal_request_calibration(struct gimbal *gimbal);
void gimbal_set_fault(struct gimbal *gimbal, const char *reason);
void gimbal_tick(struct gimbal *gimbal, int64_t now_ms);
void gimbal_process_frame(struct gimbal *gimbal,
			  const struct can_frame *frame, int64_t now_ms);
bool gimbal_set_yaw_target(struct gimbal *gimbal, float angle_deg);
bool gimbal_set_pitch_target(struct gimbal *gimbal, float angle_deg);
const char *gimbal_state_name(enum gimbal_state state);

#endif /* GIMBAL_H */
