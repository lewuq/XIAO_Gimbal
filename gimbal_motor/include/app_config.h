#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_CAN_BITRATE                 1000000U
#define APP_CONTROL_PERIOD_MS           50U
#define APP_MOTOR_TASK_PERIOD_MS        10U
#define APP_UI_REFRESH_MS               180U
#define APP_UART_TIMEOUT_MS             600U
#define APP_UART_TARGET_SCALE           100.0f
#define APP_UART_TELEMETRY_PERIOD_MS     50U

/* LKM arbitration IDs are 0x140 + node ID. */
#define GIMBAL_YAW_CAN_ID               0x141U
#define GIMBAL_PITCH_CAN_ID             0x142U

#define GIMBAL_YAW_HARD_MIN_DEG         0.0f
#define GIMBAL_YAW_HARD_MAX_DEG         345.0f
#define GIMBAL_PITCH_HARD_MIN_DEG       0.0f
#define GIMBAL_PITCH_HARD_MAX_DEG       180.0f
/* Recovery is only accepted while returning to home.  The MS3008 can report
 * about -41 degrees after an accumulated A8 sequence crosses its encoder
 * wrap even though the mechanism is not at its physical stop. */
#define GIMBAL_PITCH_RECOVERY_MIN_DEG   -50.0f

/* Conservative working limits used by the touch UI. */
#define GIMBAL_YAW_CONTROL_MIN_DEG      1.0f
#define GIMBAL_YAW_CONTROL_MAX_DEG      344.0f
#define GIMBAL_PITCH_CONTROL_MIN_DEG    1.0f
#define GIMBAL_PITCH_CONTROL_MAX_DEG    175.0f

#define GIMBAL_YAW_CENTER_DEG           180.0f
#define GIMBAL_PITCH_CENTER_DEG         90.0f
/* Keep motion locked until raw min/center/max values are measured with
 * can_driver_ms_calibration and the relative mapping is applied here. */
#define GIMBAL_RELATIVE_CALIBRATION_VALID 1

/* Yaw endpoints from the continuous read-only sweep 13:53--13:54:
 * raw 16 -> 0 degrees; raw 31774 -> nominal 345 degrees.
 * Span 31758 counts; midpoint 15895 -> 172.5 degrees.
 * The requested home (180 degrees) is independent of this mapping midpoint.
 * Pitch retains its earlier mapping and was not recalibrated by this sweep.
 * DIRECTION is +1 when raw counts increase with physical angle, -1 otherwise. */
#define GIMBAL_ENCODER_COUNTS_PER_REV   32768U
#define GIMBAL_YAW_MAP_CENTER_RAW       15895U
#define GIMBAL_YAW_MAP_DIRECTION        1
#define GIMBAL_YAW_MAP_COUNTS_PER_DEG   92.0522f
#define GIMBAL_YAW_MAP_NEG_COUNTS_PER_DEG 92.0522f
#define GIMBAL_YAW_MAP_CENTER_DEG       172.5f
/* 0xA8 angle is the geared output-shaft angle.  Multiplying this by the
 * internal reduction ratio made every 5 degree correction a 135 degree
 * command and caused the yaw axis to hunt violently. */
#define GIMBAL_YAW_COMMAND_SCALE        1.0f

#define GIMBAL_PITCH_MAP_CENTER_RAW     8445U
#define GIMBAL_PITCH_MAP_DIRECTION      1
#define GIMBAL_PITCH_MAP_COUNTS_PER_DEG 93.4083f
#define GIMBAL_PITCH_MAP_NEG_COUNTS_PER_DEG 93.4083f
#define GIMBAL_PITCH_MAP_CENTER_DEG     90.0f
#define GIMBAL_PITCH_COMMAND_SCALE      1.0f

#define GIMBAL_HOME_SPEED_DPS           20U
#define GIMBAL_UI_SPEED_DPS             30U
#define GIMBAL_CENTER_TOLERANCE_DEG     2.0f
#define GIMBAL_CENTER_DIVERGENCE_DEG    3.0f
#define GIMBAL_CENTER_DIVERGENCE_GRACE_MS 500U
#define GIMBAL_POSITION_STEP_INTERVAL_MS 250U
#define GIMBAL_LIMIT_FEEDBACK_MARGIN_DEG 1.0f
/* Auto tracking produces frequent sub-degree target corrections.  A 2 degree
 * command deadband discarded most of them, leaving the motor target stale
 * while manual joystick moves still appeared responsive. */
#define GIMBAL_TARGET_DEADBAND_DEG      0.25f
#define GIMBAL_IDLE_STOP_TOLERANCE_DEG  1.0f
#define GIMBAL_HOLD_REARM_DEG            2.0f
#define GIMBAL_FEEDBACK_POLL_MS         100U
#define GIMBAL_START_FEEDBACK_MAX_AGE_MS 1000U
#define GIMBAL_FEEDBACK_TIMEOUT_MS      3000U
#define GIMBAL_CENTER_TIMEOUT_MS        20000U
#define GIMBAL_FEEDBACK_STALE_MS        2000U
#define GIMBAL_MOTOR_BOOT_DELAY_MS      2000U
#define GIMBAL_DISARM_PROBE_PERIOD_MS   500U
#define GIMBAL_FEEDBACK_QUERY_GAP_MS    50U
#define ROUND_LCD_WIDTH                 240
#define ROUND_LCD_HEIGHT                240
#define TOUCH_I2C_ADDRESS               0x2eU

#endif /* APP_CONFIG_H */
