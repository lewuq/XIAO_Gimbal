/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include "app_config.h"
#include "gimbal.h"

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const struct device *const gpiob = DEVICE_DT_GET(DT_NODELABEL(gpiob));
#define CAN_STB_PIN 14U
CAN_MSGQ_DEFINE(motor_rx_msgq, 32);
K_MUTEX_DEFINE(gimbal_mutex);
K_THREAD_STACK_DEFINE(motor_thread_stack, 4096);
static struct k_thread motor_thread_data;
K_THREAD_STACK_DEFINE(uart_thread_stack, 3072);
static struct k_thread uart_thread_data;
static struct gimbal app_gimbal;
static const struct device *const control_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));
K_MSGQ_DEFINE(control_uart_rx_msgq, sizeof(uint8_t), 256, 1);
RING_BUF_DECLARE(control_uart_tx_ring, 512);
static volatile uint32_t control_uart_rx_dropped;
static volatile uint32_t control_uart_tx_dropped;
static uint16_t telemetry_sequence;

struct __packed telemetry_packet {
	uint8_t sync0;
	uint8_t sync1;
	uint8_t version;
	uint8_t flags;
	uint16_t sequence;
	uint16_t yaw_cdeg;
	uint16_t pitch_cdeg;
	uint16_t yaw_target_cdeg;
	uint16_t pitch_target_cdeg;
	uint16_t crc;
};

static uint16_t crc16_ccitt(const uint8_t *data, size_t length);

static void control_uart_rx_isr(const struct device *dev, void *user_data)
{
	uint8_t data[32];
	int length;

	ARG_UNUSED(user_data);
	if (!uart_irq_update(dev)) return;
	while (uart_irq_rx_ready(dev)) {
		length = uart_fifo_read(dev, data, sizeof(data));
		if (length <= 0) break;
		for (int i = 0; i < length; ++i) {
			if (k_msgq_put(&control_uart_rx_msgq, &data[i], K_NO_WAIT) != 0) {
				control_uart_rx_dropped++;
			}
		}
	}
	while (uart_irq_tx_ready(dev)) {
		uint8_t *tx_data;
		uint32_t available = ring_buf_get_claim(&control_uart_tx_ring,
			&tx_data, 32U);
		if (available == 0U) {
			uart_irq_tx_disable(dev);
			break;
		}
		int sent = uart_fifo_fill(dev, tx_data, available);
		if (sent <= 0) break;
		(void)ring_buf_get_finish(&control_uart_tx_ring, (uint32_t)sent);
	}
}

static void queue_telemetry(const struct gimbal *snapshot)
{
	struct telemetry_packet packet = {
		.sync0 = 0x55,
		.sync1 = 0xaa,
		.version = 1,
		.flags = (uint8_t)snapshot->state,
		.sequence = ++telemetry_sequence,
		.yaw_cdeg = (uint16_t)(snapshot->yaw.angle_deg * 100.0f),
		.pitch_cdeg = (uint16_t)(snapshot->pitch.angle_deg * 100.0f),
		.yaw_target_cdeg = (uint16_t)(snapshot->yaw_target_deg * 100.0f),
		.pitch_target_cdeg = (uint16_t)(snapshot->pitch_target_deg * 100.0f),
		.crc = 0,
	};

	packet.crc = crc16_ccitt((const uint8_t *)&packet,
		sizeof(packet) - sizeof(packet.crc));
	unsigned int key = irq_lock();
	if (ring_buf_space_get(&control_uart_tx_ring) < sizeof(packet)) {
		irq_unlock(key);
		control_uart_tx_dropped++;
		return;
	}
	uint32_t written = ring_buf_put(&control_uart_tx_ring,
		(const uint8_t *)&packet, sizeof(packet));
	irq_unlock(key);
	if (written != sizeof(packet)) {
		control_uart_tx_dropped++;
		return;
	}
	uart_irq_tx_enable(control_uart);
}

static int configure_control_uart(void)
{
	int ret;

	if (!device_is_ready(control_uart)) return -ENODEV;
	ret = uart_irq_callback_user_data_set(control_uart,
		control_uart_rx_isr, NULL);
	if (ret != 0) return ret;
	uart_irq_rx_enable(control_uart);
	printk("Camera control UART RX: interrupt FIFO enabled on USART1 D7/PA10\n");
	return 0;
}

struct __packed gimbal_packet {
	uint8_t sync0;
	uint8_t sync1;
	uint8_t version;
	uint8_t flags;
	uint16_t sequence;
	uint16_t yaw_cdeg;
	uint16_t pitch_cdeg;
	uint16_t crc;
};

static uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
	uint16_t crc = 0xffffU;
	for (size_t i = 0; i < length; ++i) {
		crc ^= (uint16_t)data[i] << 8;
		for (int bit = 0; bit < 8; ++bit) {
			crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static void poll_control_uart(int64_t now_ms)
{
	static uint8_t frame[sizeof(struct gimbal_packet)];
	static size_t used;
	static uint16_t last_sequence;
	static int64_t last_valid_ms;
	static uint32_t valid_frames;
	static uint32_t invalid_frames;
	static uint32_t received_bytes;
	static uint32_t discarded_bytes;
	static int64_t last_uart_diag_ms;
	uint8_t byte;

	if (!device_is_ready(control_uart)) return;
	int uart_errors = uart_err_check(control_uart);
	if (uart_errors != 0) {
		printk("UART RX HW ERROR flags=0x%x (overrun/parity/framing/noise)\n",
		       uart_errors);
	}
	while (k_msgq_get(&control_uart_rx_msgq, &byte, K_NO_WAIT) == 0) {
		received_bytes++;
		if (used == 0U && byte != 0xaaU) {
			discarded_bytes++;
			if (discarded_bytes == 1U || discarded_bytes % 100U == 0U) {
				printk("UART RX RAW unsynced byte=0x%02x total=%u discarded=%u\n",
				       byte, received_bytes, discarded_bytes);
			}
			continue;
		}
		if (used == 1U && byte != 0x55U) {
			discarded_bytes++;
			if (discarded_bytes == 1U || discarded_bytes % 100U == 0U) {
				printk("UART RX RAW bad-sync2 byte=0x%02x total=%u discarded=%u\n",
				       byte, received_bytes, discarded_bytes);
			}
			used = byte == 0xaaU ? 1U : 0U;
			continue;
		}
		frame[used++] = byte;
		if (used != sizeof(frame)) continue;
		const struct gimbal_packet *packet = (const struct gimbal_packet *)frame;
		uint16_t expected = crc16_ccitt(frame, sizeof(frame) - sizeof(packet->crc));
		used = 0U;
		if (packet->version != 1U || packet->crc != expected) {
			invalid_frames++;
			if (invalid_frames == 1U || invalid_frames % 20U == 0U) {
				printk("UART RX INVALID count=%u version=%u crc=0x%04x expected=0x%04x\n",
				       invalid_frames, packet->version, packet->crc, expected);
			}
			continue;
		}
		last_sequence = packet->sequence;
		last_valid_ms = now_ms;
		valid_frames++;
		if ((packet->flags & 0x01U) != 0U || valid_frames == 1U ||
		    valid_frames % 20U == 0U) {
			printk("UART RX OK seq=%u flags=0x%02x yaw=%.1f pitch=%.1f valid=%u invalid=%u\n",
			       packet->sequence, packet->flags,
			       (double)packet->yaw_cdeg / APP_UART_TARGET_SCALE,
			       (double)packet->pitch_cdeg / APP_UART_TARGET_SCALE,
			       valid_frames, invalid_frames);
		}
		k_mutex_lock(&gimbal_mutex, K_FOREVER);
		if ((packet->flags & 0x08U) != 0U) {
			gimbal_request_calibration(&app_gimbal);
		} else if ((packet->flags & 0x01U) != 0U) {
			gimbal_request_start(&app_gimbal);
		}
		(void)gimbal_set_yaw_target(&app_gimbal, (float)packet->yaw_cdeg / APP_UART_TARGET_SCALE);
		(void)gimbal_set_pitch_target(&app_gimbal, (float)packet->pitch_cdeg / APP_UART_TARGET_SCALE);
		k_mutex_unlock(&gimbal_mutex);
	}
	if (received_bytes != 0U && now_ms - last_uart_diag_ms >= 2000) {
		last_uart_diag_ms = now_ms;
		printk("UART RX STATUS bytes=%u valid=%u invalid=%u discarded=%u partial=%u\n",
		       received_bytes, valid_frames, invalid_frames, discarded_bytes,
		       (unsigned int)used);
	}
	if (control_uart_rx_dropped != 0U && now_ms - last_uart_diag_ms >= 2000) {
		printk("UART RX QUEUE OVERFLOW dropped=%u\n", control_uart_rx_dropped);
	}
	if (last_valid_ms != 0 && now_ms - last_valid_ms > APP_UART_TIMEOUT_MS) {
		last_valid_ms = 0;
		printk("UART control timeout after seq=%u; retaining last safe target\n", last_sequence);
	}
}

static const char *can_state_name(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE: return "ERROR_ACTIVE";
	case CAN_STATE_ERROR_WARNING: return "ERROR_WARNING";
	case CAN_STATE_ERROR_PASSIVE: return "ERROR_PASSIVE";
	case CAN_STATE_BUS_OFF: return "BUS_OFF";
	case CAN_STATE_STOPPED: return "STOPPED";
	default: return "UNKNOWN";
	}
}

static int force_can_transceiver_active(const char *stage)
{
	int level;
	int ret;
	if (!device_is_ready(gpiob)) return -ENODEV;
	ret = gpio_pin_configure(gpiob, CAN_STB_PIN, GPIO_OUTPUT_LOW);
	if (ret == 0) ret = gpio_pin_set_raw(gpiob, CAN_STB_PIN, 0);
	if (ret != 0) return ret;
	level = gpio_pin_get_raw(gpiob, CAN_STB_PIN);
	printk("CAN STB stage=%s PB14 raw=%d (%s)\n", stage, level,
	       level == 0 ? "ACTIVE" : "STANDBY - ERROR");
	return level == 0 ? 0 : (level < 0 ? level : -EIO);
}

static int configure_can(void)
{
	const struct can_filter filter = { .id = 0U, .mask = 0U, .flags = 0U };
	int ret;
	if (!device_is_ready(can_dev)) return -ENODEV;
	ret = force_can_transceiver_active("before-can-start");
	if (ret == 0) ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (ret == 0) ret = can_set_bitrate(can_dev, APP_CAN_BITRATE);
	if (ret == 0) ret = can_add_rx_filter_msgq(can_dev, &motor_rx_msgq, &filter);
	if (ret >= 0) ret = can_start(can_dev);
	if (ret == 0) ret = force_can_transceiver_active("after-can-start");
	if (ret != 0) {
		printk("CAN INIT ERROR ret=%d (%s)\n", ret, lkm_can_error_name(ret));
		return ret;
	}
	printk("CAN READY: CLASSIC NORMAL, fixed %u bps, fixed IDs 0x%03x/0x%03x\n",
	       APP_CAN_BITRATE, GIMBAL_YAW_CAN_ID, GIMBAL_PITCH_CAN_ID);
	return 0;
}

static void copy_gimbal_snapshot(struct gimbal *snapshot)
{
	k_mutex_lock(&gimbal_mutex, K_FOREVER);
	*snapshot = app_gimbal;
	k_mutex_unlock(&gimbal_mutex);
}

static void motor_task(void *unused1, void *unused2, void *unused3)
{
	int64_t next_can_report = 0;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);
	while (true) {
		struct can_frame frame;
		struct gimbal snapshot;
		int64_t now_ms = k_uptime_get();

		k_mutex_lock(&gimbal_mutex, K_FOREVER);
		while (k_msgq_get(&motor_rx_msgq, &frame, K_NO_WAIT) == 0) {
			gimbal_process_frame(&app_gimbal, &frame, now_ms);
		}
		gimbal_tick(&app_gimbal, now_ms);
		snapshot = app_gimbal;
		k_mutex_unlock(&gimbal_mutex);

		if (now_ms >= next_can_report) {
			enum can_state state;
			struct can_bus_err_cnt errors;
			int ret = can_get_state(can_dev, &state, &errors);

			if (ret == 0) {
				printk("CAN STATUS state=%s tx_err=%u rx_err=%u yaw_rx=%u pitch_rx=%u yaw=%.1f/%.1f raw=%u pitch=%.1f/%.1f raw=%u\n",
				       can_state_name(state), errors.tx_err_cnt,
				       errors.rx_err_cnt, snapshot.yaw.rx_count,
				       snapshot.pitch.rx_count,
				       (double)snapshot.yaw.angle_deg,
				       (double)snapshot.yaw_target_deg,
				       snapshot.yaw.encoder_raw,
				       (double)snapshot.pitch.angle_deg,
				       (double)snapshot.pitch_target_deg,
				       snapshot.pitch.encoder_raw);
				printk("ENCODER calibration_valid=%u yaw_raw=%u yaw_unwrapped=%ld pitch_raw=%u\n",
				       GIMBAL_RELATIVE_CALIBRATION_VALID, snapshot.yaw.encoder_raw,
				       (long)snapshot.yaw.unwrapped_counts, snapshot.pitch.encoder_raw);
			}
			next_can_report = now_ms + 2000;
		}
		k_msleep(APP_MOTOR_TASK_PERIOD_MS);
	}
}

static void uart_task(void *unused1, void *unused2, void *unused3)
{
	int64_t next_telemetry = 0;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);
	while (true) {
		int64_t now_ms = k_uptime_get();
		poll_control_uart(now_ms);
		if (now_ms >= next_telemetry) {
			struct gimbal snapshot;
			copy_gimbal_snapshot(&snapshot);
			queue_telemetry(&snapshot);
			next_telemetry = now_ms + APP_UART_TELEMETRY_PERIOD_MS;
		}
		k_msleep(5);
	}
}

int main(void)
{
	bool can_ready = false;
	int ret;

	printk("XIAO STM32C5 dual LKM gimbal: fixed classic CAN 1 Mbps\n");
	printk("Camera control UART: D6 TX / D7 RX, 115200 8N1\n");
	printk("Yaw MS3506=0x%03x, Pitch MS3008=0x%03x; scan disabled\n",
	       GIMBAL_YAW_CAN_ID, GIMBAL_PITCH_CAN_ID);
	gimbal_init(&app_gimbal, can_dev);
	ret = configure_control_uart();
	if (ret != 0) {
		printk("Camera control UART init failed: %d\n", ret);
		return ret;
	}
	ret = configure_can();
	if (ret != 0) gimbal_set_fault(&app_gimbal, "CAN init failed");
	else can_ready = true;
	printk("Control: UART CENTER/CAL/absolute targets, LKM 0xA8 position loop\n");
	printk("Motor task: priority 0, %u ms; UART task: priority 1, telemetry %u ms\n",
	       APP_MOTOR_TASK_PERIOD_MS, APP_UART_TELEMETRY_PERIOD_MS);
	printk("Motor startup guard: %u ms; fixed mechanical center yaw %.1f pitch %.1f\n",
	       GIMBAL_MOTOR_BOOT_DELAY_MS, (double)GIMBAL_YAW_CENTER_DEG,
	       (double)GIMBAL_PITCH_CENTER_DEG);
	if (can_ready) {
		(void)k_thread_create(&motor_thread_data, motor_thread_stack,
			K_THREAD_STACK_SIZEOF(motor_thread_stack), motor_task,
			NULL, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	}
	(void)k_thread_create(&uart_thread_data, uart_thread_stack,
		K_THREAD_STACK_SIZEOF(uart_thread_stack), uart_task,
		NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);

	while (true) {
		k_sleep(K_FOREVER);
	}
}
