/*
 * Copyright (c) 2026 Mateusz Zerebecki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/reboot.h>

#include "wheel_csc.h"
#include "wheel_detector.h"

LOG_MODULE_REGISTER(wheel_speed, LOG_LEVEL_INF);

#define WHEEL_SENSOR_NODE DT_ALIAS(accel0)

#define WHEEL_STREAM_TRIGGERS                                                                      \
	{SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_INCLUDE},                                       \
	{                                                                                          \
		SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE                             \
	}

SENSOR_DT_STREAM_IODEV(wheel_sensor_iodev, WHEEL_SENSOR_NODE, WHEEL_STREAM_TRIGGERS);
RTIO_DEFINE_WITH_MEMPOOL(wheel_rtio, 2, 2, 8, 256, sizeof(void *));

static const struct device *const accelerometer = DEVICE_DT_GET(WHEEL_SENSOR_NODE);
static struct wheel_detector detector;
static bool image_confirmed;

static void reboot_unhealthy_image(void)
{
	/* Leave a test image unconfirmed so MCUboot rolls back after the reboot. */
	k_sleep(K_SECONDS(1));
	sys_reboot(SYS_REBOOT_COLD);
}

static float q31_to_float(q31_t value, int8_t shift)
{
	return ldexpf((float)value, shift - 31);
}

static int configure_accelerometer(void)
{
	struct sensor_value value;
	int err;

	value.val1 = 100;
	value.val2 = 0;
	err = sensor_attr_set(accelerometer, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
			      &value);
	if (err != 0) {
		return err;
	}

	sensor_g_to_ms2(16, &value);
	return sensor_attr_set(accelerometer, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
			       &value);
}

static void confirm_healthy_image(void)
{
	int err;

	if (image_confirmed) {
		return;
	}

	if (!boot_is_img_confirmed()) {
		err = boot_write_img_confirmed();
		if (err != 0) {
			LOG_ERR("Could not confirm MCUboot image: %d", err);
			return;
		}
		LOG_INF("MCUboot test image confirmed");
	}

	image_confirmed = true;
}

static int process_fifo_buffer(const uint8_t *buffer)
{
	const struct sensor_decoder_api *decoder;
	struct sensor_three_axis_data decoded = {0};
	struct wheel_revolution_event event;
	struct sensor_chan_spec channel = {SENSOR_CHAN_ACCEL_XYZ, 0};
	uint32_t fit = 0U;
	uint16_t frame_count;
	int err;

	err = sensor_get_decoder(accelerometer, &decoder);
	if (err != 0) {
		return err;
	}

	err = decoder->get_frame_count(buffer, channel, &frame_count);
	if (err != 0) {
		return err;
	}

	for (uint16_t i = 0U; i < frame_count; i++) {
		uint64_t timestamp_ns;
		float x_ms2;
		float y_ms2;

		err = decoder->decode(buffer, channel, &fit, 1U, &decoded);
		if (err != 1) {
			return err < 0 ? err : -EIO;
		}

		timestamp_ns =
			decoded.header.base_timestamp_ns + decoded.readings[0].timestamp_delta;
		x_ms2 = q31_to_float(decoded.readings[0].x, decoded.shift);
		y_ms2 = q31_to_float(decoded.readings[0].y, decoded.shift);

		/*
		 * The PCB is expected to be mounted parallel to the wheel, making X/Y the
		 * rotating plane. Axis selection will be calibrated on real hardware.
		 */
		if (wheel_detector_process(&detector, x_ms2, y_ms2, timestamp_ns, &event)) {
			LOG_INF("Wheel revolution %u", event.cumulative_revolutions);
			wheel_csc_publish(event.cumulative_revolutions, event.last_event_time_1024);
		}
	}

	return 0;
}

static int run_sensor_stream(void)
{
	struct rtio_sqe *stream_handle;
	int err;

	err = sensor_stream(&wheel_sensor_iodev, &wheel_rtio, NULL, &stream_handle);
	if (err != 0) {
		return err;
	}

	while (true) {
		struct rtio_cqe *cqe;
		uint8_t *buffer;
		uint32_t buffer_len;

		cqe = rtio_cqe_consume_block(&wheel_rtio);
		if (cqe->result != 0) {
			err = cqe->result;
			rtio_cqe_release(&wheel_rtio, cqe);
			return err;
		}

		err = rtio_cqe_get_mempool_buffer(&wheel_rtio, cqe, &buffer, &buffer_len);
		rtio_cqe_release(&wheel_rtio, cqe);
		if (err != 0) {
			return err;
		}

		err = process_fifo_buffer(buffer);
		rtio_release_buffer(&wheel_rtio, buffer, buffer_len);
		if (err != 0) {
			return err;
		}

		/* Sensor, FIFO, decoder, and RTIO all worked before accepting an update. */
		confirm_healthy_image();
	}
}

int main(void)
{
	int err;

	if (!device_is_ready(accelerometer)) {
		LOG_ERR("LIS2DH12 is not ready");
		reboot_unhealthy_image();
	}

	err = configure_accelerometer();
	if (err != 0) {
		LOG_ERR("LIS2DH12 configuration failed: %d", err);
		reboot_unhealthy_image();
	}

	wheel_detector_init(&detector);
	err = wheel_csc_start();
	if (err != 0) {
		LOG_ERR("Bluetooth initialization failed: %d", err);
		reboot_unhealthy_image();
	}

	LOG_INF("Wheel speed sensor ready; CSC and SMP DFU are advertising");
	err = run_sensor_stream();
	LOG_ERR("Sensor stream stopped: %d", err);

	reboot_unhealthy_image();
	return 0;
}
