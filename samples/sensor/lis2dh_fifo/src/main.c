/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/lis2dh.h>
#include <zephyr/kernel.h>

#define FIFO_READ_CAPACITY 32U

static K_SEM_DEFINE(fifo_ready, 0, 1);

static const struct device *const accel = DEVICE_DT_GET(DT_ALIAS(accel0));

static void fifo_trigger_handler(const struct device *dev,
				 const struct sensor_trigger *trigger)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trigger);

	k_sem_give(&fifo_ready);
}

static void print_fifo_samples(const struct lis2dh_fifo_sample *samples, size_t count)
{
	size_t i;

	for (i = 0U; i < count; i++) {
		printf("%" PRIu64 " ns: (%f, %f, %f) m/s^2\n", samples[i].timestamp_ns,
		       sensor_value_to_double(&samples[i].accel[0]),
		       sensor_value_to_double(&samples[i].accel[1]),
		       sensor_value_to_double(&samples[i].accel[2]));
	}
}

int main(void)
{
	struct sensor_trigger watermark_trigger = {
		.type = SENSOR_TRIG_FIFO_WATERMARK,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	struct sensor_trigger full_trigger = {
		.type = SENSOR_TRIG_FIFO_FULL,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	struct lis2dh_fifo_sample samples[FIFO_READ_CAPACITY];
	int status;

	if (!device_is_ready(accel)) {
		printf("LIS2DH device is not ready\n");
		return 0;
	}

	status = sensor_trigger_set(accel, &watermark_trigger, fifo_trigger_handler);
	if (status < 0) {
		printf("Cannot set FIFO watermark trigger: %d\n", status);
		return 0;
	}

	status = sensor_trigger_set(accel, &full_trigger, fifo_trigger_handler);
	if (status < 0) {
		printf("Cannot set FIFO full trigger: %d\n", status);
		return 0;
	}

	status = lis2dh_fifo_start(accel);
	if (status < 0) {
		printf("Cannot start LIS2DH FIFO: %d\n", status);
		return 0;
	}

	printf("LIS2DH FIFO streaming started\n");

	while (true) {
		size_t count;

		k_sem_take(&fifo_ready, K_FOREVER);

		do {
			status = lis2dh_fifo_read(accel, samples, ARRAY_SIZE(samples), &count);
			if (status < 0) {
				printf("FIFO read failed: %d\n", status);
				break;
			}

			print_fifo_samples(samples, count);
		} while (count == ARRAY_SIZE(samples));
	}
}
