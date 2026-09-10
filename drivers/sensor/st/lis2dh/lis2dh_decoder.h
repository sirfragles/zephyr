/* ST Microelectronics LIS2DH 3-axis accelerometer driver
 *
 * Copyright (c) 2026 Mateusz Zerebecki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LIS2DH_LIS2DH_DECODER_H_
#define ZEPHYR_DRIVERS_SENSOR_LIS2DH_LIS2DH_DECODER_H_

#include <stdint.h>

struct lis2dh_encoded_header {
	uint64_t timestamp;
	uint32_t accel_scale;
	uint8_t is_fifo: 1;
	uint8_t range: 2;
	uint8_t mode: 2;
	uint8_t reserved: 3;
	uint8_t odr: 4;
	uint8_t reserved_1: 4;
	uint8_t int_status;
} __packed;

struct lis2dh_fifo_data {
	struct lis2dh_encoded_header header;
	uint8_t sample_count;
	uint8_t samples[];
} __packed;

struct lis2dh_rtio_data {
	struct lis2dh_encoded_header header;
	uint8_t has_accel: 1;
	uint8_t has_temp: 1;
	uint8_t reserved: 6;
	uint8_t accel[6];
	int32_t temperature;
} __packed;

#endif /* ZEPHYR_DRIVERS_SENSOR_LIS2DH_LIS2DH_DECODER_H_ */
