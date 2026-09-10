/* ST Microelectronics LIS2DH 3-axis accelerometer driver
 *
 * Copyright (c) 2026 Mateusz Zerebecki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_lis2dh

#include <errno.h>
#include <stdint.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "lis2dh.h"
#include "lis2dh_decoder.h"

static const int8_t lis2dh_accel_shift[] = {5, 6, 7, 8};

static uint32_t lis2dh_accel_period_ns(uint8_t odr, uint8_t mode)
{
	switch (odr) {
	case LIS2DH_ODR_1:
		return 1000000000U;
	case LIS2DH_ODR_2:
		return 100000000U;
	case LIS2DH_ODR_3:
		return 40000000U;
	case LIS2DH_ODR_4:
		return 20000000U;
	case LIS2DH_ODR_5:
		return 10000000U;
	case LIS2DH_ODR_6:
		return 5000000U;
	case LIS2DH_ODR_7:
		return 2500000U;
	case LIS2DH_ODR_8:
		return 1000000000U / 1620U;
	case LIS2DH_ODR_9:
		return mode == 0 ? 1000000000U / 5376U : 1000000000U / 1344U;
	default:
		return 0U;
	}
}

static q31_t lis2dh_micro_to_q31(int64_t micro_value, int8_t shift)
{
	return (q31_t)(micro_value * BIT64(31 - shift) / 1000000LL);
}

static int lis2dh_decoder_get_frame_count(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
					  uint16_t *frame_count)
{
	const struct lis2dh_encoded_header *header = (const struct lis2dh_encoded_header *)buffer;

	if (chan_spec.chan_idx != 0U) {
		return -ENOTSUP;
	}

	if (header->is_fifo) {
		const struct lis2dh_fifo_data *data = (const struct lis2dh_fifo_data *)buffer;

		if (!SENSOR_CHANNEL_IS_ACCEL(chan_spec.chan_type)) {
			*frame_count = 0U;
			return -ENOTSUP;
		}

		*frame_count = data->sample_count;
		return 0;
	}

	const struct lis2dh_rtio_data *data = (const struct lis2dh_rtio_data *)buffer;

	if (SENSOR_CHANNEL_IS_ACCEL(chan_spec.chan_type)) {
		*frame_count = data->has_accel ? 1U : 0U;
		return 0;
	}

	if (chan_spec.chan_type == SENSOR_CHAN_DIE_TEMP) {
		*frame_count = data->has_temp ? 1U : 0U;
		return 0;
	}

	*frame_count = 0U;
	return -ENOTSUP;
}

static void lis2dh_decode_accel_sample(const struct lis2dh_encoded_header *header,
				       const uint8_t *raw,
				       struct sensor_three_axis_sample_data *sample)
{
	const int8_t shift = lis2dh_accel_shift[header->range];
	int16_t x = (int16_t)sys_get_le16(&raw[0]);
	int16_t y = (int16_t)sys_get_le16(&raw[2]);
	int16_t z = (int16_t)sys_get_le16(&raw[4]);

	sample->x = lis2dh_micro_to_q31((int64_t)(x >> 4) * header->accel_scale, shift);
	sample->y = lis2dh_micro_to_q31((int64_t)(y >> 4) * header->accel_scale, shift);
	sample->z = lis2dh_micro_to_q31((int64_t)(z >> 4) * header->accel_scale, shift);
}

static int lis2dh_decode_fifo(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
			      uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct lis2dh_fifo_data *data = (const struct lis2dh_fifo_data *)buffer;
	const struct lis2dh_encoded_header *header = &data->header;
	struct sensor_three_axis_data *out = data_out;
	uint32_t period_ns;
	uint16_t count;

	if (!SENSOR_CHANNEL_IS_ACCEL(chan_spec.chan_type) || chan_spec.chan_idx != 0U) {
		return -EINVAL;
	}

	if (*fit >= data->sample_count) {
		return 0;
	}

	count = MIN(max_count, data->sample_count - *fit);
	if (count == 0U) {
		return 0;
	}

	period_ns = lis2dh_accel_period_ns(header->odr, header->mode);
	out->header.base_timestamp_ns = header->timestamp;
	if (data->sample_count > 1U) {
		uint64_t history_ns = (uint64_t)(data->sample_count - 1U) * period_ns;

		out->header.base_timestamp_ns =
			header->timestamp > history_ns ? header->timestamp - history_ns : 0U;
	}
	out->header.reading_count = count;
	out->shift = lis2dh_accel_shift[header->range];

	for (uint16_t i = 0U; i < count; i++) {
		uint32_t sample_index = *fit + i;
		const uint8_t *raw = &data->samples[sample_index * LIS2DH_FIFO_SAMPLE_SIZE];

		lis2dh_decode_accel_sample(header, raw, &out->readings[i]);
		out->readings[i].timestamp_delta = sample_index * period_ns;
	}

	*fit += count;
	return count;
}

static int lis2dh_decode_one_shot(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				  uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct lis2dh_rtio_data *data = (const struct lis2dh_rtio_data *)buffer;
	const struct lis2dh_encoded_header *header = &data->header;

	if (*fit != 0U) {
		return 0;
	}
	if (max_count == 0U || chan_spec.chan_idx != 0U) {
		return -EINVAL;
	}

	if (SENSOR_CHANNEL_IS_ACCEL(chan_spec.chan_type)) {
		struct sensor_three_axis_data *out = data_out;

		if (!data->has_accel) {
			return -ENODATA;
		}

		out->header.base_timestamp_ns = header->timestamp;
		out->header.reading_count = 1U;
		out->shift = lis2dh_accel_shift[header->range];
		out->readings[0].timestamp_delta = 0U;
		lis2dh_decode_accel_sample(header, data->accel, &out->readings[0]);
		*fit = 1U;
		return 1;
	}

	if (chan_spec.chan_type == SENSOR_CHAN_DIE_TEMP) {
		struct sensor_q31_data *out = data_out;
		int32_t temperature;

		if (!data->has_temp) {
			return -ENODATA;
		}

		temperature = (int32_t)sys_get_le32((const uint8_t *)&data->temperature);
		out->header.base_timestamp_ns = header->timestamp;
		out->header.reading_count = 1U;
		out->shift = 8;
		out->readings[0].timestamp_delta = 0U;
		out->readings[0].temperature = lis2dh_micro_to_q31(temperature, out->shift);
		*fit = 1U;
		return 1;
	}

	return -EINVAL;
}

static int lis2dh_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				 uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct lis2dh_encoded_header *header = (const struct lis2dh_encoded_header *)buffer;

	if (header->range >= ARRAY_SIZE(lis2dh_accel_shift)) {
		return -EINVAL;
	}

	if (header->is_fifo) {
		return lis2dh_decode_fifo(buffer, chan_spec, fit, max_count, data_out);
	}

	return lis2dh_decode_one_shot(buffer, chan_spec, fit, max_count, data_out);
}

static int lis2dh_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					size_t *frame_size)
{
	if (SENSOR_CHANNEL_IS_ACCEL(chan_spec.chan_type)) {
		*base_size = sizeof(struct sensor_three_axis_data);
		*frame_size = sizeof(struct sensor_three_axis_sample_data);
		return 0;
	}

	if (chan_spec.chan_type == SENSOR_CHAN_DIE_TEMP) {
		*base_size = sizeof(struct sensor_q31_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
	}

	return -ENOTSUP;
}

static bool lis2dh_decoder_has_trigger(const uint8_t *buffer, enum sensor_trigger_type trigger)
{
	const struct lis2dh_encoded_header *header = (const struct lis2dh_encoded_header *)buffer;

	if (header->is_fifo) {
		switch (trigger) {
		case SENSOR_TRIG_FIFO_WATERMARK:
			return (header->int_status & LIS2DH_FIFO_SRC_WTM) != 0U;
		case SENSOR_TRIG_FIFO_FULL:
			return (header->int_status & LIS2DH_FIFO_SRC_OVRN) != 0U;
		default:
			return false;
		}
	}

	return trigger == SENSOR_TRIG_DATA_READY &&
	       (header->int_status & LIS2DH_STATUS_ZYX_DRDY) != 0U;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = lis2dh_decoder_get_frame_count,
	.get_size_info = lis2dh_decoder_get_size_info,
	.decode = lis2dh_decoder_decode,
	.has_trigger = lis2dh_decoder_has_trigger,
};

int lis2dh_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);

	*decoder = &SENSOR_DECODER_NAME();
	return 0;
}
