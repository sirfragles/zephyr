/*
 * Copyright (c) 2026 Mateusz Zerebecki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#include "lis2dh.h"
#include "lis2dh_decoder.h"

static const struct sensor_decoder_api *decoder;

static q31_t expected_q31(int64_t micro_value, int8_t shift)
{
	return (q31_t)(micro_value * BIT64(31 - shift) / 1000000LL);
}

static void put_accel_sample(uint8_t *sample, int16_t x, int16_t y, int16_t z)
{
	sys_put_le16((uint16_t)x, &sample[0]);
	sys_put_le16((uint16_t)y, &sample[2]);
	sys_put_le16((uint16_t)z, &sample[4]);
}

static void *lis2dh_decoder_setup(void)
{
	zassert_ok(lis2dh_get_decoder(NULL, &decoder));
	zassert_not_null(decoder);
	return NULL;
}

ZTEST_SUITE(lis2dh_decoder, NULL, lis2dh_decoder_setup, NULL, NULL, NULL);

ZTEST(lis2dh_decoder, test_fifo_decode_and_timestamps)
{
	uint8_t buffer[sizeof(struct lis2dh_fifo_data) + 3U * LIS2DH_FIFO_SAMPLE_SIZE] = {0};
	struct lis2dh_fifo_data *encoded = (struct lis2dh_fifo_data *)buffer;
	struct {
		struct sensor_data_header header;
		int8_t shift;
		struct sensor_three_axis_sample_data readings[2];
	} output;
	struct sensor_three_axis_data *output_data = (struct sensor_three_axis_data *)&output;
	struct sensor_chan_spec accel = {SENSOR_CHAN_ACCEL_XYZ, 0};
	uint16_t frame_count;
	uint32_t fit = 0U;
	int rc;

	encoded->header.timestamp = 1000000000ULL;
	encoded->header.accel_scale = 9807U;
	encoded->header.is_fifo = 1U;
	encoded->header.range = 0U;
	encoded->header.mode = 1U;
	encoded->header.odr = LIS2DH_ODR_5;
	encoded->header.int_status = LIS2DH_FIFO_SRC_WTM;
	encoded->sample_count = 3U;
	put_accel_sample(&encoded->samples[0], 16, -32, 0);
	put_accel_sample(&encoded->samples[6], 32, -48, 16);
	put_accel_sample(&encoded->samples[12], 48, -64, 32);

	zassert_ok(decoder->get_frame_count(buffer, accel, &frame_count));
	zassert_equal(frame_count, 3U);
	zassert_true(decoder->has_trigger(buffer, SENSOR_TRIG_FIFO_WATERMARK));
	zassert_false(decoder->has_trigger(buffer, SENSOR_TRIG_FIFO_FULL));

	rc = decoder->decode(buffer, accel, &fit, 2U, output_data);
	zassert_equal(rc, 2);
	zassert_equal(fit, 2U);
	zassert_equal(output.header.reading_count, 2U);
	zassert_equal(output.header.base_timestamp_ns, 980000000ULL);
	zassert_equal(output.shift, 5);
	zassert_equal(output.readings[0].timestamp_delta, 0U);
	zassert_equal(output.readings[1].timestamp_delta, 10000000U);
	zassert_equal(output.readings[0].x, expected_q31(9807, 5));
	zassert_equal(output.readings[0].y, expected_q31(-19614, 5));
	zassert_equal(output.readings[0].z, 0);
	zassert_equal(output.readings[1].x, expected_q31(19614, 5));

	rc = decoder->decode(buffer, accel, &fit, 2U, output_data);
	zassert_equal(rc, 1);
	zassert_equal(fit, 3U);
	zassert_equal(output.header.reading_count, 1U);
	zassert_equal(output.readings[0].timestamp_delta, 20000000U);
	zassert_equal(output.readings[0].x, expected_q31(29421, 5));
	zassert_equal(decoder->decode(buffer, accel, &fit, 1U, output_data), 0);
}

ZTEST(lis2dh_decoder, test_fifo_full_and_empty_frames)
{
	struct lis2dh_fifo_data encoded = {
		.header =
			{
				.is_fifo = 1U,
				.range = 0U,
				.mode = 1U,
				.odr = LIS2DH_ODR_5,
				.int_status = LIS2DH_FIFO_SRC_OVRN,
			},
		.sample_count = LIS2DH_FIFO_DEPTH,
	};
	struct sensor_chan_spec accel = {SENSOR_CHAN_ACCEL_XYZ, 0};
	uint16_t frame_count;

	zassert_ok(decoder->get_frame_count((const uint8_t *)&encoded, accel, &frame_count));
	zassert_equal(frame_count, LIS2DH_FIFO_DEPTH);
	zassert_true(decoder->has_trigger((const uint8_t *)&encoded, SENSOR_TRIG_FIFO_FULL));

	encoded.sample_count = 0U;
	zassert_ok(decoder->get_frame_count((const uint8_t *)&encoded, accel, &frame_count));
	zassert_equal(frame_count, 0U);
}

ZTEST(lis2dh_decoder, test_one_shot_accel_and_temperature)
{
	struct lis2dh_rtio_data encoded = {
		.header =
			{
				.timestamp = 123456789ULL,
				.accel_scale = 9807U,
				.range = 0U,
				.mode = 1U,
				.odr = LIS2DH_ODR_5,
				.int_status = LIS2DH_STATUS_ZYX_DRDY,
			},
		.has_accel = 1U,
		.has_temp = 1U,
	};
	struct sensor_three_axis_data accel_output;
	struct sensor_q31_data temp_output;
	struct sensor_chan_spec accel = {SENSOR_CHAN_ACCEL_XYZ, 0};
	struct sensor_chan_spec temp = {SENSOR_CHAN_DIE_TEMP, 0};
	uint16_t frame_count;
	uint32_t fit = 0U;

	put_accel_sample(encoded.accel, -16, 0, 16);
	sys_put_le32(25750000U, (uint8_t *)&encoded.temperature);

	zassert_ok(decoder->get_frame_count((const uint8_t *)&encoded, accel, &frame_count));
	zassert_equal(frame_count, 1U);
	zassert_true(decoder->has_trigger((const uint8_t *)&encoded, SENSOR_TRIG_DATA_READY));
	zassert_equal(decoder->decode((const uint8_t *)&encoded, accel, &fit, 1U, &accel_output),
		      1);
	zassert_equal(accel_output.header.base_timestamp_ns, 123456789ULL);
	zassert_equal(accel_output.readings[0].x, expected_q31(-9807, 5));
	zassert_equal(accel_output.readings[0].z, expected_q31(9807, 5));

	fit = 0U;
	zassert_ok(decoder->get_frame_count((const uint8_t *)&encoded, temp, &frame_count));
	zassert_equal(frame_count, 1U);
	zassert_equal(decoder->decode((const uint8_t *)&encoded, temp, &fit, 1U, &temp_output), 1);
	zassert_equal(temp_output.shift, 8);
	zassert_equal(temp_output.readings[0].temperature, expected_q31(25750000, 8));
}

ZTEST(lis2dh_decoder, test_size_and_invalid_channels)
{
	struct sensor_chan_spec accel = {SENSOR_CHAN_ACCEL_XYZ, 0};
	struct sensor_chan_spec invalid_index = {SENSOR_CHAN_ACCEL_XYZ, 1};
	struct sensor_chan_spec unsupported = {SENSOR_CHAN_PRESS, 0};
	struct lis2dh_fifo_data encoded = {.header.is_fifo = 1U};
	size_t base_size;
	size_t frame_size;
	uint16_t frame_count;

	zassert_ok(decoder->get_size_info(accel, &base_size, &frame_size));
	zassert_equal(base_size, sizeof(struct sensor_three_axis_data));
	zassert_equal(frame_size, sizeof(struct sensor_three_axis_sample_data));
	zassert_equal(
		decoder->get_frame_count((const uint8_t *)&encoded, invalid_index, &frame_count),
		-ENOTSUP);
	zassert_equal(
		decoder->get_frame_count((const uint8_t *)&encoded, unsupported, &frame_count),
		-ENOTSUP);
}
