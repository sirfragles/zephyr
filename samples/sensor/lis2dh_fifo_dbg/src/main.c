/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/stats/stats.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lis2dh_fifo_dbg, CONFIG_LOG_DEFAULT_LEVEL);

#include "bluetooth.h"

#define ACCEL_STREAM_TRIGGERS                                                                      \
	{SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE},                                  \
	{                                                                                          \
		SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_INCLUDE                                  \
	}

SENSOR_DT_STREAM_IODEV(accel_stream, DT_ALIAS(accel0), ACCEL_STREAM_TRIGGERS);
RTIO_DEFINE_WITH_MEMPOOL(stream_ctx, 1, 1, 20, 256, sizeof(void *));

static const struct device *const accel = DEVICE_DT_GET(DT_ALIAS(accel0));

/* Exposed to the MCUmgr `stat` group, e.g. `mcumgr ... stat list`. */
STATS_SECT_START(lis2dh_fifo_stats)
STATS_SECT_ENTRY(batches)
STATS_SECT_ENTRY(frames)
STATS_SECT_END;

STATS_NAME_START(lis2dh_fifo_stats)
STATS_NAME(lis2dh_fifo_stats, batches)
STATS_NAME(lis2dh_fifo_stats, frames)
STATS_NAME_END(lis2dh_fifo_stats);

STATS_SECT_DECL(lis2dh_fifo_stats) lis2dh_fifo_stats;

static void report_fifo_batch(const uint8_t *buffer)
{
	const struct sensor_decoder_api *decoder;
	struct sensor_three_axis_data accel_data;
	struct sensor_chan_spec channel = {SENSOR_CHAN_ACCEL_XYZ, 0};
	uint16_t frame_count;
	uint32_t fit = 0U;
	int status;

	status = sensor_get_decoder(accel, &decoder);
	if (status < 0) {
		LOG_ERR("Cannot get LIS2DH decoder: %d", status);
		return;
	}

	status = decoder->get_frame_count(buffer, channel, &frame_count);
	if (status < 0) {
		LOG_ERR("Cannot get FIFO frame count: %d", status);
		return;
	}

	STATS_INC(lis2dh_fifo_stats, batches);
	STATS_INCN(lis2dh_fifo_stats, frames, frame_count);

	while (fit < frame_count) {
		status = decoder->decode(buffer, channel, &fit, 1U, &accel_data);
		if (status < 0) {
			LOG_ERR("Cannot decode FIFO frame: %d", status);
			return;
		}

		LOG_DBG("%" PRIu64 " ns: (%" PRIq(6) ", %" PRIq(6) ", %" PRIq(6) ") m/s^2",
			PRIsensor_three_axis_data_arg(accel_data, 0));
	}
}

static void stream_thread(void *p1, void *p2, void *p3)
{
	struct rtio_sqe *handle;
	struct rtio_cqe *cqe;
	uint8_t *buffer = NULL;
	uint32_t buffer_len = 0U;
	int status;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!device_is_ready(accel)) {
		LOG_ERR("LIS2DH device is not ready");
		return;
	}

	status = sensor_stream(&accel_stream, &stream_ctx, NULL, &handle);
	if (status < 0) {
		LOG_ERR("Cannot start LIS2DH FIFO stream: %d", status);
		return;
	}

	LOG_INF("LIS2DH FIFO stream started");

	while (true) {
		cqe = rtio_cqe_consume_block(&stream_ctx);
		status = cqe->result;
		if (status == 0) {
			status = rtio_cqe_get_mempool_buffer(&stream_ctx, cqe, &buffer,
							     &buffer_len);
		}
		rtio_cqe_release(&stream_ctx, cqe);
		if (status < 0) {
			LOG_ERR("FIFO stream failed: %d", status);
			return;
		}

		report_fifo_batch(buffer);
		rtio_release_buffer(&stream_ctx, buffer, buffer_len);
	}
}

K_THREAD_DEFINE(stream_tid, 4096, stream_thread, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
	int rc;

	rc = STATS_INIT_AND_REG(lis2dh_fifo_stats, STATS_SIZE_32, "lis2dh_fifo_stats");
	if (rc < 0) {
		LOG_ERR("Cannot register LIS2DH FIFO stats: %d", rc);
	}

	/* Start advertising so a remote client can reach the SMP service. */
	start_smp_bluetooth_adverts();

	LOG_INF("LIS2DH FIFO debug firmware ready");

	return 0;
}
