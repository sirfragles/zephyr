/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_lis2dh

#include <errno.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>

#include "lis2dh.h"
#include "lis2dh_decoder.h"

static int lis2dh_stream_accel_shift(void)
{
	return LIS2DH_FS_IDX + 5;
}

static const struct sensor_stream_trigger *lis2dh_stream_find_trigger(
	const struct sensor_read_config *cfg, enum sensor_trigger_type trigger)
{
	size_t i;

	for (i = 0U; i < cfg->count; i++) {
		if (cfg->triggers[i].trigger == trigger) {
			return &cfg->triggers[i];
		}
	}

	return NULL;
}

static int lis2dh_stream_encode(struct rtio_iodev_sqe *iodev_sqe,
				const struct lis2dh_data *lis2dh,
				enum sensor_trigger_type trigger, const uint8_t *raw,
				uint8_t sample_count, uint64_t timestamp_ns,
				enum sensor_stream_data_opt option)
{
	struct lis2dh_encoded_header *header;
	uint8_t *buffer;
	uint32_t buffer_len;
	uint32_t required_len;
	int status;

	required_len = sizeof(*header);
	if (option == SENSOR_STREAM_DATA_INCLUDE) {
		required_len += sample_count * LIS2DH_ENCODED_SAMPLE_SIZE;
	}

	status = rtio_sqe_rx_buf(iodev_sqe, required_len, required_len, &buffer, &buffer_len);
	if (status < 0) {
		return status;
	}

	header = (struct lis2dh_encoded_header *)buffer;
	header->timestamp_ns = timestamp_ns;
	header->period_ns = lis2dh->fifo_period_ns;
	header->scale = lis2dh->scale;
	header->sample_count = option == SENSOR_STREAM_DATA_INCLUDE ? sample_count : 0U;
	header->trigger = trigger;
	header->shift = lis2dh_stream_accel_shift();
	header->is_fifo = 1U;
	memset(header->reserved, 0, sizeof(header->reserved));

	if (option == SENSOR_STREAM_DATA_INCLUDE) {
		memcpy(buffer + sizeof(*header), raw, sample_count * LIS2DH_ENCODED_SAMPLE_SIZE);
	}

	return 0;
}

void lis2dh_stream_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	struct lis2dh_data *lis2dh = dev->data;
	int status;
	size_t i;

	if (cfg->count == 0U) {
		rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
		return;
	}

	for (i = 0U; i < cfg->count; i++) {
		if (cfg->triggers[i].trigger != SENSOR_TRIG_FIFO_WATERMARK &&
		    cfg->triggers[i].trigger != SENSOR_TRIG_FIFO_FULL) {
			rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
			return;
		}
	}

	(void)k_mutex_lock(&lis2dh->fifo_lock, K_FOREVER);
	if (lis2dh->streaming_sqe != NULL) {
		(void)k_mutex_unlock(&lis2dh->fifo_lock);
		rtio_iodev_sqe_err(iodev_sqe, -EBUSY);
		return;
	}

	lis2dh->streaming_sqe = iodev_sqe;
	if (atomic_get(&lis2dh->stream_active) == 0) {
		/* Reserve the stream before enabling the interrupt source. */
		atomic_set(&lis2dh->stream_active, 1);
		(void)k_mutex_unlock(&lis2dh->fifo_lock);
		status = lis2dh_fifo_start(dev);
		(void)k_mutex_lock(&lis2dh->fifo_lock, K_FOREVER);
		if (status < 0) {
			if (lis2dh->streaming_sqe == iodev_sqe) {
				lis2dh->streaming_sqe = NULL;
				atomic_clear(&lis2dh->stream_active);
				(void)k_mutex_unlock(&lis2dh->fifo_lock);
				rtio_iodev_sqe_err(iodev_sqe, status);
				return;
			}
			(void)k_mutex_unlock(&lis2dh->fifo_lock);
			return;
		}
		if (lis2dh->streaming_sqe != iodev_sqe) {
			(void)k_mutex_unlock(&lis2dh->fifo_lock);
			/* stop() won the race and already completed the SQE */
			(void)lis2dh_fifo_stop(dev);
			return;
		}
	}
	(void)k_mutex_unlock(&lis2dh->fifo_lock);
}

int lis2dh_stream_handle_irq(const struct device *dev, uint8_t fifo_src, const uint8_t *raw,
			     uint8_t sample_count, uint64_t timestamp_ns, bool *drop)
{
	struct lis2dh_data *lis2dh = dev->data;
	struct rtio_iodev_sqe *iodev_sqe = lis2dh->streaming_sqe;
	const struct sensor_read_config *cfg;
	const struct sensor_stream_trigger *stream_trigger;
	enum sensor_trigger_type trigger;
	int status;

	*drop = false;
	if (iodev_sqe == NULL) {
		return 0;
	}

	if (FIELD_GET(RTIO_SQE_CANCELED, iodev_sqe->sqe.flags) != 0U) {
		lis2dh->streaming_sqe = NULL;
		atomic_clear(&lis2dh->stream_active);
		rtio_iodev_sqe_err(iodev_sqe, -ECANCELED);
		return -ECANCELED;
	}

	if ((fifo_src & LIS2DH_FIFO_OVRN) != 0U) {
		trigger = SENSOR_TRIG_FIFO_FULL;
	} else if ((fifo_src & LIS2DH_FIFO_WTM) != 0U) {
		trigger = SENSOR_TRIG_FIFO_WATERMARK;
	} else {
		return 0;
	}

	cfg = iodev_sqe->sqe.iodev->data;
	stream_trigger = lis2dh_stream_find_trigger(cfg, trigger);
	if (stream_trigger == NULL) {
		return 0;
	}

	status = lis2dh_stream_encode(iodev_sqe, lis2dh, trigger, raw, sample_count, timestamp_ns,
				      stream_trigger->opt);
	lis2dh->streaming_sqe = NULL;
	if (status < 0) {
		atomic_clear(&lis2dh->stream_active);
		rtio_iodev_sqe_err(iodev_sqe, status);
		return status;
	}

	*drop = stream_trigger->opt == SENSOR_STREAM_DATA_DROP;
	if (FIELD_GET(RTIO_SQE_CANCELED, iodev_sqe->sqe.flags) != 0U) {
		atomic_clear(&lis2dh->stream_active);
		rtio_iodev_sqe_err(iodev_sqe, -ECANCELED);
		return -ECANCELED;
	}
	rtio_iodev_sqe_ok(iodev_sqe, 0);

	return 0;
}
