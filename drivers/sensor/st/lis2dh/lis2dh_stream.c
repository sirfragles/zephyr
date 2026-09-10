/* ST Microelectronics LIS2DH 3-axis accelerometer driver
 *
 * Copyright (c) 2026 Mateusz Zerebecki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/util.h>

#include "lis2dh.h"
#include "lis2dh_decoder.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(lis2dh, CONFIG_SENSOR_LOG_LEVEL);

static int lis2dh_fifo_set_mode(const struct device *dev, uint8_t mode)
{
	const struct lis2dh_config *cfg = dev->config;
	struct lis2dh_data *data = dev->data;
	uint8_t value = mode;

	if (mode != LIS2DH_FIFO_MODE_BYPASS) {
		value |= (cfg->fifo_watermark - 1U) & LIS2DH_FIFO_THRESHOLD_MASK;
	}

	return data->hw_tf->write_reg(dev, LIS2DH_REG_FIFO_CTRL, value);
}

static int lis2dh_fifo_stop(const struct device *dev)
{
	struct lis2dh_data *data = dev->data;
	int rc;
	int first_error = 0;

	lis2dh_setup_int1(dev, false);

	rc = data->hw_tf->update_reg(dev, LIS2DH_REG_CTRL3, LIS2DH_FIFO_INT1_MASK, 0U);
	if (rc < 0) {
		first_error = rc;
	}

	rc = lis2dh_fifo_set_mode(dev, LIS2DH_FIFO_MODE_BYPASS);
	if (rc < 0 && first_error == 0) {
		first_error = rc;
	}

	rc = data->hw_tf->update_reg(dev, LIS2DH_REG_CTRL5, LIS2DH_FIFO_EN_BIT, 0U);
	if (rc < 0 && first_error == 0) {
		first_error = rc;
	}

	data->streaming_sqe = NULL;
	data->fifo_irq_mask = 0U;
	data->fifo_stream_active = false;

	return first_error;
}

static int lis2dh_fifo_flush(const struct device *dev)
{
	int rc;

	rc = lis2dh_fifo_set_mode(dev, LIS2DH_FIFO_MODE_BYPASS);
	if (rc < 0) {
		return rc;
	}

	return lis2dh_fifo_set_mode(dev, LIS2DH_FIFO_MODE_STREAM);
}

static int lis2dh_fifo_configure(const struct device *dev, uint8_t irq_mask)
{
	struct lis2dh_data *data = dev->data;
	int rc;

	if (data->fifo_stream_active && data->fifo_irq_mask == irq_mask) {
		return 0;
	}

	if (data->handler_drdy != NULL) {
		return -EBUSY;
	}

	lis2dh_setup_int1(dev, false);

	rc = data->hw_tf->update_reg(dev, LIS2DH_REG_CTRL3, LIS2DH_FIFO_INT1_MASK, 0U);
	if (rc < 0) {
		return rc;
	}

	rc = lis2dh_fifo_set_mode(dev, LIS2DH_FIFO_MODE_BYPASS);
	if (rc < 0) {
		return rc;
	}

	rc = data->hw_tf->update_reg(dev, LIS2DH_REG_CTRL5, LIS2DH_FIFO_EN_BIT, LIS2DH_FIFO_EN_BIT);
	if (rc < 0) {
		return rc;
	}

	rc = lis2dh_fifo_set_mode(dev, LIS2DH_FIFO_MODE_STREAM);
	if (rc < 0) {
		return rc;
	}

	rc = data->hw_tf->update_reg(dev, LIS2DH_REG_CTRL3, LIS2DH_FIFO_INT1_MASK, irq_mask);
	if (rc < 0) {
		return rc;
	}

	data->fifo_irq_mask = irq_mask;
	data->fifo_stream_active = true;
	return 0;
}

static int lis2dh_stream_irq_mask(const struct sensor_read_config *cfg, uint8_t *irq_mask)
{
	bool has_watermark = false;
	bool has_full = false;

	*irq_mask = 0U;
	for (size_t i = 0U; i < cfg->count; i++) {
		switch (cfg->triggers[i].trigger) {
		case SENSOR_TRIG_FIFO_WATERMARK:
			if (has_watermark) {
				return -EINVAL;
			}
			has_watermark = true;
			*irq_mask |= LIS2DH_EN_FIFO_WTM_INT1;
			break;
		case SENSOR_TRIG_FIFO_FULL:
			if (has_full) {
				return -EINVAL;
			}
			has_full = true;
			*irq_mask |= LIS2DH_EN_FIFO_OVRN_INT1;
			break;
		default:
			return -ENOTSUP;
		}
	}

	return *irq_mask != 0U ? 0 : -EINVAL;
}

static struct sensor_stream_trigger *lis2dh_stream_get_trigger(const struct sensor_read_config *cfg,
							       enum sensor_trigger_type trigger)
{
	for (size_t i = 0U; i < cfg->count; i++) {
		if (cfg->triggers[i].trigger == trigger) {
			return &cfg->triggers[i];
		}
	}

	return NULL;
}

static void lis2dh_fifo_header_fill(const struct device *dev, struct lis2dh_fifo_data *encoded,
				    uint8_t fifo_src, uint8_t sample_count)
{
	struct lis2dh_data *data = dev->data;

	memset(encoded, 0, sizeof(*encoded));
	encoded->header.timestamp = data->fifo_timestamp;
	encoded->header.accel_scale = data->scale;
	encoded->header.is_fifo = 1U;
	encoded->header.range = data->range;
	encoded->header.mode = LIS2DH_OPER_MODE_IDX;
	encoded->header.odr = data->odr;
	encoded->header.int_status = fifo_src;
	encoded->sample_count = sample_count;
}

static void lis2dh_stream_complete(const struct device *dev, int result)
{
	struct lis2dh_data *data = dev->data;
	struct rtio_iodev_sqe *iodev_sqe = data->streaming_sqe;

	data->streaming_sqe = NULL;
	if (result < 0) {
		(void)lis2dh_fifo_stop(dev);
		rtio_iodev_sqe_err(iodev_sqe, result);
	} else {
		rtio_iodev_sqe_ok(iodev_sqe, result);
	}
}

void lis2dh_submit_stream(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct lis2dh_config *config = dev->config;
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	struct lis2dh_data *data = dev->data;
	uint8_t irq_mask;
	int rc;

	if (config->gpio_drdy.port == NULL) {
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
		return;
	}

	if (FIELD_GET(RTIO_SQE_CANCELED, iodev_sqe->sqe.flags) != 0U) {
		data->streaming_sqe = iodev_sqe;
		(void)lis2dh_fifo_stop(dev);
		rtio_iodev_sqe_ok(iodev_sqe, 0);
		return;
	}

	rc = lis2dh_stream_irq_mask(cfg, &irq_mask);
	if (rc < 0) {
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	lis2dh_setup_int1(dev, false);
	rc = lis2dh_fifo_configure(dev, irq_mask);
	if (rc < 0) {
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	data->streaming_sqe = iodev_sqe;
	lis2dh_setup_int1(dev, true);
}

void lis2dh_stream_irq_handler(const struct device *dev)
{
	struct lis2dh_data *data = dev->data;
	struct rtio_iodev_sqe *iodev_sqe = data->streaming_sqe;
	const struct sensor_read_config *cfg;
	struct sensor_stream_trigger *watermark_cfg;
	struct sensor_stream_trigger *full_cfg;
	struct lis2dh_fifo_data *encoded;
	enum sensor_stream_data_opt data_opt;
	uint8_t fifo_src;
	uint8_t sample_count;
	uint8_t *buf;
	uint32_t buf_len;
	uint64_t cycles;
	bool watermark_event;
	bool full_event;
	int rc;

	if (iodev_sqe == NULL) {
		return;
	}

	if (FIELD_GET(RTIO_SQE_CANCELED, iodev_sqe->sqe.flags) != 0U) {
		(void)lis2dh_fifo_stop(dev);
		rtio_iodev_sqe_ok(iodev_sqe, 0);
		return;
	}

	rc = sensor_clock_get_cycles(&cycles);
	if (rc != 0) {
		lis2dh_stream_complete(dev, rc);
		return;
	}
	data->fifo_timestamp = sensor_clock_cycles_to_ns(cycles);

	rc = data->hw_tf->read_reg(dev, LIS2DH_REG_FIFO_SRC, &fifo_src);
	if (rc < 0) {
		lis2dh_stream_complete(dev, rc);
		return;
	}

	cfg = iodev_sqe->sqe.iodev->data;
	watermark_cfg = lis2dh_stream_get_trigger(cfg, SENSOR_TRIG_FIFO_WATERMARK);
	full_cfg = lis2dh_stream_get_trigger(cfg, SENSOR_TRIG_FIFO_FULL);
	watermark_event = watermark_cfg != NULL && (fifo_src & LIS2DH_FIFO_SRC_WTM) != 0U;
	full_event = full_cfg != NULL && (fifo_src & LIS2DH_FIFO_SRC_OVRN) != 0U;

	if (!watermark_event && !full_event) {
		lis2dh_setup_int1(dev, true);
		return;
	}

	if (watermark_event && full_event) {
		data_opt = MIN(watermark_cfg->opt, full_cfg->opt);
	} else if (watermark_event) {
		data_opt = watermark_cfg->opt;
	} else {
		data_opt = full_cfg->opt;
	}

	if (data_opt != SENSOR_STREAM_DATA_INCLUDE) {
		rc = rtio_sqe_rx_buf(iodev_sqe, sizeof(*encoded), sizeof(*encoded), &buf, &buf_len);
		if (rc < 0) {
			lis2dh_stream_complete(dev, rc);
			return;
		}

		encoded = (struct lis2dh_fifo_data *)buf;
		lis2dh_fifo_header_fill(dev, encoded, fifo_src, 0U);

		if (data_opt == SENSOR_STREAM_DATA_DROP) {
			rc = lis2dh_fifo_flush(dev);
			if (rc < 0) {
				lis2dh_stream_complete(dev, rc);
				return;
			}
		}

		lis2dh_stream_complete(dev, 0);
		return;
	}

	sample_count = (fifo_src & LIS2DH_FIFO_SRC_OVRN) != 0U
			       ? LIS2DH_FIFO_DEPTH
			       : fifo_src & LIS2DH_FIFO_SRC_FSS_MASK;
	if ((fifo_src & LIS2DH_FIFO_SRC_EMPTY) != 0U) {
		sample_count = 0U;
	}

	size_t required_len = sizeof(*encoded) + sample_count * LIS2DH_FIFO_SAMPLE_SIZE;

	rc = rtio_sqe_rx_buf(iodev_sqe, required_len, required_len, &buf, &buf_len);
	if (rc < 0) {
		lis2dh_stream_complete(dev, rc);
		return;
	}

	encoded = (struct lis2dh_fifo_data *)buf;
	lis2dh_fifo_header_fill(dev, encoded, fifo_src, sample_count);

	if (sample_count != 0U) {
		/*
		 * AN5005 section 9.5: a single multi-byte read rolls over from
		 * OUT_Z_H to OUT_X_L and drains complete XYZ sample sets.
		 */
		rc = data->hw_tf->read_data(dev, LIS2DH_REG_ACCEL_X_LSB, encoded->samples,
					    sample_count * LIS2DH_FIFO_SAMPLE_SIZE);
		if (rc < 0) {
			lis2dh_stream_complete(dev, rc);
			return;
		}
	}

	lis2dh_stream_complete(dev, 0);
}
