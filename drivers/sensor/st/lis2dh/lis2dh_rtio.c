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
#include <zephyr/rtio/work.h>
#include <zephyr/sys/byteorder.h>

#include "lis2dh.h"
#include "lis2dh_decoder.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(lis2dh, CONFIG_SENSOR_LOG_LEVEL);

static int lis2dh_async_temperature_read(const struct device *dev, int32_t *temperature)
{
#ifdef CONFIG_LIS2DH_MEASURE_TEMPERATURE
	const struct lis2dh_config *cfg = dev->config;
	struct lis2dh_data *data = dev->data;
	uint8_t raw[2];
	int32_t fractional = 0;
	int rc;

	rc = data->hw_tf->read_data(dev, cfg->temperature.dout_addr, raw, sizeof(raw));
	if (rc < 0) {
		return rc;
	}

	if (cfg->temperature.fractional_bits != 0U) {
		fractional = raw[0] >> (8U - cfg->temperature.fractional_bits);
		fractional = fractional * 1000000 / BIT(cfg->temperature.fractional_bits);
		if ((int8_t)raw[1] < 0) {
			fractional = -fractional;
		}
	}

	*temperature = (int32_t)(int8_t)raw[1] * 1000000 + fractional;
	return 0;
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(temperature);
	return -ENOTSUP;
#endif
}

static int lis2dh_async_validate_channels(const struct sensor_read_config *cfg, bool *read_accel,
					  bool *read_temp)
{
	*read_accel = false;
	*read_temp = false;

	for (size_t i = 0U; i < cfg->count; i++) {
		switch (cfg->channels[i].chan_type) {
		case SENSOR_CHAN_ACCEL_X:
		case SENSOR_CHAN_ACCEL_Y:
		case SENSOR_CHAN_ACCEL_Z:
		case SENSOR_CHAN_ACCEL_XYZ:
			*read_accel = true;
			break;
		case SENSOR_CHAN_DIE_TEMP:
			if (!IS_ENABLED(CONFIG_LIS2DH_MEASURE_TEMPERATURE)) {
				return -ENOTSUP;
			}
			*read_temp = true;
			break;
		case SENSOR_CHAN_ALL:
			*read_accel = true;
			*read_temp = IS_ENABLED(CONFIG_LIS2DH_MEASURE_TEMPERATURE);
			break;
		default:
			return -ENOTSUP;
		}

		if (cfg->channels[i].chan_idx != 0U) {
			return -ENOTSUP;
		}
	}

	return (*read_accel || *read_temp) ? 0 : -ENOTSUP;
}

static void lis2dh_submit_sample(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	struct lis2dh_data *data = dev->data;
	struct lis2dh_rtio_data *encoded;
	uint8_t *buf;
	uint32_t buf_len;
	uint64_t cycles;
	int32_t temperature = 0;
	bool read_accel;
	bool read_temp;
	int rc;

	rc = lis2dh_async_validate_channels(cfg, &read_accel, &read_temp);
	if (rc < 0) {
		goto error;
	}

	rc = rtio_sqe_rx_buf(iodev_sqe, sizeof(*encoded), sizeof(*encoded), &buf, &buf_len);
	if (rc < 0) {
		goto error;
	}

	encoded = (struct lis2dh_rtio_data *)buf;
	memset(encoded, 0, sizeof(*encoded));

	if (read_accel) {
		uint8_t raw[LIS2DH_BUF_SZ];

		rc = data->hw_tf->read_data(dev, LIS2DH_REG_STATUS, raw, sizeof(raw));
		if (rc < 0) {
			goto error;
		}

		encoded->header.int_status = raw[0];
		memcpy(encoded->accel, &raw[1], sizeof(encoded->accel));
		encoded->has_accel = 1U;
	}

	if (read_temp) {
		rc = lis2dh_async_temperature_read(dev, &temperature);
		if (rc < 0) {
			goto error;
		}

		sys_put_le32((uint32_t)temperature, (uint8_t *)&encoded->temperature);
		encoded->has_temp = 1U;
	}

	rc = sensor_clock_get_cycles(&cycles);
	if (rc < 0) {
		goto error;
	}

	encoded->header.timestamp = sensor_clock_cycles_to_ns(cycles);
	encoded->header.accel_scale = data->scale;
	encoded->header.is_fifo = 0U;
	encoded->header.range = data->range;
	encoded->header.mode = LIS2DH_OPER_MODE_IDX;
	encoded->header.odr = data->odr;

	rtio_iodev_sqe_ok(iodev_sqe, 0);
	return;

error:
	LOG_ERR("Failed to read sensor asynchronously: %d", rc);
	rtio_iodev_sqe_err(iodev_sqe, rc);
}

static void lis2dh_submit_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = cfg->sensor;

	if (!cfg->is_streaming) {
		lis2dh_submit_sample(dev, iodev_sqe);
	} else if (IS_ENABLED(CONFIG_LIS2DH_STREAM)) {
#ifdef CONFIG_LIS2DH_STREAM
		lis2dh_submit_stream(dev, iodev_sqe);
#endif
	} else {
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	}
}

void lis2dh_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_work_req *req;

	ARG_UNUSED(dev);

	req = rtio_work_req_alloc();
	if (req == NULL) {
		LOG_ERR("RTIO work item allocation failed");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, lis2dh_submit_sync);
}
