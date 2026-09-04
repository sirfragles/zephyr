/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_lis2dh

#include <zephyr/drivers/sensor/lis2dh.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "lis2dh.h"

LOG_MODULE_DECLARE(lis2dh, CONFIG_SENSOR_LOG_LEVEL);

#define LIS2DH_NSEC_PER_SEC 1000000000ULL

static int lis2dh_fifo_period_ns(const struct device *dev, uint64_t *period_ns)
{
	struct lis2dh_data *lis2dh = dev->data;
	uint32_t frequency_millihz;
	uint8_t ctrl1;
	uint8_t odr;
	int status;

	status = lis2dh->hw_tf->read_reg(dev, LIS2DH_REG_CTRL1, &ctrl1);
	if (status < 0) {
		return status;
	}

	odr = (ctrl1 & LIS2DH_ODR_MASK) >> LIS2DH_ODR_SHIFT;
	switch (odr) {
	case LIS2DH_ODR_1:
		frequency_millihz = 1000U;
		break;
	case LIS2DH_ODR_2:
		frequency_millihz = 10000U;
		break;
	case LIS2DH_ODR_3:
		frequency_millihz = 25000U;
		break;
	case LIS2DH_ODR_4:
		frequency_millihz = 50000U;
		break;
	case LIS2DH_ODR_5:
		frequency_millihz = 100000U;
		break;
	case LIS2DH_ODR_6:
		frequency_millihz = 200000U;
		break;
	case LIS2DH_ODR_7:
		frequency_millihz = 400000U;
		break;
	case LIS2DH_ODR_8:
		frequency_millihz = 1620000U;
		break;
	case LIS2DH_ODR_9:
		frequency_millihz = (ctrl1 & LIS2DH_LP_EN_BIT_MASK) != 0U ?
			5376000U : 1344000U;
		break;
	default:
		return -EINVAL;
	}

	*period_ns = (LIS2DH_NSEC_PER_SEC * 1000U) / frequency_millihz;

	return 0;
}

static void lis2dh_fifo_clear(struct lis2dh_data *lis2dh)
{
	lis2dh->fifo_head = 0U;
	lis2dh->fifo_tail = 0U;
	lis2dh->fifo_count = 0U;
	lis2dh->fifo_dropped_samples = 0U;
	lis2dh->fifo_cache_valid = false;
}

static void lis2dh_fifo_push(struct lis2dh_data *lis2dh, const int16_t xyz[3],
			     uint64_t timestamp_ns)
{
	uint16_t head = lis2dh->fifo_head;

	memcpy(lis2dh->fifo_samples[head].xyz, xyz, sizeof(lis2dh->fifo_samples[head].xyz));
	lis2dh->fifo_samples[head].timestamp_ns = timestamp_ns;
	lis2dh->fifo_head = (head + 1U) % CONFIG_LIS2DH_FIFO_SW_QUEUE_SAMPLES;

	if (lis2dh->fifo_count == CONFIG_LIS2DH_FIFO_SW_QUEUE_SAMPLES) {
		lis2dh->fifo_tail = (lis2dh->fifo_tail + 1U) %
				     CONFIG_LIS2DH_FIFO_SW_QUEUE_SAMPLES;
		lis2dh->fifo_dropped_samples++;
	} else {
		lis2dh->fifo_count++;
	}
}

static void lis2dh_fifo_convert(int16_t raw, uint32_t scale, struct sensor_value *value)
{
	int32_t converted;

	converted = (raw >> 4) * scale;
	value->val1 = converted / 1000000;
	value->val2 = converted % 1000000;
}

int lis2dh_fifo_init(const struct device *dev)
{
	struct lis2dh_data *lis2dh = dev->data;

	k_mutex_init(&lis2dh->fifo_lock);
	lis2dh_fifo_clear(lis2dh);
	atomic_clear(&lis2dh->fifo_active);

	return 0;
}

bool lis2dh_fifo_is_active(const struct device *dev)
{
	const struct lis2dh_data *lis2dh = dev->data;

	return atomic_get(&lis2dh->fifo_active) != 0;
}

void lis2dh_fifo_irq_timestamp(const struct device *dev)
{
	struct lis2dh_data *lis2dh = dev->data;

	lis2dh->fifo_irq_timestamp_ns = k_cyc_to_ns_floor64(k_cycle_get_64());
}

int lis2dh_fifo_start(const struct device *dev)
{
	const struct lis2dh_config *cfg = dev->config;
	struct lis2dh_data *lis2dh = dev->data;
	uint8_t ctrl3;
	int status;

	if (cfg->gpio_drdy.port == NULL) {
		return -ENOTSUP;
	}

	(void)k_mutex_lock(&lis2dh->fifo_lock, K_FOREVER);

	if (lis2dh_fifo_is_active(dev)) {
		status = -EBUSY;
		goto unlock;
	}

	if (lis2dh->handler_drdy != NULL) {
		status = -EBUSY;
		goto unlock;
	}

	status = lis2dh->hw_tf->read_reg(dev, LIS2DH_REG_CTRL3, &ctrl3);
	if (status < 0) {
		goto unlock;
	}

	if ((ctrl3 & (LIS2DH_EN_CLICK_INT1 | LIS2DH_EN_IA_INT1 |
		      LIS2DH_EN_DRDY1_INT1)) != 0U) {
		status = -EBUSY;
		goto unlock;
	}

	status = lis2dh_fifo_period_ns(dev, &lis2dh->fifo_period_ns);
	if (status < 0) {
		goto unlock;
	}

	status = lis2dh_trigger_int1_set(dev, false);
	if (status < 0) {
		goto unlock;
	}

	status = lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_FIFO_CTRL,
					  LIS2DH_FIFO_MODE_BYPASS);
	if (status < 0) {
		goto unlock;
	}

	status = lis2dh->hw_tf->update_reg(dev, LIS2DH_REG_CTRL5, LIS2DH_EN_FIFO,
					   LIS2DH_EN_FIFO);
	if (status < 0) {
		goto disable_fifo;
	}

	status = lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_FIFO_CTRL,
					  LIS2DH_FIFO_MODE_STREAM |
					  (cfg->fifo_watermark - 1U));
	if (status < 0) {
		goto disable_fifo;
	}

	status = lis2dh->hw_tf->update_reg(dev, LIS2DH_REG_CTRL3,
					   LIS2DH_EN_FIFO_WTM_INT1 |
					   LIS2DH_EN_FIFO_OVRN_INT1,
					   LIS2DH_EN_FIFO_WTM_INT1 |
					   LIS2DH_EN_FIFO_OVRN_INT1);
	if (status < 0) {
		goto disable_fifo;
	}

	lis2dh_fifo_clear(lis2dh);
	atomic_set(&lis2dh->fifo_active, 1);
	lis2dh_fifo_irq_timestamp(dev);
	status = lis2dh_trigger_int1_set(dev, true);
	if (status < 0) {
		atomic_clear(&lis2dh->fifo_active);
		goto disable_fifo;
	}

	goto unlock;

disable_fifo:
	(void)lis2dh->hw_tf->update_reg(dev, LIS2DH_REG_CTRL3,
					 LIS2DH_EN_FIFO_WTM_INT1 |
					 LIS2DH_EN_FIFO_OVRN_INT1, 0U);
	(void)lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_FIFO_CTRL, LIS2DH_FIFO_MODE_BYPASS);
	(void)lis2dh->hw_tf->update_reg(dev, LIS2DH_REG_CTRL5, LIS2DH_EN_FIFO, 0U);

unlock:
	(void)k_mutex_unlock(&lis2dh->fifo_lock);

	return status;
}

int lis2dh_fifo_stop(const struct device *dev)
{
	struct lis2dh_data *lis2dh = dev->data;
	int first_error = 0;
	int status;

	(void)k_mutex_lock(&lis2dh->fifo_lock, K_FOREVER);

	if (!lis2dh_fifo_is_active(dev)) {
		goto unlock;
	}

	atomic_clear(&lis2dh->fifo_active);
	status = lis2dh_trigger_int1_set(dev, false);
	if (status < 0) {
		first_error = status;
	}

	status = lis2dh->hw_tf->update_reg(dev, LIS2DH_REG_CTRL3,
					   LIS2DH_EN_FIFO_WTM_INT1 |
					   LIS2DH_EN_FIFO_OVRN_INT1, 0U);
	if (status < 0 && first_error == 0) {
		first_error = status;
	}

	status = lis2dh->hw_tf->write_reg(dev, LIS2DH_REG_FIFO_CTRL, LIS2DH_FIFO_MODE_BYPASS);
	if (status < 0 && first_error == 0) {
		first_error = status;
	}

	status = lis2dh->hw_tf->update_reg(dev, LIS2DH_REG_CTRL5, LIS2DH_EN_FIFO, 0U);
	if (status < 0 && first_error == 0) {
		first_error = status;
	}

	lis2dh_fifo_clear(lis2dh);

unlock:
	(void)k_mutex_unlock(&lis2dh->fifo_lock);

	return first_error;
}

int lis2dh_fifo_sample_fetch(const struct device *dev)
{
	struct lis2dh_data *lis2dh = dev->data;
	int status = 0;

	(void)k_mutex_lock(&lis2dh->fifo_lock, K_FOREVER);
	if (!lis2dh->fifo_cache_valid) {
		status = -ENODATA;
	}
	(void)k_mutex_unlock(&lis2dh->fifo_lock);

	return status;
}

int lis2dh_fifo_cache_copy(const struct device *dev, union lis2dh_sample *sample)
{
	struct lis2dh_data *lis2dh = dev->data;
	int status = 0;

	(void)k_mutex_lock(&lis2dh->fifo_lock, K_FOREVER);
	if (!lis2dh_fifo_is_active(dev) || !lis2dh->fifo_cache_valid) {
		status = -ENODATA;
	} else {
		memcpy(sample, &lis2dh->sample, sizeof(*sample));
	}
	(void)k_mutex_unlock(&lis2dh->fifo_lock);

	return status;
}

int lis2dh_fifo_read(const struct device *dev, struct lis2dh_fifo_sample *samples,
		     size_t capacity, size_t *count)
{
	struct lis2dh_data *lis2dh = dev->data;
	size_t sample_count;
	size_t i;

	if (count == NULL || (capacity > 0U && samples == NULL)) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&lis2dh->fifo_lock, K_FOREVER);
	if (!lis2dh_fifo_is_active(dev)) {
		(void)k_mutex_unlock(&lis2dh->fifo_lock);
		return -EACCES;
	}

	sample_count = MIN(capacity, lis2dh->fifo_count);
	for (i = 0U; i < sample_count; i++) {
		uint16_t tail = lis2dh->fifo_tail;
		size_t axis;

		for (axis = 0U; axis < ARRAY_SIZE(samples[i].accel); axis++) {
			lis2dh_fifo_convert(lis2dh->fifo_samples[tail].xyz[axis], lis2dh->scale,
					    &samples[i].accel[axis]);
		}
		samples[i].timestamp_ns = lis2dh->fifo_samples[tail].timestamp_ns;
		lis2dh->fifo_tail = (tail + 1U) % CONFIG_LIS2DH_FIFO_SW_QUEUE_SAMPLES;
		lis2dh->fifo_count--;
	}
	*count = sample_count;

	(void)k_mutex_unlock(&lis2dh->fifo_lock);

	return 0;
}

int lis2dh_fifo_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			    sensor_trigger_handler_t handler)
{
	struct lis2dh_data *lis2dh = dev->data;

	if (trig->chan != SENSOR_CHAN_ACCEL_XYZ) {
		return -ENOTSUP;
	}

	(void)k_mutex_lock(&lis2dh->fifo_lock, K_FOREVER);
	if (trig->type == SENSOR_TRIG_FIFO_WATERMARK) {
		lis2dh->fifo_handler_watermark = handler;
		lis2dh->fifo_trig_watermark = trig;
	} else if (trig->type == SENSOR_TRIG_FIFO_FULL) {
		lis2dh->fifo_handler_full = handler;
		lis2dh->fifo_trig_full = trig;
	} else {
		(void)k_mutex_unlock(&lis2dh->fifo_lock);
		return -ENOTSUP;
	}
	(void)k_mutex_unlock(&lis2dh->fifo_lock);

	return 0;
}

int lis2dh_fifo_handle_irq(const struct device *dev)
{
	struct lis2dh_data *lis2dh = dev->data;
	sensor_trigger_handler_t handler = NULL;
	const struct sensor_trigger *trig = NULL;
	uint8_t raw[LIS2DH_FIFO_MAX_BYTES];
	uint8_t src;
	uint8_t sample_count;
	uint64_t timestamp_ns;
	bool overrun;
	int status;
	size_t i;

	(void)k_mutex_lock(&lis2dh->fifo_lock, K_FOREVER);
	if (!lis2dh_fifo_is_active(dev)) {
		(void)k_mutex_unlock(&lis2dh->fifo_lock);
		return 0;
	}

	status = lis2dh->hw_tf->read_reg(dev, LIS2DH_REG_FIFO_SRC, &src);
	if (status < 0) {
		goto unlock;
	}

	if ((src & LIS2DH_FIFO_EMPTY) != 0U) {
		status = 0;
		goto unlock;
	}

	overrun = (src & LIS2DH_FIFO_OVRN) != 0U;
	sample_count = overrun ? LIS2DH_FIFO_MAX_SAMPLES : src & LIS2DH_FIFO_FSS_MASK;
	if (sample_count == 0U) {
		status = 0;
		goto unlock;
	}

	status = lis2dh->hw_tf->read_data(dev, LIS2DH_REG_ACCEL_X_LSB, raw,
					  sample_count * LIS2DH_FIFO_SAMPLE_SIZE);
	if (status < 0) {
		goto unlock;
	}

	timestamp_ns = lis2dh->fifo_irq_timestamp_ns;
	if (timestamp_ns == 0U) {
		timestamp_ns = k_cyc_to_ns_floor64(k_cycle_get_64());
	}
	timestamp_ns -= (sample_count - 1U) * lis2dh->fifo_period_ns;

	for (i = 0U; i < sample_count; i++) {
		int16_t xyz[3];
		size_t axis;

		for (axis = 0U; axis < ARRAY_SIZE(xyz); axis++) {
			xyz[axis] = (int16_t)sys_get_le16(&raw[i * LIS2DH_FIFO_SAMPLE_SIZE +
								      axis * sizeof(int16_t)]);
		}
		lis2dh_fifo_push(lis2dh, xyz, timestamp_ns);
		timestamp_ns += lis2dh->fifo_period_ns;
	}

	memcpy(lis2dh->sample.xyz, lis2dh->fifo_samples[
		(lis2dh->fifo_head + CONFIG_LIS2DH_FIFO_SW_QUEUE_SAMPLES - 1U) %
		CONFIG_LIS2DH_FIFO_SW_QUEUE_SAMPLES].xyz, sizeof(lis2dh->sample.xyz));
	lis2dh->sample.status = LIS2DH_STATUS_ZYX_DRDY;
	lis2dh->fifo_cache_valid = true;

	if (overrun && lis2dh->fifo_handler_full != NULL) {
		handler = lis2dh->fifo_handler_full;
		trig = lis2dh->fifo_trig_full;
	} else if ((src & LIS2DH_FIFO_WTM) != 0U &&
		   lis2dh->fifo_handler_watermark != NULL) {
		handler = lis2dh->fifo_handler_watermark;
		trig = lis2dh->fifo_trig_watermark;
	}

unlock:
	(void)k_mutex_unlock(&lis2dh->fifo_lock);

	if (status == 0 && handler != NULL) {
		handler(dev, trig);
	}

	return status;
}
