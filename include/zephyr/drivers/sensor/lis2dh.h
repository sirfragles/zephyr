/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Header file for extended sensor API of LIS2DH sensor
 * @ingroup lis2dh_interface
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_LIS2DH_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_LIS2DH_H_

/**
 * @defgroup lis2dh_interface LIS2DH
 * @ingroup sensor_interface_ext_st
 * @brief ST Microelectronics LIS2DH 3-axis accelerometer
 * @{
 */

#include <zephyr/drivers/sensor.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief A single LIS2DH FIFO sample.
 */
struct lis2dh_fifo_sample {
	struct sensor_value accel[3];
	uint64_t timestamp_ns;
};

/**
 * @brief Start hardware FIFO streaming.
 *
 * @param dev LIS2DH device.
 *
 * @retval 0 FIFO streaming started.
 * @retval -EBUSY INT1 or FIFO streaming is already in use.
 * @retval -ENOTSUP FIFO support or an INT1 GPIO is unavailable.
 * @retval <0 Bus or GPIO error.
 */
int lis2dh_fifo_start(const struct device *dev);

/**
 * @brief Stop hardware FIFO streaming and discard queued samples.
 *
 * @param dev LIS2DH device.
 *
 * @retval 0 FIFO streaming stopped.
 * @retval <0 Bus or GPIO error.
 */
int lis2dh_fifo_stop(const struct device *dev);

/**
 * @brief Read samples drained from the hardware FIFO.
 *
 * @param dev LIS2DH device.
 * @param samples Destination sample array.
 * @param capacity Number of entries in @p samples.
 * @param count Number of samples copied to @p samples.
 *
 * @retval 0 Samples copied successfully.
 * @retval -EACCES FIFO streaming is not active.
 * @retval -EINVAL A required argument is NULL.
 */
int lis2dh_fifo_read(const struct device *dev, struct lis2dh_fifo_sample *samples,
		     size_t capacity, size_t *count);

/**
 * Possible values for @ref SENSOR_ATTR_LIS2DH_SELF_TEST custom attribute.
 */
enum lis2dh_self_test {
	LIS2DH_SELF_TEST_DISABLE = 0,  /**< Self-test disabled */
	LIS2DH_SELF_TEST_POSITIVE = 1, /**< Simulates a positive-direction acceleration */
	LIS2DH_SELF_TEST_NEGATIVE = 2, /**< Simulates a negative-direction acceleration */
};

/**
 * @brief Custom sensor attributes for LIS2DH
 */
enum sensor_attribute_lis2dh {
	/**
	 * Sets the self-test mode.
	 *
	 * Applies an electrostatic force to the sensor to simulate acceleration without
	 * actually moving the device.
	 *
	 * Use a value from @ref lis2dh_self_test, passed in the sensor_value.val1 field.
	 */
	SENSOR_ATTR_LIS2DH_SELF_TEST = SENSOR_ATTR_PRIV_START,
	/** Number of samples overwritten in the software FIFO queue. */
	SENSOR_ATTR_LIS2DH_FIFO_DROPPED,
};

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_LIS2DH_H_ */
