/*
 * Copyright (c) 2026 Mateusz Zerebecki
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "wheel_csc.h"

#include <errno.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(wheel_csc, LOG_LEVEL_INF);

#define CSC_FEATURE_WHEEL_REVOLUTION BIT(0)
#define CSC_FLAG_WHEEL_REVOLUTION    BIT(0)
#define CSC_LOCATION_FRONT_HUB       0x09

static bool notifications_enabled;
static struct k_work advertise_work;

static void csc_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notifications_enabled = value == BT_GATT_CCC_NOTIFY;
}

static ssize_t read_csc_feature(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				uint16_t len, uint16_t offset)
{
	const uint16_t feature = sys_cpu_to_le16(CSC_FEATURE_WHEEL_REVOLUTION);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &feature, sizeof(feature));
}

static ssize_t read_sensor_location(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    void *buf, uint16_t len, uint16_t offset)
{
	const uint8_t location = CSC_LOCATION_FRONT_HUB;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &location, sizeof(location));
}

BT_GATT_SERVICE_DEFINE(wheel_csc_service, BT_GATT_PRIMARY_SERVICE(BT_UUID_CSC),
		       BT_GATT_CHARACTERISTIC(BT_UUID_CSC_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_NONE, NULL, NULL, NULL),
		       BT_GATT_CCC(csc_ccc_changed,
				   BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
		       BT_GATT_CHARACTERISTIC(BT_UUID_CSC_FEATURE, BT_GATT_CHRC_READ,
					      BT_GATT_PERM_READ, read_csc_feature, NULL, NULL),
		       BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LOCATION, BT_GATT_CHRC_READ,
					      BT_GATT_PERM_READ, read_sensor_location, NULL, NULL));

struct csc_measurement {
	uint8_t flags;
	uint32_t cumulative_wheel_revolutions;
	uint16_t last_wheel_event_time;
} __packed;

void wheel_csc_publish(uint32_t cumulative_revolutions, uint16_t last_event_time_1024)
{
	struct csc_measurement measurement = {
		.flags = CSC_FLAG_WHEEL_REVOLUTION,
		.cumulative_wheel_revolutions = sys_cpu_to_le32(cumulative_revolutions),
		.last_wheel_event_time = sys_cpu_to_le16(last_event_time_1024),
	};
	int err;

	if (!notifications_enabled) {
		return;
	}

	err = bt_gatt_notify(NULL, &wheel_csc_service.attrs[2], &measurement, sizeof(measurement));
	if (err != 0 && err != -ENOTCONN) {
		LOG_WRN("CSC notification failed: %d", err);
	}
}

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_CSC_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, SMP_BT_SVC_UUID_VAL),
};

static const struct bt_data scan_response_data[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void advertise(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, advertising_data, ARRAY_SIZE(advertising_data),
			      scan_response_data, ARRAY_SIZE(scan_response_data));
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("Advertising failed: %d", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	int rc;

	if (err != 0U) {
		LOG_WRN("Connection failed: 0x%02x %s", err, bt_hci_err_to_str(err));
		k_work_submit(&advertise_work);
		return;
	}

	LOG_INF("Connected");
	rc = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (rc != 0 && rc != -EALREADY) {
		LOG_WRN("Could not request encrypted link: %d", rc);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	LOG_INF("Disconnected: 0x%02x %s", reason, bt_hci_err_to_str(reason));
}

static void recycled(void)
{
	k_work_submit(&advertise_work);
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	ARG_UNUSED(conn);
	if (err == BT_SECURITY_ERR_SUCCESS) {
		LOG_INF("Link security level %u", level);
	} else {
		LOG_WRN("Security failed: %s", bt_security_err_to_str(err));
	}
}

BT_CONN_CB_DEFINE(wheel_conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
	.security_changed = security_changed,
};

int wheel_csc_start(void)
{
	int err;

	k_work_init(&advertise_work, advertise);
	err = bt_enable(NULL);
	if (err != 0) {
		return err;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_load();
		if (err != 0) {
			return err;
		}
	}

	/* Replace this placeholder after the battery measurement circuit is known. */
	bt_bas_set_battery_level(100U);
	k_work_submit(&advertise_work);
	return 0;
}
