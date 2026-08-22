/*
	Copyright 2026 VESC Express contributors

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#ifndef MAIN_BLE_BLE_CLIENT_H_
#define MAIN_BLE_BLE_CLIENT_H_

#include <stdint.h>
#include <stdbool.h>
#include "sdkconfig.h"

#if !CONFIG_BT_BLUEDROID_ENABLED

typedef enum {
	BLE_CLIENT_DISABLED = 0,
} ble_client_result_t;

static inline bool ble_client_started(void) { return false; }

#else

#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"

// Maximum number of simultaneous outgoing (central-role) connections, e.g.
// one heart rate monitor and one BMS.
#define BLE_CLIENT_MAX_CONN 3

// Maximum length of a value returned from ble_client_read.
#define BLE_CLIENT_MAX_VAL_LEN 128

typedef enum {
	BLE_CLIENT_OK              = 0,
	/** Generic/unexpected error. */
	BLE_CLIENT_ERROR           = 1,
	/** ble_client_start (implicitly called by ble-start-app) has not run. */
	BLE_CLIENT_NOT_STARTED     = 2,
	/** Waiting for a result timed out. */
	BLE_CLIENT_TIMEOUT         = 3,
	/** conn_handle didn't refer to a connection that is currently open. */
	BLE_CLIENT_NOT_CONNECTED   = 4,
	/** The requested service and/or characteristic could not be found on the
	 * remote device. */
	BLE_CLIENT_NOT_FOUND       = 5,
	/** All connection slots (BLE_CLIENT_MAX_CONN) are in use. */
	BLE_CLIENT_TOO_MANY_CONN   = 6,
	/** The underlying ESP BLE/GATT API returned an error. */
	BLE_CLIENT_ESP_ERROR       = 7,
	/** The remote GATT operation itself failed (e.g. write rejected). */
	BLE_CLIENT_GATT_ERROR      = 8,
	/** A value didn't fit in the provided/available buffer. */
	BLE_CLIENT_TOO_LONG        = 9,
} ble_client_result_t;

typedef struct {
	esp_bd_addr_t addr;
	esp_ble_addr_type_t addr_type;
	int8_t rssi;
	bool has_name;
	char name[32];
} ble_client_scan_result_t;

typedef void (*ble_client_scan_result_cb_t)(const ble_client_scan_result_t *result);
// connected is true when the connection was (re)established, false on
// disconnect (either requested or due to a link loss).
typedef void (*ble_client_connect_cb_t)(int conn_handle, bool connected);
typedef void (*ble_client_notify_cb_t)(
	int conn_handle, uint16_t char_handle, uint16_t len,
	const uint8_t value[len]
);

/**
 * Bring up the BLE client (central) role. This piggybacks on the same
 * Bluedroid stack instance used by the custom BLE GATT server, and is
 * called automatically from custom_ble_start() - there is normally no need
 * to call this directly.
 */
ble_client_result_t ble_client_start(void);
bool ble_client_started(void);

/**
 * Register callbacks used to report asynchronous events. Pass NULL to
 * unregister. Only a single callback of each kind can be registered at a
 * time (registering a new one replaces the old one).
 */
void ble_client_set_scan_result_cb(ble_client_scan_result_cb_t cb);
void ble_client_set_connect_cb(ble_client_connect_cb_t cb);
void ble_client_set_notify_cb(ble_client_notify_cb_t cb);

/**
 * Start scanning for advertising BLE devices. Results are reported via the
 * scan result callback as they arrive. Scanning stops automatically after
 * duration_s seconds, or can be stopped early with ble_client_scan_stop.
 */
ble_client_result_t ble_client_scan_start(uint32_t duration_s);
ble_client_result_t ble_client_scan_stop(void);
bool ble_client_scanning(void);

/**
 * Open a connection to a peripheral. Blocks until the connection is
 * established, fails, or times out (~10s).
 *
 * @return A connection handle (>= 0) that identifies this connection in all
 * other ble_client_* calls, or a negative value on failure - the failure
 * reason is (-result) as a ble_client_result_t.
 */
int ble_client_connect(const esp_bd_addr_t addr, esp_ble_addr_type_t addr_type);
ble_client_result_t ble_client_disconnect(int conn_handle);
bool ble_client_is_connected(int conn_handle);

/**
 * Write to a characteristic, identified by the UUID of the service it
 * belongs to and its own UUID. The first access to a given service on a
 * given connection triggers (blocking) service discovery; the resulting
 * handle range is cached for the lifetime of the connection.
 */
ble_client_result_t ble_client_write(
	int conn_handle, esp_bt_uuid_t service_uuid, esp_bt_uuid_t char_uuid,
	uint16_t len, const uint8_t data[len], bool need_rsp
);

/**
 * Read the current value of a characteristic. *inout_len should be set to
 * the capacity of out_data on entry, and is set to the actual length of the
 * read value on success.
 */
ble_client_result_t ble_client_read(
	int conn_handle, esp_bt_uuid_t service_uuid, esp_bt_uuid_t char_uuid,
	uint8_t *out_data, uint16_t *inout_len
);

/**
 * Enable notifications/indications for a characteristic. Received values
 * are subsequently reported via the notify callback, tagged with the
 * characteristic's ATT handle (not its UUID, since that's cheaper to match
 * against in the callback and there is no ambiguity within one connection).
 * *out_handle (if non-NULL) is set to that ATT handle on success, so the
 * caller can match it against later notifications without having to
 * resolve the UUID again.
 */
ble_client_result_t ble_client_subscribe(
	int conn_handle, esp_bt_uuid_t service_uuid, esp_bt_uuid_t char_uuid,
	uint16_t *out_handle
);

// Internal: called by custom_ble.c's shared GAP callback to forward events
// that belong to the client (scanning) role rather than the GATT server
// (advertising) role.
void ble_client_handle_gap_event(
	esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param
);

#endif

#endif /* MAIN_BLE_BLE_CLIENT_H_ */
