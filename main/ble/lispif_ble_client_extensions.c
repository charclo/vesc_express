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

// LispBM bindings for the BLE central (client) role implemented in
// ble_client.c. These are meant to be generic building blocks - device
// specific protocol handling (Polar H10 heart rate, JBD/Xiaoxiang BMS
// packs, ...) is expected to live in .lbm scripts on top of this, see
// lbm_examples/.

#include "lispif_ble_client_extensions.h"

#include <string.h>

#include "esp_bt_defs.h"

#include "ble_client.h"
#include "lispif_events.h"
#include "utils.h"
#include "heap.h"
#include "lbm_defines.h"
#include "lbm_memory.h"
#include "lbm_flat_value.h"
#include "eval_cps.h"
#include "extensions.h"
#include "commands.h"

static char *error_not_started    = "ble-start-app has not been called.";
static char *error_invalid_addr   = "Expected a 6 byte address array.";
static char *error_invalid_uuid   =
	"Expected a uuid as a number (16 bit) or a 2- or 16-byte array.";
static char *error_not_connected  = "conn-handle is not connected.";
static char *error_not_found      = "Service or characteristic not found.";
static char *error_timeout        = "Timed out waiting for a BLE response.";
static char *error_gatt           = "The remote device rejected the operation.";

static bool decode_addr(lbm_value v, esp_bd_addr_t out) {
	if (!lbm_is_array_r(v) || lbm_heap_array_get_size(v) != 6) {
		return false;
	}
	const uint8_t *data = (const uint8_t *)lbm_heap_array_get_data_ro(v);
	if (data == NULL) {
		return false;
	}
	memcpy(out, data, 6);
	return true;
}

static bool decode_uuid(lbm_value v, esp_bt_uuid_t *out) {
	if (lbm_is_number(v)) {
		out->len         = ESP_UUID_LEN_16;
		out->uuid.uuid16 = (uint16_t)lbm_dec_as_u32(v);
		return true;
	}

	if (lbm_is_array_r(v)) {
		unsigned int size   = lbm_heap_array_get_size(v);
		const uint8_t *data = (const uint8_t *)lbm_heap_array_get_data_ro(v);
		if (data == NULL) {
			return false;
		}

		if (size == 2) {
			out->len         = ESP_UUID_LEN_16;
			out->uuid.uuid16 = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
			return true;
		} else if (size == 16) {
			out->len = ESP_UUID_LEN_128;
			memcpy(out->uuid.uuid128, data, 16);
			return true;
		}
	}

	return false;
}

static lbm_value ble_result_to_lbm(ble_client_result_t result, bool ok_value) {
	switch (result) {
		case BLE_CLIENT_OK: {
			return ok_value ? ENC_SYM_TRUE : ENC_SYM_NIL;
		}
		case BLE_CLIENT_NOT_STARTED: {
			lbm_set_error_reason(error_not_started);
			return ENC_SYM_EERROR;
		}
		case BLE_CLIENT_NOT_CONNECTED: {
			lbm_set_error_reason(error_not_connected);
			return ENC_SYM_EERROR;
		}
		case BLE_CLIENT_NOT_FOUND: {
			lbm_set_error_reason(error_not_found);
			return ENC_SYM_NIL;
		}
		case BLE_CLIENT_TIMEOUT: {
			lbm_set_error_reason(error_timeout);
			return ENC_SYM_NIL;
		}
		case BLE_CLIENT_GATT_ERROR: {
			lbm_set_error_reason(error_gatt);
			return ENC_SYM_NIL;
		}
		case BLE_CLIENT_TOO_MANY_CONN:
		case BLE_CLIENT_TOO_LONG:
		case BLE_CLIENT_ESP_ERROR:
		case BLE_CLIENT_ERROR:
		default: {
			return ENC_SYM_EERROR;
		}
	}
}

/**
 * signature: (ble-client-scan-start [seconds:number]) -> bool
 *
 * Starts scanning for advertising BLE devices. Results are reported as
 * (event-ble-scan addr rssi name-or-nil) events - see event-enable.
 * Scanning stops automatically after `seconds` (10 by default), or can be
 * stopped early with ble-client-scan-stop.
 */
static lbm_value ext_ble_client_scan_start(lbm_value *args, lbm_uint argn) {
	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		return ENC_SYM_TERROR;
	}

	uint32_t duration_s = argn == 1 ? (uint32_t)lbm_dec_as_u32(args[0]) : 10;

	return ble_result_to_lbm(ble_client_scan_start(duration_s), true);
}

/**
 * signature: (ble-client-scan-stop) -> bool
 */
static lbm_value ext_ble_client_scan_stop(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return ble_result_to_lbm(ble_client_scan_stop(), true);
}

/**
 * signature: (ble-client-scanning) -> bool
 */
static lbm_value ext_ble_client_scanning(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	return ble_client_scanning() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

/**
 * signature: (ble-client-connect addr:byte-array [addr-type:number]) -> conn-handle | nil
 *
 * addr is the 6 byte device address, as received in an event-ble-scan
 * event. addr-type defaults to 0 (public); use the value from the scan
 * event for devices using a random address (most modern peripherals,
 * including the Polar H10 and most cheap BMS boards, do). Blocks (for up
 * to ~10s) until the connection succeeds or fails.
 */
static lbm_value ext_ble_client_connect(lbm_value *args, lbm_uint argn) {
	if (argn < 1 || argn > 2) {
		return ENC_SYM_EERROR;
	}

	esp_bd_addr_t addr;
	if (!decode_addr(args[0], addr)) {
		lbm_set_error_reason(error_invalid_addr);
		return ENC_SYM_EERROR;
	}

	esp_ble_addr_type_t addr_type = BLE_ADDR_TYPE_PUBLIC;
	if (argn == 2) {
		if (!lbm_is_number(args[1])) {
			return ENC_SYM_TERROR;
		}
		addr_type = (esp_ble_addr_type_t)lbm_dec_as_i32(args[1]);
	}

	int result = ble_client_connect(addr, addr_type);
	if (result < 0) {
		return ble_result_to_lbm((ble_client_result_t)(-result), false);
	}

	return lbm_enc_i(result);
}

/**
 * signature: (ble-client-disconnect conn-handle:number) -> bool
 */
static lbm_value ext_ble_client_disconnect(lbm_value *args, lbm_uint argn) {
	if (argn != 1 || !lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}

	return ble_result_to_lbm(
		ble_client_disconnect(lbm_dec_as_i32(args[0])), true
	);
}

/**
 * signature: (ble-client-connected conn-handle:number) -> bool
 */
static lbm_value ext_ble_client_connected(lbm_value *args, lbm_uint argn) {
	if (argn != 1 || !lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}

	return ble_client_is_connected(lbm_dec_as_i32(args[0])) ? ENC_SYM_TRUE
															 : ENC_SYM_NIL;
}

/**
 * signature: (ble-client-write conn-handle:number service-uuid char-uuid
 *              data:byte-array [need-response:bool]) -> bool
 *
 * service-uuid and char-uuid are each either a number (16 bit uuid) or a 2-
 * or 16-byte array. need-response defaults to true.
 */
static lbm_value ext_ble_client_write(lbm_value *args, lbm_uint argn) {
	if (argn < 4 || argn > 5) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0]) || !lbm_is_array_r(args[3])) {
		return ENC_SYM_TERROR;
	}

	esp_bt_uuid_t service_uuid, char_uuid;
	if (!decode_uuid(args[1], &service_uuid) || !decode_uuid(args[2], &char_uuid)) {
		lbm_set_error_reason(error_invalid_uuid);
		return ENC_SYM_EERROR;
	}

	unsigned int len    = lbm_heap_array_get_size(args[3]);
	const uint8_t *data = (const uint8_t *)lbm_heap_array_get_data_ro(args[3]);
	if (data == NULL) {
		return ENC_SYM_EERROR;
	}

	bool need_rsp = true;
	if (argn == 5) {
		need_rsp = lbm_dec_as_i32(args[4]) != 0;
	}

	ble_client_result_t result = ble_client_write(
		lbm_dec_as_i32(args[0]), service_uuid, char_uuid, (uint16_t)len, data,
		need_rsp
	);

	return ble_result_to_lbm(result, true);
}

/**
 * signature: (ble-client-read conn-handle:number service-uuid char-uuid) -> byte-array | nil
 */
static lbm_value ext_ble_client_read(lbm_value *args, lbm_uint argn) {
	if (argn != 3 || !lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}

	esp_bt_uuid_t service_uuid, char_uuid;
	if (!decode_uuid(args[1], &service_uuid) || !decode_uuid(args[2], &char_uuid)) {
		lbm_set_error_reason(error_invalid_uuid);
		return ENC_SYM_EERROR;
	}

	uint8_t buf[BLE_CLIENT_MAX_VAL_LEN];
	uint16_t len = sizeof(buf);

	ble_client_result_t result = ble_client_read(
		lbm_dec_as_i32(args[0]), service_uuid, char_uuid, buf, &len
	);
	if (result != BLE_CLIENT_OK) {
		return ble_result_to_lbm(result, false);
	}

	uint8_t *result_data = lbm_malloc_reserve(len);
	if (result_data == NULL) {
		return ENC_SYM_MERROR;
	}
	memcpy(result_data, buf, len);

	lbm_value ret;
	if (!lbm_lift_array(&ret, (char *)result_data, len)) {
		return ENC_SYM_MERROR;
	}

	return ret;
}

/**
 * signature: (ble-client-subscribe conn-handle:number service-uuid char-uuid) -> char-handle | nil
 *
 * Enables notifications/indications for the given characteristic and
 * returns its ATT handle (a number) on success, or nil/eval-error on
 * failure. Incoming values are subsequently reported as
 * (event-ble-client-data conn-handle char-handle data) events, tagged
 * with that same handle - save the return value and compare against it,
 * rather than the UUID, when matching notifications (a UUID is not part
 * of the event payload, since resolving it back from the handle would
 * require extra bookkeeping that's not needed for the common case of
 * subscribing to a small, known set of characteristics per connection).
 */
static lbm_value ext_ble_client_subscribe(lbm_value *args, lbm_uint argn) {
	if (argn != 3 || !lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}

	esp_bt_uuid_t service_uuid, char_uuid;
	if (!decode_uuid(args[1], &service_uuid) || !decode_uuid(args[2], &char_uuid)) {
		lbm_set_error_reason(error_invalid_uuid);
		return ENC_SYM_EERROR;
	}

	uint16_t handle = 0;
	ble_client_result_t result = ble_client_subscribe(
		lbm_dec_as_i32(args[0]), service_uuid, char_uuid, &handle
	);
	if (result != BLE_CLIENT_OK) {
		return ble_result_to_lbm(result, false);
	}

	return lbm_enc_u(handle);
}

// Async event plumbing. These run on the Bluedroid callback task, so only
// cheap, allocation-free-on-failure work should happen here.

static void scan_result_cb(const ble_client_scan_result_t *result) {
	if (!event_ble_scan_en) {
		return;
	}

	lbm_flat_value_t v;
	if (!lbm_start_flatten(&v, 60)) {
		return;
	}

	f_cons(&v);
	f_sym(&v, sym_event_ble_scan);

	f_cons(&v);
	f_lbm_array(&v, 6, (uint8_t *)result->addr);

	f_cons(&v);
	f_i(&v, result->rssi);

	f_cons(&v);
	if (result->has_name) {
		f_lbm_array(&v, strlen(result->name), (uint8_t *)result->name);
	} else {
		f_sym(&v, SYM_NIL);
	}

	f_sym(&v, SYM_NIL);

	lbm_finish_flatten(&v);

	if (!lbm_event(&v)) {
		lbm_free(v.buf);
	}
}

static void connect_cb(int conn_handle, bool connected) {
	if (!event_ble_client_connect_en) {
		return;
	}

	lbm_flat_value_t v;
	if (!lbm_start_flatten(&v, 30)) {
		return;
	}

	f_cons(&v);
	f_sym(&v, sym_event_ble_client_connect);

	f_cons(&v);
	f_i(&v, conn_handle);

	f_cons(&v);
	f_sym(&v, connected ? SYM_TRUE : SYM_NIL);

	f_sym(&v, SYM_NIL);

	lbm_finish_flatten(&v);

	if (!lbm_event(&v)) {
		lbm_free(v.buf);
	}
}

static void notify_cb(
	int conn_handle, uint16_t char_handle, uint16_t len, const uint8_t value[len]
) {
	if (!event_ble_client_data_en) {
		return;
	}

	lbm_flat_value_t v;
	if (!lbm_start_flatten(&v, 50 + len)) {
		return;
	}

	f_cons(&v);
	f_sym(&v, sym_event_ble_client_data);

	f_cons(&v);
	f_i(&v, conn_handle);

	f_cons(&v);
	f_u(&v, char_handle);

	f_cons(&v);
	f_lbm_array(&v, len, (uint8_t *)value);

	f_sym(&v, SYM_NIL);

	lbm_finish_flatten(&v);

	if (!lbm_event(&v)) {
		lbm_free(v.buf);
	}
}

void lispif_load_ble_client_extensions(void) {
	ble_client_set_scan_result_cb(scan_result_cb);
	ble_client_set_connect_cb(connect_cb);
	ble_client_set_notify_cb(notify_cb);

	lbm_add_extension("ble-client-scan-start", ext_ble_client_scan_start);
	lbm_add_extension("ble-client-scan-stop", ext_ble_client_scan_stop);
	lbm_add_extension("ble-client-scanning", ext_ble_client_scanning);
	lbm_add_extension("ble-client-connect", ext_ble_client_connect);
	lbm_add_extension("ble-client-disconnect", ext_ble_client_disconnect);
	lbm_add_extension("ble-client-connected", ext_ble_client_connected);
	lbm_add_extension("ble-client-write", ext_ble_client_write);
	lbm_add_extension("ble-client-read", ext_ble_client_read);
	lbm_add_extension("ble-client-subscribe", ext_ble_client_subscribe);
}
