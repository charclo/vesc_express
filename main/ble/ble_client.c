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

// Minimal BLE central (GATT client) role, meant to be driven from LispBM
// scripts (see lispif_ble_client_extensions.c) so that things like heart
// rate straps or third party BLE BMS units can be talked to without a
// firmware rebuild for every device that's supported.
//
// This intentionally does not try to be a general purpose BLE stack: a
// single shared "pending operation" is used to turn the async ESP GATTC
// callbacks into blocking calls, the same way the rest of custom_ble.c
// already does for the GATT server role. This is safe because the LispBM
// evaluator only ever runs one extension call at a time - see the callers
// in lispif_ble_client_extensions.c.

#include "ble_client.h"

#if CONFIG_BT_BLUEDROID_ENABLED

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"

#include "commands.h"

// custom_ble.c registers the GATT server under app id 0.
#define BLE_CLIENT_APP_ID 1

#define BLE_CLIENT_CONNECT_TIMEOUT_MS 10000
#define BLE_CLIENT_OP_TIMEOUT_MS      3000
#define BLE_CLIENT_POLL_MS            10

typedef struct {
	bool in_use;
	bool connected;
	uint16_t conn_id;
	esp_bd_addr_t addr;
	esp_ble_addr_type_t addr_type;

	// Handle range of the most recently discovered service, cached so that
	// repeated access to the same service doesn't need a new discovery.
	bool have_service_range;
	esp_bt_uuid_t service_uuid;
	uint16_t service_start_handle;
	uint16_t service_end_handle;
} ble_client_conn_t;

typedef enum {
	OP_NONE = 0,
	OP_CONNECT,
	OP_SEARCH,
	OP_REG_NOTIFY,
	OP_WRITE_DESCR,
	OP_WRITE_CHAR,
	OP_READ_CHAR,
} pending_op_kind_t;

static bool m_started              = false;
static esp_gatt_if_t m_gattc_if    = 0;
static bool m_have_gattc_if        = false;
static bool m_scanning             = false;

static ble_client_conn_t m_conns[BLE_CLIENT_MAX_CONN];

static ble_client_scan_result_cb_t m_scan_cb = NULL;
static ble_client_connect_cb_t m_connect_cb  = NULL;
static ble_client_notify_cb_t m_notify_cb    = NULL;

// State for the single in-flight blocking operation. Only ever touched
// while a ble_client_* call from the (single-threaded) LispBM evaluator is
// blocked waiting for it, so no locking is needed beyond that.
static volatile pending_op_kind_t m_op = OP_NONE;
static volatile bool m_op_ready        = false;
static volatile uint16_t m_op_conn_id  = 0xFFFF;
static volatile esp_gatt_status_t m_op_status = ESP_GATT_OK;

static volatile bool m_search_found          = false;
static volatile uint16_t m_search_start      = 0;
static volatile uint16_t m_search_end        = 0;

static uint8_t m_read_buf[BLE_CLIENT_MAX_VAL_LEN];
static volatile uint16_t m_read_len = 0;

static bool uuid_equal(esp_bt_uuid_t a, esp_bt_uuid_t b) {
	if (a.len != b.len) {
		return false;
	}
	switch (a.len) {
		case ESP_UUID_LEN_16:
			return a.uuid.uuid16 == b.uuid.uuid16;
		case ESP_UUID_LEN_32:
			return a.uuid.uuid32 == b.uuid.uuid32;
		case ESP_UUID_LEN_128:
			return memcmp(a.uuid.uuid128, b.uuid.uuid128, 16) == 0;
		default:
			return false;
	}
}

static int find_conn_slot_by_conn_id(uint16_t conn_id) {
	for (int i = 0; i < BLE_CLIENT_MAX_CONN; i++) {
		if (m_conns[i].in_use && m_conns[i].conn_id == conn_id) {
			return i;
		}
	}
	return -1;
}

// Blocks the calling task until m_op_ready is set (by the GATTC/GAP event
// handler running on the Bluedroid callback task) or the timeout expires.
// Always clears m_op back to OP_NONE before returning.
static bool wait_for_op(uint32_t timeout_ms) {
	uint32_t waited = 0;
	while (!m_op_ready) {
		if (waited >= timeout_ms) {
			m_op = OP_NONE;
			return false;
		}
		vTaskDelay(BLE_CLIENT_POLL_MS / portTICK_PERIOD_MS);
		waited += BLE_CLIENT_POLL_MS;
	}
	m_op = OP_NONE;
	return true;
}

static void gattc_event_handler(
	esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
	esp_ble_gattc_cb_param_t *param
) {
	switch (event) {
		case ESP_GATTC_REG_EVT: {
			if (param->reg.status == ESP_GATT_OK) {
				m_gattc_if     = gattc_if;
				m_have_gattc_if = true;
			}
			break;
		}

		case ESP_GATTC_OPEN_EVT: {
			int slot = find_conn_slot_by_conn_id(param->open.conn_id);
			if (slot < 0) {
				// Might not have been recorded yet if OPEN races CONNECT;
				// try to match by address instead.
				for (int i = 0; i < BLE_CLIENT_MAX_CONN; i++) {
					if (m_conns[i].in_use && !m_conns[i].connected
						&& memcmp(
							   m_conns[i].addr, param->open.remote_bda,
							   sizeof(esp_bd_addr_t)
						   ) == 0) {
						slot = i;
						break;
					}
				}
			}

			if (slot >= 0) {
				m_conns[slot].conn_id = param->open.conn_id;
				if (param->open.status == ESP_GATT_OK) {
					m_conns[slot].connected = true;
				} else {
					m_conns[slot].in_use = false;
				}
			}

			if (m_op == OP_CONNECT
				&& (slot < 0
					|| param->open.conn_id == m_op_conn_id
					|| m_op_conn_id == 0xFFFF)) {
				m_op_status = param->open.status;
				m_op_ready  = true;
			}

			break;
		}

		case ESP_GATTC_CONNECT_EVT: {
			// Just make sure the pending connect's conn_id is filled in as
			// soon as it's known - the actual success/failure is reported
			// through ESP_GATTC_OPEN_EVT above.
			for (int i = 0; i < BLE_CLIENT_MAX_CONN; i++) {
				if (m_conns[i].in_use && !m_conns[i].connected
					&& memcmp(
						   m_conns[i].addr, param->connect.remote_bda,
						   sizeof(esp_bd_addr_t)
					   ) == 0) {
					m_conns[i].conn_id = param->connect.conn_id;
					if (m_op == OP_CONNECT && m_op_conn_id == 0xFFFF) {
						m_op_conn_id = param->connect.conn_id;
					}
					break;
				}
			}
			break;
		}

		case ESP_GATTC_DISCONNECT_EVT: {
			int slot = find_conn_slot_by_conn_id(param->disconnect.conn_id);
			if (slot >= 0) {
				m_conns[slot].in_use   = false;
				m_conns[slot].connected = false;
				if (m_connect_cb != NULL) {
					m_connect_cb(slot, false);
				}
			}
			break;
		}

		case ESP_GATTC_SEARCH_RES_EVT: {
			if (m_op == OP_SEARCH && param->search_res.conn_id == m_op_conn_id
				&& !m_search_found) {
				m_search_found = true;
				m_search_start = param->search_res.start_handle;
				m_search_end   = param->search_res.end_handle;
			}
			break;
		}

		case ESP_GATTC_SEARCH_CMPL_EVT: {
			if (m_op == OP_SEARCH && param->search_cmpl.conn_id == m_op_conn_id) {
				m_op_status = param->search_cmpl.status;
				m_op_ready  = true;
			}
			break;
		}

		case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
			if (m_op == OP_REG_NOTIFY) {
				m_op_status = param->reg_for_notify.status;
				m_op_ready  = true;
			}
			break;
		}

		case ESP_GATTC_WRITE_DESCR_EVT: {
			if (m_op == OP_WRITE_DESCR && param->write.conn_id == m_op_conn_id) {
				m_op_status = param->write.status;
				m_op_ready  = true;
			}
			break;
		}

		case ESP_GATTC_WRITE_CHAR_EVT: {
			if (m_op == OP_WRITE_CHAR && param->write.conn_id == m_op_conn_id) {
				m_op_status = param->write.status;
				m_op_ready  = true;
			}
			break;
		}

		case ESP_GATTC_READ_CHAR_EVT: {
			if (m_op == OP_READ_CHAR && param->read.conn_id == m_op_conn_id) {
				m_op_status = param->read.status;
				if (param->read.status == ESP_GATT_OK) {
					uint16_t len = param->read.value_len;
					if (len > BLE_CLIENT_MAX_VAL_LEN) {
						len = BLE_CLIENT_MAX_VAL_LEN;
					}
					memcpy(m_read_buf, param->read.value, len);
					m_read_len = len;
				} else {
					m_read_len = 0;
				}
				m_op_ready = true;
			}
			break;
		}

		case ESP_GATTC_NOTIFY_EVT: {
			int slot = find_conn_slot_by_conn_id(param->notify.conn_id);
			if (slot >= 0 && m_notify_cb != NULL) {
				m_notify_cb(
					slot, param->notify.handle, param->notify.value_len,
					param->notify.value
				);
			}
			break;
		}

		default:
			break;
	}
}

// AD structures are [len][type][data...], len includes the type byte.
static bool parse_adv_name(
	const uint8_t *adv, uint8_t adv_len, char *out, size_t out_cap
) {
	size_t pos = 0;
	while (pos + 1 < adv_len) {
		uint8_t field_len = adv[pos];
		if (field_len == 0) {
			break;
		}
		if (pos + 1 + field_len > adv_len) {
			break;
		}
		uint8_t type = adv[pos + 1];
		if (type == ESP_BLE_AD_TYPE_NAME_CMPL
			|| type == ESP_BLE_AD_TYPE_NAME_SHORT) {
			size_t name_len = field_len - 1;
			if (name_len >= out_cap) {
				name_len = out_cap - 1;
			}
			memcpy(out, &adv[pos + 2], name_len);
			out[name_len] = '\0';
			return true;
		}
		pos += 1 + field_len;
	}
	return false;
}

void ble_client_handle_gap_event(
	esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param
) {
	switch (event) {
		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
			// Kicked off by ble_client_scan_start once params are applied.
			break;
		}

		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
			m_scanning = param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS;
			break;
		}

		case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
			m_scanning = false;
			break;
		}

		case ESP_GAP_BLE_SCAN_RESULT_EVT: {
			struct ble_scan_result_evt_param *scan_rst = &param->scan_rst;
			if (scan_rst->search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
				if (m_scan_cb != NULL) {
					ble_client_scan_result_t result = {0};
					memcpy(result.addr, scan_rst->bda, sizeof(esp_bd_addr_t));
					result.addr_type = scan_rst->ble_addr_type;
					result.rssi      = scan_rst->rssi;
					result.has_name  = parse_adv_name(
						 scan_rst->ble_adv, scan_rst->adv_data_len + scan_rst->scan_rsp_len,
						 result.name, sizeof(result.name)
					 );
					m_scan_cb(&result);
				}
			} else if (scan_rst->search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
				m_scanning = false;
			}
			break;
		}

		default:
			break;
	}
}

ble_client_result_t ble_client_start(void) {
	if (m_started) {
		return BLE_CLIENT_OK;
	}

	memset(m_conns, 0, sizeof(m_conns));

	esp_err_t res = esp_ble_gattc_register_callback(gattc_event_handler);
	if (res != ESP_OK) {
		return BLE_CLIENT_ESP_ERROR;
	}

	res = esp_ble_gattc_app_register(BLE_CLIENT_APP_ID);
	if (res != ESP_OK) {
		return BLE_CLIENT_ESP_ERROR;
	}

	m_started = true;

	return BLE_CLIENT_OK;
}

bool ble_client_started(void) {
	return m_started;
}

void ble_client_set_scan_result_cb(ble_client_scan_result_cb_t cb) {
	m_scan_cb = cb;
}

void ble_client_set_connect_cb(ble_client_connect_cb_t cb) {
	m_connect_cb = cb;
}

void ble_client_set_notify_cb(ble_client_notify_cb_t cb) {
	m_notify_cb = cb;
}

ble_client_result_t ble_client_scan_start(uint32_t duration_s) {
	if (!m_started) {
		return BLE_CLIENT_NOT_STARTED;
	}

	static esp_ble_scan_params_t scan_params = {
		.scan_type          = BLE_SCAN_TYPE_ACTIVE,
		.own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
		.scan_interval      = 0x50,
		.scan_window        = 0x30,
		.scan_duplicate     = BLE_SCAN_DUPLICATE_ENABLE,
	};

	esp_err_t res = esp_ble_gap_set_scan_params(&scan_params);
	if (res != ESP_OK) {
		return BLE_CLIENT_ESP_ERROR;
	}

	// esp_ble_gap_start_scanning can safely be called right away; the
	// controller queues it until the parameter update has completed.
	res = esp_ble_gap_start_scanning(duration_s);
	if (res != ESP_OK) {
		return BLE_CLIENT_ESP_ERROR;
	}

	m_scanning = true;

	return BLE_CLIENT_OK;
}

ble_client_result_t ble_client_scan_stop(void) {
	if (!m_started) {
		return BLE_CLIENT_NOT_STARTED;
	}

	esp_err_t res = esp_ble_gap_stop_scanning();
	if (res != ESP_OK) {
		return BLE_CLIENT_ESP_ERROR;
	}

	m_scanning = false;

	return BLE_CLIENT_OK;
}

bool ble_client_scanning(void) {
	return m_scanning;
}

int ble_client_connect(const esp_bd_addr_t addr, esp_ble_addr_type_t addr_type) {
	if (!m_started || !m_have_gattc_if) {
		return -BLE_CLIENT_NOT_STARTED;
	}

	int slot = -1;
	for (int i = 0; i < BLE_CLIENT_MAX_CONN; i++) {
		if (!m_conns[i].in_use) {
			slot = i;
			break;
		}
	}
	if (slot < 0) {
		return -BLE_CLIENT_TOO_MANY_CONN;
	}

	memset(&m_conns[slot], 0, sizeof(m_conns[slot]));
	m_conns[slot].in_use = true;
	memcpy(m_conns[slot].addr, addr, sizeof(esp_bd_addr_t));
	m_conns[slot].addr_type = addr_type;

	m_op         = OP_CONNECT;
	m_op_conn_id = 0xFFFF;
	m_op_ready   = false;
	m_op_status  = ESP_GATT_ERROR;

	esp_err_t res = esp_ble_gattc_open(m_gattc_if, (uint8_t *)addr, addr_type, true);
	if (res != ESP_OK) {
		m_conns[slot].in_use = false;
		m_op                 = OP_NONE;
		return -BLE_CLIENT_ESP_ERROR;
	}

	if (!wait_for_op(BLE_CLIENT_CONNECT_TIMEOUT_MS)) {
		m_conns[slot].in_use = false;
		return -BLE_CLIENT_TIMEOUT;
	}

	if (m_op_status != ESP_GATT_OK || !m_conns[slot].connected) {
		m_conns[slot].in_use = false;
		return -BLE_CLIENT_GATT_ERROR;
	}

	if (m_connect_cb != NULL) {
		m_connect_cb(slot, true);
	}

	return slot;
}

ble_client_result_t ble_client_disconnect(int conn_handle) {
	if (conn_handle < 0 || conn_handle >= BLE_CLIENT_MAX_CONN
		|| !m_conns[conn_handle].in_use) {
		return BLE_CLIENT_NOT_CONNECTED;
	}

	esp_err_t res = esp_ble_gattc_close(m_gattc_if, m_conns[conn_handle].conn_id);

	// The connection slot itself is freed from ESP_GATTC_DISCONNECT_EVT, so
	// that unsolicited disconnects are handled the same way.
	return res == ESP_OK ? BLE_CLIENT_OK : BLE_CLIENT_ESP_ERROR;
}

bool ble_client_is_connected(int conn_handle) {
	if (conn_handle < 0 || conn_handle >= BLE_CLIENT_MAX_CONN) {
		return false;
	}
	return m_conns[conn_handle].in_use && m_conns[conn_handle].connected;
}

// Resolves char_uuid (within service_uuid) to an attribute handle, running
// (and caching) service discovery first if needed. On success also reports
// the characteristic's properties, so callers can e.g. pick notify vs
// indicate.
static ble_client_result_t resolve_char(
	int conn_handle, esp_bt_uuid_t service_uuid, esp_bt_uuid_t char_uuid,
	uint16_t *out_handle, esp_gatt_char_prop_t *out_prop
) {
	if (conn_handle < 0 || conn_handle >= BLE_CLIENT_MAX_CONN
		|| !m_conns[conn_handle].in_use || !m_conns[conn_handle].connected) {
		return BLE_CLIENT_NOT_CONNECTED;
	}

	ble_client_conn_t *conn = &m_conns[conn_handle];

	if (!conn->have_service_range || !uuid_equal(conn->service_uuid, service_uuid)) {
		m_op         = OP_SEARCH;
		m_op_conn_id = conn->conn_id;
		m_op_ready   = false;
		m_op_status  = ESP_GATT_ERROR;
		m_search_found = false;

		esp_err_t res =
			esp_ble_gattc_search_service(m_gattc_if, conn->conn_id, &service_uuid);
		if (res != ESP_OK) {
			m_op = OP_NONE;
			return BLE_CLIENT_ESP_ERROR;
		}

		if (!wait_for_op(BLE_CLIENT_OP_TIMEOUT_MS)) {
			return BLE_CLIENT_TIMEOUT;
		}

		if (!m_search_found) {
			return BLE_CLIENT_NOT_FOUND;
		}

		conn->have_service_range = true;
		conn->service_uuid       = service_uuid;
		conn->service_start_handle = m_search_start;
		conn->service_end_handle   = m_search_end;
	}

	esp_gattc_char_elem_t char_elem;
	uint16_t count = 1;
	esp_gatt_status_t status = esp_ble_gattc_get_char_by_uuid(
		m_gattc_if, conn->conn_id, conn->service_start_handle,
		conn->service_end_handle, char_uuid, &char_elem, &count
	);
	if (status != ESP_GATT_OK || count == 0) {
		return BLE_CLIENT_NOT_FOUND;
	}

	*out_handle = char_elem.char_handle;
	if (out_prop != NULL) {
		*out_prop = char_elem.properties;
	}

	return BLE_CLIENT_OK;
}

ble_client_result_t ble_client_write(
	int conn_handle, esp_bt_uuid_t service_uuid, esp_bt_uuid_t char_uuid,
	uint16_t len, const uint8_t data[len], bool need_rsp
) {
	uint16_t handle;
	ble_client_result_t result =
		resolve_char(conn_handle, service_uuid, char_uuid, &handle, NULL);
	if (result != BLE_CLIENT_OK) {
		return result;
	}

	ble_client_conn_t *conn = &m_conns[conn_handle];

	if (need_rsp) {
		m_op         = OP_WRITE_CHAR;
		m_op_conn_id = conn->conn_id;
		m_op_ready   = false;
		m_op_status  = ESP_GATT_ERROR;
	}

	esp_err_t res = esp_ble_gattc_write_char(
		m_gattc_if, conn->conn_id, handle, len, (uint8_t *)data,
		need_rsp ? ESP_GATT_WRITE_TYPE_RSP : ESP_GATT_WRITE_TYPE_NO_RSP,
		ESP_GATT_AUTH_REQ_NONE
	);
	if (res != ESP_OK) {
		m_op = OP_NONE;
		return BLE_CLIENT_ESP_ERROR;
	}

	if (!need_rsp) {
		return BLE_CLIENT_OK;
	}

	if (!wait_for_op(BLE_CLIENT_OP_TIMEOUT_MS)) {
		return BLE_CLIENT_TIMEOUT;
	}

	return m_op_status == ESP_GATT_OK ? BLE_CLIENT_OK : BLE_CLIENT_GATT_ERROR;
}

ble_client_result_t ble_client_read(
	int conn_handle, esp_bt_uuid_t service_uuid, esp_bt_uuid_t char_uuid,
	uint8_t *out_data, uint16_t *inout_len
) {
	uint16_t handle;
	ble_client_result_t result =
		resolve_char(conn_handle, service_uuid, char_uuid, &handle, NULL);
	if (result != BLE_CLIENT_OK) {
		return result;
	}

	ble_client_conn_t *conn = &m_conns[conn_handle];

	m_op         = OP_READ_CHAR;
	m_op_conn_id = conn->conn_id;
	m_op_ready   = false;
	m_op_status  = ESP_GATT_ERROR;
	m_read_len   = 0;

	esp_err_t res = esp_ble_gattc_read_char(
		m_gattc_if, conn->conn_id, handle, ESP_GATT_AUTH_REQ_NONE
	);
	if (res != ESP_OK) {
		m_op = OP_NONE;
		return BLE_CLIENT_ESP_ERROR;
	}

	if (!wait_for_op(BLE_CLIENT_OP_TIMEOUT_MS)) {
		return BLE_CLIENT_TIMEOUT;
	}

	if (m_op_status != ESP_GATT_OK) {
		return BLE_CLIENT_GATT_ERROR;
	}

	uint16_t len = m_read_len;
	if (len > *inout_len) {
		len = *inout_len;
	}
	memcpy(out_data, m_read_buf, len);
	*inout_len = len;

	return BLE_CLIENT_OK;
}

ble_client_result_t ble_client_subscribe(
	int conn_handle, esp_bt_uuid_t service_uuid, esp_bt_uuid_t char_uuid,
	uint16_t *out_handle
) {
	uint16_t handle;
	esp_gatt_char_prop_t prop;
	ble_client_result_t result =
		resolve_char(conn_handle, service_uuid, char_uuid, &handle, &prop);
	if (result != BLE_CLIENT_OK) {
		return result;
	}

	if (out_handle != NULL) {
		*out_handle = handle;
	}

	ble_client_conn_t *conn = &m_conns[conn_handle];

	m_op         = OP_REG_NOTIFY;
	m_op_conn_id = conn->conn_id;
	m_op_ready   = false;
	m_op_status  = ESP_GATT_ERROR;

	esp_err_t res =
		esp_ble_gattc_register_for_notify(m_gattc_if, conn->addr, handle);
	if (res != ESP_OK) {
		m_op = OP_NONE;
		return BLE_CLIENT_ESP_ERROR;
	}

	if (!wait_for_op(BLE_CLIENT_OP_TIMEOUT_MS)) {
		return BLE_CLIENT_TIMEOUT;
	}
	if (m_op_status != ESP_GATT_OK) {
		return BLE_CLIENT_GATT_ERROR;
	}

	esp_bt_uuid_t cccd_uuid = {
		.len = ESP_UUID_LEN_16,
		.uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
	};
	esp_gattc_descr_elem_t descr_elem;
	uint16_t descr_count = 1;
	esp_gatt_status_t status = esp_ble_gattc_get_descr_by_char_handle(
		m_gattc_if, conn->conn_id, handle, cccd_uuid, &descr_elem, &descr_count
	);
	if (status != ESP_GATT_OK || descr_count == 0) {
		// The remote characteristic doesn't have a CCCD - notifications
		// were still enabled via register_for_notify above (some stacks
		// implement notify without a CCCD), so treat this as success.
		return BLE_CLIENT_OK;
	}

	uint8_t notify_en[2] = {0x01, 0x00};
	if (!(prop & ESP_GATT_CHAR_PROP_BIT_NOTIFY) && (prop & ESP_GATT_CHAR_PROP_BIT_INDICATE)) {
		notify_en[0] = 0x02;
	}

	m_op         = OP_WRITE_DESCR;
	m_op_conn_id = conn->conn_id;
	m_op_ready   = false;
	m_op_status  = ESP_GATT_ERROR;

	res = esp_ble_gattc_write_char_descr(
		m_gattc_if, conn->conn_id, descr_elem.handle, sizeof(notify_en),
		notify_en, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE
	);
	if (res != ESP_OK) {
		m_op = OP_NONE;
		return BLE_CLIENT_ESP_ERROR;
	}

	if (!wait_for_op(BLE_CLIENT_OP_TIMEOUT_MS)) {
		return BLE_CLIENT_TIMEOUT;
	}

	return m_op_status == ESP_GATT_OK ? BLE_CLIENT_OK : BLE_CLIENT_GATT_ERROR;
}

#endif /* CONFIG_BT_BLUEDROID_ENABLED */
