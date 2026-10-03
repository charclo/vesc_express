/*
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

#include "auto_sleep.h"
#include "conf_general.h"
#include "comm_ble.h"
#include "comm_wifi.h"
#include "comm_can.h"

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

#if CONFIG_ESP_WIFI_ENABLED || CONFIG_ESP_WIFI_REMOTE_ENABLED
#include "esp_wifi.h"
#endif

/*
 * Automatic deep sleep when the CAN-bus has been quiet for a while.
 *
 * The device wakes up again on CAN-activity (when the CAN RX pin can be used
 * as wakeup source) and every AUTO_SLEEP_WAKE_INTERVAL seconds. After a timer
 * wakeup it stays awake for AUTO_SLEEP_AWAKE_TIME seconds so that BLE can
 * advertise and a phone can connect. Any CAN-traffic, received command or
 * open BLE/TCP connection keeps the device awake.
 */

#if AUTO_SLEEP_CAN_TIMEOUT > 0 && defined(CAN_RX_GPIO_NUM)

#define AUTO_SLEEP_MAGIC			0x5A17E3B1

static const char *TAG = "auto_sleep";

// Set right before an automatic deep sleep, so that a timer wakeup from it can
// be told apart from the deep sleep that COMM_REBOOT uses to restart.
static RTC_NOINIT_ATTR uint32_t auto_sleep_magic;

static volatile TickType_t last_activity = 0;
static volatile bool activity_seen = false;

static bool is_busy(void) {
	if (comm_ble_is_connected()) {
		return true;
	}

#if CONFIG_ESP_WIFI_ENABLED || CONFIG_ESP_WIFI_REMOTE_ENABLED
	if (comm_wifi_is_client_connected() || comm_wifi_is_connected_hub()) {
		return true;
	}
#endif

	return false;
}

static bool can_wakeup_possible(void) {
	int pin = CAN_RX_GPIO_NUM;

	if (!esp_sleep_is_valid_wakeup_gpio(pin)) {
		return false;
	}

	// The recessive bus level is high. If the pin is low now (no transceiver
	// power or a stuck bus) we would wake up immediately, so skip it.
	return gpio_get_level(pin) == 1;
}

static void enable_can_wakeup(void) {
	int pin = CAN_RX_GPIO_NUM;

	gpio_set_direction(pin, GPIO_MODE_INPUT);

#if CONFIG_IDF_TARGET_ESP32S3
	esp_sleep_enable_ext0_wakeup(pin, 0);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32P4
	esp_deep_sleep_enable_gpio_wakeup(1ULL << pin, ESP_GPIO_WAKEUP_GPIO_LOW);
#else
	#error "Unsupported target"
#endif
}

static void go_to_sleep(void) {
	bool can_wakeup = can_wakeup_possible();
	bool timer_wakeup = AUTO_SLEEP_WAKE_INTERVAL > 0;

	if (!can_wakeup && !timer_wakeup) {
		// Without a wakeup source only a power cycle would wake us up again.
		ESP_LOGW(TAG, "No wakeup source available, not sleeping");
		auto_sleep_feed();
		return;
	}

#if CONFIG_ESP_WIFI_ENABLED || CONFIG_ESP_WIFI_REMOTE_ENABLED
	comm_wifi_disconnect();
	vTaskDelay(50 / portTICK_PERIOD_MS);
	esp_wifi_stop();
#endif

	comm_can_stop();

	if (can_wakeup) {
		enable_can_wakeup();
	}

	if (timer_wakeup) {
		esp_sleep_enable_timer_wakeup((uint64_t)AUTO_SLEEP_WAKE_INTERVAL * 1000000ULL);
	}

#ifdef HW_AUTO_SLEEP_HOOK
	HW_AUTO_SLEEP_HOOK();
#endif

	auto_sleep_magic = AUTO_SLEEP_MAGIC;
	esp_deep_sleep_start();
}

static void sleep_task(void *arg) {
	(void)arg;

	bool timer_wakeup = auto_sleep_magic == AUTO_SLEEP_MAGIC &&
			esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
	auto_sleep_magic = 0;

	for (;;) {
		vTaskDelay(100 / portTICK_PERIOD_MS);

		if (is_busy()) {
			auto_sleep_feed();
		}

		// After a timer wakeup only stay awake for a short window, unless
		// something happens during it.
		uint32_t timeout_s = AUTO_SLEEP_CAN_TIMEOUT;
		if (timer_wakeup && !activity_seen) {
			timeout_s = AUTO_SLEEP_AWAKE_TIME;
		}

		if ((TickType_t)(xTaskGetTickCount() - last_activity) > pdMS_TO_TICKS(timeout_s * 1000)) {
			go_to_sleep();
		}
	}
}

void auto_sleep_init(void) {
	last_activity = xTaskGetTickCount();
	xTaskCreatePinnedToCore(sleep_task, "auto_sleep", 3072, NULL, 3, NULL, tskNO_AFFINITY);
}

void auto_sleep_feed(void) {
	last_activity = xTaskGetTickCount();
	activity_seen = true;
}

#else

void auto_sleep_init(void) {
}

void auto_sleep_feed(void) {
}

#endif
