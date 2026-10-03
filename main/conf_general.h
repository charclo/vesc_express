/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_CONF_GENERAL_H_
#define MAIN_CONF_GENERAL_H_

#include "datatypes.h"

// Firmware version
#define FW_VERSION_MAJOR			7
#define FW_VERSION_MINOR			00
// Set to 0 for building a release and iterate during beta test builds
#define FW_TEST_VERSION_NUMBER		0

#if !defined(HW_SOURCE) && !defined(HW_SOURCE_ALT)
#error "No hardware source file set"
#endif

#ifndef HW_HEADER
#error "No hardware header file set"
#endif

#include "main.h"
#include "hw.h"

// Automatic deep sleep. Go to deep sleep when there has been no CAN-traffic,
// received command or open BLE/TCP connection for AUTO_SLEEP_CAN_TIMEOUT
// seconds. Set to 0 to disable. Can be overridden in the hardware header.
#ifndef AUTO_SLEEP_CAN_TIMEOUT
#define AUTO_SLEEP_CAN_TIMEOUT		0
#endif

// Wake up from automatic deep sleep every this many seconds so that BLE can
// advertise. Set to 0 to only wake up on CAN-activity.
#ifndef AUTO_SLEEP_WAKE_INTERVAL
#define AUTO_SLEEP_WAKE_INTERVAL	60
#endif

// Seconds to stay awake after a timer wakeup when nothing happens
#ifndef AUTO_SLEEP_AWAKE_TIME
#define AUTO_SLEEP_AWAKE_TIME		10
#endif

#endif /* MAIN_CONF_GENERAL_H_ */
