/*
	Copyright 2020 - 2023 Benjamin Vedder	benjamin@vedder.se

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

#ifndef BMS_H_
#define BMS_H_

#include "datatypes.h"

// Functions
void bms_init(void);
bool bms_process_can_frame(uint32_t can_id, uint8_t *data8, int len, bool is_ext);
void bms_process_cmd(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len));
volatile bms_values *bms_get_values(void);
void bms_send_status_can(void);
void bms_register_cmd_handler(void (*handler)(COMM_PACKET_ID cmd, int param1, int param2));

#endif /* BMS_H_ */
