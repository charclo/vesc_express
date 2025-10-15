/*
	Copyright 2017 - 2022 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ublox.h"
#include "commands.h"
#include "terminal.h"
#include "nmea.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include <string.h>
#include <math.h>

// Settings
#define BAUDRATE_UBX_DEFAULT		9600
#define BAUDRATE_UBX_DEFAULT_NEW	38400
#define BAUDRATE					115200
#define LINE_BUFFER_SIZE			256
#define UBX_BUFFER_SIZE				2000
#define CFG_ACK_WAIT_MS				100

// For double precision literals
#define D(x) 						((double)x##L)
#define D_PI						D(3.14159265358979323846)

// Private types
typedef struct {
	uint8_t line[LINE_BUFFER_SIZE];
	uint8_t ubx[UBX_BUFFER_SIZE];
	int line_pos;
	int ubx_pos;
	uint8_t ubx_class;
	uint8_t ubx_id;
	uint8_t ubx_ck_a;
	uint8_t ubx_ck_b;
	int ubx_len;
} decoder_state;

// Private variables
static bool m_print_next_nav_sol = false;
static bool m_print_next_relposned = false;
static bool m_print_next_rawx = false;
static bool m_print_next_svin = false;
static bool m_print_next_nav_sat = false;
static bool m_print_next_mon_ver = false;
static bool m_print_next_cfg_gnss = false;
static decoder_state m_decoder_state;
static SemaphoreHandle_t wait_sem;
static volatile bool wait_was_ack;
static volatile bool m_init_ok = false;
static volatile bool sem_init_done = false;
static volatile bool should_terminate = true;
static volatile bool thd_rx_is_running = false;
static volatile int m_uart_num = 0;

// Private functions
static void proc_byte(uint8_t ch);
static void reset_decoder_state(void);
static void ubx_terminal_cmd_poll(int argc, const char **argv);
static void ubx_encode_send(uint8_t class, uint8_t id, uint8_t *msg, int len);
static int wait_ack_nak(int timeout_ms);

// Decode functions
static void ubx_decode(uint8_t class, uint8_t id, uint8_t *msg, int len);
static void ubx_decode_nav_sol(uint8_t *msg, int len);
static void ubx_decode_relposned(uint8_t *msg, int len);
static void ubx_decode_svin(uint8_t *msg, int len);
static void ubx_decode_ack(uint8_t *msg, int len);
static void ubx_decode_nak(uint8_t *msg, int len);
static void ubx_decode_rawx(uint8_t *msg, int len);
static void ubx_decode_nav_sat(uint8_t *msg, int len);
static void ubx_decode_cfg_gnss(uint8_t *msg, int len);
static void ubx_decode_mon_ver(uint8_t *msg, int len);

// Ublox type getters
static uint8_t ubx_get_U1(uint8_t *msg, int *ind);
static int8_t ubx_get_I1(uint8_t *msg, int *ind);
static uint8_t ubx_get_X1(uint8_t *msg, int *ind);
static uint16_t ubx_get_U2(uint8_t *msg, int *ind);
static int16_t ubx_get_I2(uint8_t *msg, int *ind);
static uint16_t ubx_get_X2(uint8_t *msg, int *ind);
static uint32_t ubx_get_U4(uint8_t *msg, int *ind);
static int32_t ubx_get_I4(uint8_t *msg, int *ind);
static uint32_t ubx_get_X4(uint8_t *msg, int *ind);
static float ubx_get_R4(uint8_t *msg, int *ind);
static double ubx_get_R8(uint8_t *msg, int *ind);

// Ublox type setters
static void ubx_put_U1(uint8_t *msg, int *ind, uint8_t data);
static void ubx_put_I1(uint8_t *msg, int *ind, int8_t data);
static void ubx_put_X1(uint8_t *msg, int *ind, uint8_t data);
static void ubx_put_U2(uint8_t *msg, int *ind, uint16_t data);
static void ubx_put_I2(uint8_t *msg, int *ind, int16_t data);
static void ubx_put_X2(uint8_t *msg, int *ind, uint16_t data);
static void ubx_put_U4(uint8_t *msg, int *ind, uint32_t data);
static void ubx_put_I4(uint8_t *msg, int *ind, int32_t data);
static void ubx_put_X4(uint8_t *msg, int *ind, uint32_t data);
static void ubx_put_R4(uint8_t *msg, int *ind, float data);
static void ubx_put_R8(uint8_t *msg, int *ind, double data);

// Callbacks
static void(*rx_nav_sol)(ubx_nav_sol *sol) = 0;
static void(*rx_relposned)(ubx_nav_relposned *pos) = 0;
static void(*rx_rawx)(ubx_rxm_rawx *pos) = 0;
static void(*rx_svin)(ubx_nav_svin *svin) = 0;
static void(*rx_nav_sat)(ubx_nav_sat *sat) = 0;
static void(*rx_gnss)(ubx_cfg_gnss *gnss) = 0;

static void rx_task(void *arg) {
	reset_decoder_state();

	while (!should_terminate) {
		if (!uart_is_driver_installed(m_uart_num)) {
			break;
		}

		uint8_t c;
		int res = uart_read_bytes(m_uart_num, &c, 1, 10);
		if (res == 1) {
			proc_byte(c);
		}
	}

	thd_rx_is_running = false;
	vTaskDelete(NULL);
}

bool ublox_init(bool print, uint16_t rate_ms, int num_uart, int pin_rx, int pin_tx) {
	m_init_ok = false;

	if (!sem_init_done) {
		wait_sem = xSemaphoreCreateBinary();
		sem_init_done = true;
	}

	uart_config_t uart_config = {
			.baud_rate = BAUDRATE,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_DEFAULT,
	};

	should_terminate = true;
	while (thd_rx_is_running) {
		vTaskDelay(1);
	}

	m_uart_num = num_uart;

	if (uart_is_driver_installed(m_uart_num)) {
		uart_driver_delete(m_uart_num);
	}

	uart_driver_install(m_uart_num, 512, 512, 0, 0, 0);
	uart_param_config(m_uart_num, &uart_config);
	uart_set_pin(m_uart_num, pin_tx, pin_rx, -1, -1);

	should_terminate = false;
	thd_rx_is_running = true;
	xTaskCreatePinnedToCore(rx_task, "ublox_rx", 3072, NULL, 8, NULL, tskNO_AFFINITY);

	// Prevent unused warnings
	(void)ubx_get_U1;
	(void)ubx_get_I1;
	(void)ubx_get_X1;
	(void)ubx_get_U2;
	(void)ubx_get_I2;
	(void)ubx_get_X2;
	(void)ubx_get_U4;
	(void)ubx_get_I4;
	(void)ubx_get_X4;
	(void)ubx_get_R4;
	(void)ubx_get_R8;

	(void)ubx_put_U1;
	(void)ubx_put_I1;
	(void)ubx_put_X1;
	(void)ubx_put_U2;
	(void)ubx_put_I2;
	(void)ubx_put_X2;
	(void)ubx_put_U4;
	(void)ubx_put_I4;
	(void)ubx_put_X4;
	(void)ubx_put_R4;
	(void)ubx_put_R8;

	ubx_cfg_prt_uart uart;
	uart.baudrate = BAUDRATE;
	uart.in_ubx = true;
	uart.in_nmea = true;
	uart.in_rtcm2 = false;
	uart.in_rtcm3 = false;
	uart.out_ubx = true;
	uart.out_nmea = true;
	uart.out_rtcm3 = false;

	bool baud_ok = false;
	bool is_m10 = false;

	// Up to 4 retries
	for (int i = 0;i < 4;i++) {
		if (!thd_rx_is_running) {
			return false;
		}

		// Make sure that the baudrate is correct on unconfigured UBX.
		if (ublox_cfg_rate(rate_ms, 1, 0) == -1) {
			// Set default baudrate
			uart_config.baud_rate = BAUDRATE_UBX_DEFAULT;

			uart_param_config(m_uart_num, &uart_config);

			vTaskDelay(100 / portTICK_PERIOD_MS);
			reset_decoder_state();
			ublox_cfg_prt_uart(&uart);

			vTaskDelay(100 / portTICK_PERIOD_MS);
			reset_decoder_state();
			ublox_cfg_prt_uart(&uart);

			// Set configured baudrate
			uart_config.baud_rate = BAUDRATE;
			uart_param_config(m_uart_num, &uart_config);

			vTaskDelay(100 / portTICK_PERIOD_MS);
			reset_decoder_state();

			if (ublox_cfg_rate(rate_ms, 1, 0) != -1) {
				baud_ok = true;
				break;
			}

			vTaskDelay(100 / portTICK_PERIOD_MS);
			reset_decoder_state();

			if (ublox_cfg_rate(rate_ms, 1, 0) != -1) {
				baud_ok = true;
				break;
			}
		} else {
			baud_ok = true;
			break;
		}

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}

	if (!baud_ok) {
		unsigned char buffer_baud[80];
		int ind = 0;
		ublox_cfg_append_uart1_baud(buffer_baud, &ind, BAUDRATE);
		ublox_cfg_append_rate(buffer_baud, &ind, rate_ms, 1);

		for (int i = 0;i < 4;i++) {
			if (!thd_rx_is_running) {
				return false;
			}

			// Make sure that the baudrate is correct on unconfigured UBX.
			if (ublox_cfg_valset(buffer_baud, ind, BAUDRATE, true, true) == -1) {
				// Set default baudrate
				uart_config.baud_rate = BAUDRATE_UBX_DEFAULT_NEW;
				uart_param_config(m_uart_num, &uart_config);

				vTaskDelay(100 / portTICK_PERIOD_MS);
				reset_decoder_state();
				ublox_cfg_valset(buffer_baud, ind, BAUDRATE, true, true);

				vTaskDelay(100 / portTICK_PERIOD_MS);
				reset_decoder_state();
				ublox_cfg_valset(buffer_baud, ind, BAUDRATE, true, true);

				// Set configured baudrate
				uart_config.baud_rate = BAUDRATE;
				uart_param_config(m_uart_num, &uart_config);

				vTaskDelay(100 / portTICK_PERIOD_MS);
				reset_decoder_state();

				if (ublox_cfg_valset(buffer_baud, ind, BAUDRATE, true, true) != -1) {
					baud_ok = true;
					break;
				}

				vTaskDelay(100 / portTICK_PERIOD_MS);
				reset_decoder_state();

				if (ublox_cfg_valset(buffer_baud, ind, BAUDRATE, true, true) != -1) {
					baud_ok = true;
					break;
				}
			} else {
				baud_ok = true;
				break;
			}

			vTaskDelay(500 / portTICK_PERIOD_MS);
		}

		if (baud_ok) {
			is_m10 = true;
		}
	}

	if (!baud_ok) {
		should_terminate = true;
		while (thd_rx_is_running) {
			vTaskDelay(1);
		}

		if (print) {
			commands_printf("Could not set baud rate");
		}
		return false;
	}

	if (!is_m10) {
		if (!thd_rx_is_running) {
			return false;
		}

		ublox_cfg_prt_uart(&uart);
		ublox_cfg_rate(rate_ms, 1, 0);

		// Dynamic model
		ubx_cfg_nav5 nav5;
		memset(&nav5, 0, sizeof(ubx_cfg_nav5));
		nav5.apply_dyn = true;
		nav5.dyn_model = 4;
		ublox_cfg_nav5(&nav5);

		ublox_cfg_msg(UBX_CLASS_NAV, UBX_NAV_SOL, 0);
		ublox_cfg_msg(UBX_CLASS_NAV, UBX_NAV_RELPOSNED, 0);
		ublox_cfg_msg(UBX_CLASS_NAV, UBX_NAV_SVIN, 0);
		ublox_cfg_msg(UBX_CLASS_NAV, UBX_NAV_SAT, 0);

		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_GGA, 1);
		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_GSV, 1);
		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_RMC, 1);

		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_GLL, 0);
		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_GSA, 0);
		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_VTG, 0);
		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_GRS, 0);
		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_GST, 0);
		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_ZDA, 0);
		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_GBS, 0);
		ublox_cfg_msg(UBX_CLASS_NMEA, UBX_NMEA_DTM, 0);

		ubx_cfg_nmea nmea;
		memset(&nmea, 0, sizeof(ubx_cfg_nmea));
		nmea.nmeaVersion = 0x41;
		nmea.numSv = 0;
		//	nmea.highPrec = true;

		ublox_cfg_nmea(&nmea);

		ubx_cfg_gnss gnss;
		memset(&gnss, 0, sizeof(ubx_cfg_gnss));
		gnss.num_ch_hw = 32;
		gnss.num_ch_use = 0xFF;
		gnss.num_blocks = 4;

		gnss.blocks[0].gnss_id = UBX_GNSS_ID_GPS;
		gnss.blocks[0].en = true;
		gnss.blocks[0].minTrkCh = 6;
		gnss.blocks[0].maxTrkCh = 16;
		gnss.blocks[0].flags = UBX_CFG_GNSS_GPS_L1C;

		gnss.blocks[1].gnss_id = UBX_GNSS_ID_SBAS;
		gnss.blocks[1].en = true;
		gnss.blocks[1].minTrkCh = 0;
		gnss.blocks[1].maxTrkCh = 3;
		gnss.blocks[1].flags = UBX_CFG_GNSS_SBAS_L1C;

		gnss.blocks[2].gnss_id = UBX_GNSS_ID_GLONASS;
		gnss.blocks[2].en = true;
		gnss.blocks[2].minTrkCh = 6;
		gnss.blocks[2].maxTrkCh = 16;
		gnss.blocks[2].flags = UBX_CFG_GNSS_GLO_L1;

		gnss.blocks[3].gnss_id = UBX_GNSS_ID_BEIDOU;
		gnss.blocks[3].en = false;
		gnss.blocks[3].minTrkCh = 6;
		gnss.blocks[3].maxTrkCh = 16;
		gnss.blocks[3].flags = UBX_CFG_GNSS_BDS_B1L;

		ublox_cfg_gnss(&gnss);
	} else {
		if (!thd_rx_is_running) {
			return false;
		}

		unsigned char buffer[90];
		int ind = 0;

		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_GGA_UART1, 1);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_GSV_UART1, 1);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_RMC_UART1, 1);

		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_UBX_NAV_SAT_UART1, 1);

		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_DTM_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_GBS_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_GLL_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_GNS_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_GRS_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_GSA_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_GST_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_RLM_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_VLW_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_VTG_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_NMEA_ID_ZDA_UART1, 0);
		ublox_cfg_append_u1(buffer, &ind, CFG_MSGOUT_PUBX_ID_POLYP_UART1, 0);

		ublox_cfg_valset(buffer, ind, BAUDRATE, true, true);
	}

	terminal_register_command_callback(
			"ubx_poll",
			"Poll one of the ubx protocol messages. Supported messages:\n"
			"  UBX_NAV_SOL - Position solution\n"
			"  UBX_NAV_RELPOSNED - Relative position to base in NED frame\n"
			"  UBX_NAV_SVIN - survey-in data\n"
			"  UBX_RXM_RAWX - raw data\n"
			"  UBX_NAV_SAT - satellite information\n"
			"  UBX_MON_VER - Ublox version information\n"
			"  UBX_CFG_GNSS - Print supported GNSS configuration",
			"[msg]",
			ubx_terminal_cmd_poll);

	m_init_ok = true;

	return true;
}

void ublox_stop(int num_uart) {
	if (m_uart_num != num_uart) {
		return;
	}

	should_terminate = true;
	while (thd_rx_is_running) {
		vTaskDelay(1);
	}
}

bool ublox_init_ok(void) {
	return m_init_ok;
}

void ublox_send(unsigned char *data, unsigned int len) {
	// Wait for the previous transmission to finish.
	uart_wait_tx_done(m_uart_num, 1000 / portTICK_PERIOD_MS);
	uart_write_bytes(m_uart_num, data, len);
}

void ublox_set_rx_callback_nav_sol(void(*func)(ubx_nav_sol *sol)) {
	rx_nav_sol = func;
}

void ublox_set_rx_callback_relposned(void(*func)(ubx_nav_relposned *pos)) {
	rx_relposned = func;
}

void ublox_set_rx_callback_rawx(void(*func)(ubx_rxm_rawx *rawx)) {
	rx_rawx = func;
}

void ublox_set_rx_callback_svin(void(*func)(ubx_nav_svin *svin)) {
	rx_svin = func;
}

void ublox_set_rx_callback_nav_sat(void(*func)(ubx_nav_sat *sat)) {
	rx_nav_sat = func;
}

void ublox_set_rx_callback_cfg_gnss(void(*func)(ubx_cfg_gnss *gnss)) {
	rx_gnss = func;
}

void ublox_poll(uint8_t msg_class, uint8_t id) {
	ubx_encode_send(msg_class, id, 0, 0);
}

/**
 * Set the uart1 port configuration.
 *
 * @param cfg
 * The configuration. Notice that always 8N1 configuration
 * and no tx ready function is used.
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_prt_uart(ubx_cfg_prt_uart *cfg) {
	uint8_t buffer[20];
	int ind = 0;

	ubx_put_U1(buffer, &ind, 1); // ID for UART1
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_X2(buffer, &ind, 0); // Always disable txready function

	uint32_t mode = 0;
	mode |= 3 << 6; // Always use 8 bits
	mode |= 4 << 9; // No parity
	mode |= 0 << 12; // 1 stop bit

	ubx_put_X4(buffer, &ind, mode);
	ubx_put_U4(buffer, &ind, cfg->baudrate);

	uint16_t in_proto = 0;
	in_proto |= (cfg->in_ubx ? 1 : 0) << 0;
	in_proto |= (cfg->in_nmea ? 1 : 0) << 1;
	in_proto |= (cfg->in_rtcm2 ? 1 : 0) << 2;
	in_proto |= (cfg->in_rtcm3 ? 1 : 0) << 5;

	ubx_put_X2(buffer, &ind, in_proto);

	uint16_t out_proto = 0;
	out_proto |= (cfg->out_ubx ? 1 : 0) << 0;
	out_proto |= (cfg->out_nmea ? 1 : 0) << 1;
	out_proto |= (cfg->out_rtcm3 ? 1 : 0) << 5;

	ubx_put_X2(buffer, &ind, out_proto);
	ubx_put_X2(buffer, &ind, 0); // No extended timeout
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_PRT, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

/**
 * Set the tmode3 configuration.
 *
 * @param cfg
 * The configuration.
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_tmode3(ubx_cfg_tmode3 *cfg) {
	uint8_t buffer[40];
	int ind = 0;

	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	uint16_t flags = ((cfg->lla ? 1 : 0) << 8) | cfg->mode;
	ubx_put_X2(buffer, &ind, flags);

	int32_t x_lat = 0;
	int32_t y_lon = 0;
	int32_t z_alt = 0;
	int8_t x_lat_hp = 0;
	int8_t y_lon_hp = 0;
	int8_t z_alt_hp = 0;
	if (cfg->lla) {
		x_lat = round(cfg->ecefx_lat * D(1e7));
		y_lon = round(cfg->ecefy_lon * D(1e7));
		z_alt = round(cfg->ecefz_alt * D(1e2));
		x_lat_hp = ((cfg->ecefx_lat - ((double)x_lat * D(1e-7))) * D(1e9));
		y_lon_hp = ((cfg->ecefy_lon - ((double)y_lon * D(1e-7))) * D(1e9));
		z_alt_hp = ((cfg->ecefz_alt - ((double)z_alt * D(1e-2))) * D(1e4));
	} else {
		x_lat = cfg->ecefx_lat * D(1e2);
		y_lon = cfg->ecefy_lon * D(1e2);
		z_alt = cfg->ecefz_alt * D(1e2);
		x_lat_hp = ((cfg->ecefx_lat - ((double)x_lat * D(1e-2))) * D(1e4));
		y_lon_hp = ((cfg->ecefy_lon - ((double)y_lon * D(1e-2))) * D(1e4));
		z_alt_hp = ((cfg->ecefz_alt - ((double)z_alt * D(1e-2))) * D(1e4));
	}

	ubx_put_I4(buffer, &ind, x_lat);
	ubx_put_I4(buffer, &ind, y_lon);
	ubx_put_I4(buffer, &ind, z_alt);
	ubx_put_I1(buffer, &ind, x_lat_hp);
	ubx_put_I1(buffer, &ind, y_lon_hp);
	ubx_put_I1(buffer, &ind, z_alt_hp);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U4(buffer, &ind, cfg->fixed_pos_acc * 1e4);
	ubx_put_U4(buffer, &ind, cfg->svin_min_dur);
	ubx_put_U4(buffer, &ind, cfg->svin_acc_limit * 1e4);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_TMODE3, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

/**
 * Set the msg output configuration.
 *
 * @param msg_class
 * The message class.
 *
 * @param id
 * The message id
 *
 * @param
 * The message rate. 0 = disbaled
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_msg(uint8_t msg_class, uint8_t id, uint8_t rate) {
	uint8_t buffer[8];
	int ind = 0;

	ubx_put_U1(buffer, &ind, msg_class);
	ubx_put_U1(buffer, &ind, id);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_MSG, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

/**
 * Set the measurement rate, navigation rate and time reference.
 *
 * @param meas_rate_ms
 * The elapsed time between GNSS measurements, which defines the rate,
 * e.g. 100ms => 10Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
 *
 * @param nav_rate_ms
 * The ratio between the number of measurements and the number of navigation
 * solutions, e.g. 5 means five measurements for every navigation solution.
 * Max. value is 127. (This parameter is ignored and the navRate is fixed to 1
 * in protocol versions less than 18)
 *
 * @param time_ref
 * The time system to which measurements are aligned:
 * 0: UTC time
 * 1: GPS time
 * 2: GLONASS time (not supported in protocol versions less than 18)
 * 3: BeiDou time (not supported in protocol versions less than 18)
 * 4: Galileo time (not supported in protocol versions less than 18)
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_rate(uint16_t meas_rate_ms, uint16_t nav_rate_ms, uint16_t time_ref) {
	uint8_t buffer[6];
	int ind = 0;

	ubx_put_U2(buffer, &ind, meas_rate_ms);
	ubx_put_U2(buffer, &ind, nav_rate_ms);
	ubx_put_U2(buffer, &ind, time_ref);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_RATE, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

/**
 * Save, load or clear configurations
 *
 * @param cfg
 * Which configurations to save/load/clear and which memories.
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_cfg(ubx_cfg_cfg *cfg) {
	uint8_t buffer[13];
	int ind = 0;

	uint32_t clear = 0;
	clear |= (cfg->clear_io_port ? 1 : 0) << 0;
	clear |= (cfg->clear_msg_conf ? 1 : 0) << 1;
	clear |= (cfg->clear_inf_msg ? 1 : 0) << 2;
	clear |= (cfg->clear_nav_conf ? 1 : 0) << 3;
	clear |= (cfg->clear_rxm_conf ? 1 : 0) << 4;
	clear |= (cfg->clear_sen_conf ? 1 : 0) << 8;
	clear |= (cfg->clear_rinv_conf ? 1 : 0) << 9;
	clear |= (cfg->clear_ant_conf ? 1 : 0) << 10;
	clear |= (cfg->clear_log_conf ? 1 : 0) << 11;
	clear |= (cfg->clear_fts_conf ? 1 : 0) << 12;

	uint32_t save = 0;
	save |= (cfg->save_io_port ? 1 : 0) << 0;
	save |= (cfg->save_msg_conf ? 1 : 0) << 1;
	save |= (cfg->save_inf_msg ? 1 : 0) << 2;
	save |= (cfg->save_nav_conf ? 1 : 0) << 3;
	save |= (cfg->save_rxm_conf ? 1 : 0) << 4;
	save |= (cfg->save_sen_conf ? 1 : 0) << 8;
	save |= (cfg->save_rinv_conf ? 1 : 0) << 9;
	save |= (cfg->save_ant_conf ? 1 : 0) << 10;
	save |= (cfg->save_log_conf ? 1 : 0) << 11;
	save |= (cfg->save_fts_conf ? 1 : 0) << 12;

	uint32_t load = 0;
	load |= (cfg->load_io_port ? 1 : 0) << 0;
	load |= (cfg->load_msg_conf ? 1 : 0) << 1;
	load |= (cfg->load_inf_msg ? 1 : 0) << 2;
	load |= (cfg->load_nav_conf ? 1 : 0) << 3;
	load |= (cfg->load_rxm_conf ? 1 : 0) << 4;
	load |= (cfg->load_sen_conf ? 1 : 0) << 8;
	load |= (cfg->load_rinv_conf ? 1 : 0) << 9;
	load |= (cfg->load_ant_conf ? 1 : 0) << 10;
	load |= (cfg->load_log_conf ? 1 : 0) << 11;
	load |= (cfg->load_fts_conf ? 1 : 0) << 12;

	uint8_t device = 0;
	device |= (cfg->dev_bbr ? 1 : 0) << 0;
	device |= (cfg->dev_flash ? 1 : 0) << 1;
	device |= (cfg->dev_eeprom ? 1 : 0) << 2;
	device |= (cfg->dev_spi_flash ? 1 : 0) << 4;

	ubx_put_X4(buffer, &ind, clear);
	ubx_put_X4(buffer, &ind, save);
	ubx_put_X4(buffer, &ind, load);
	ubx_put_X1(buffer, &ind, device);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_CFG, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

/**
 * Set the nav5 configuration.
 *
 * @param cfg
 * The configuration.
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_nav5(ubx_cfg_nav5 *cfg) {
	uint8_t buffer[36];
	int ind = 0;

	uint16_t mask = 0;
	mask |= (cfg->apply_dyn ? 1 : 0) << 0;
	mask |= (cfg->apply_min_el ? 1 : 0) << 1;
	mask |= (cfg->apply_pos_fix_mode ? 1 : 0) << 2;
	mask |= (cfg->apply_pos_mask ? 1 : 0) << 4;
	mask |= (cfg->apply_time_mask ? 1 : 0) << 5;
	mask |= (cfg->apply_static_hold_mask ? 1 : 0) << 6;
	mask |= (cfg->apply_dgps ? 1 : 0) << 7;
	mask |= (cfg->apply_cno ? 1 : 0) << 8;
	mask |= (cfg->apply_utc ? 1 : 0) << 10;

	ubx_put_X2(buffer, &ind, mask);
	ubx_put_U1(buffer, &ind, cfg->dyn_model);
	ubx_put_U1(buffer, &ind, cfg->fix_mode);
	ubx_put_I4(buffer, &ind, cfg->fixed_alt * D(100.0));
	ubx_put_U4(buffer, &ind, cfg->fixed_alt_var * D(10000.0));
	ubx_put_I1(buffer, &ind, cfg->min_elev);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U2(buffer, &ind, cfg->p_dop * 10.0);
	ubx_put_U2(buffer, &ind, cfg->t_dop * 10.0);
	ubx_put_U2(buffer, &ind, cfg->p_acc);
	ubx_put_U2(buffer, &ind, cfg->t_acc);
	ubx_put_U1(buffer, &ind, cfg->static_hold_thres);
	ubx_put_U1(buffer, &ind, cfg->dgnss_timeout);
	ubx_put_U1(buffer, &ind, cfg->cno_tres_num_sat);
	ubx_put_U1(buffer, &ind, cfg->cno_tres);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U2(buffer, &ind, cfg->static_hold_max_dist);
	ubx_put_U1(buffer, &ind, cfg->utc_standard);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_NAV5, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

int ublox_cfg_tp5(ubx_cfg_tp5 *cfg) {
	uint8_t buffer[32];
	int ind = 0;

	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 1);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_I2(buffer, &ind, cfg->ant_cable_delay);
	ubx_put_I2(buffer, &ind, cfg->rf_group_delay);
	ubx_put_U4(buffer, &ind, cfg->freq_period);
	ubx_put_U4(buffer, &ind, cfg->freq_period_lock);
	ubx_put_U4(buffer, &ind, cfg->pulse_len_ratio);
	ubx_put_U4(buffer, &ind, cfg->pulse_len_ratio_lock);
	ubx_put_I4(buffer, &ind, cfg->user_config_delay);

	uint32_t mask = 0;
	mask |= (cfg->active ? 1 : 0) << 0;
	mask |= (cfg->lockGnssFreq ? 1 : 0) << 1;
	mask |= (cfg->lockedOtherSet ? 1 : 0) << 2;
	mask |= (cfg->isFreq ? 1 : 0) << 3;
	mask |= (cfg->isLength ? 1 : 0) << 4;
	mask |= (cfg->alignToTow ? 1 : 0) << 5;
	mask |= (cfg->polarity ? 1 : 0) << 6;
	mask |= (cfg->gridUtcGnss & 0b1111) << 7;
	mask |= (cfg->syncMode & 0b111) << 8;

	ubx_put_X4(buffer, &ind, mask);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_TP5, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

int ublox_cfg_gnss(ubx_cfg_gnss *gnss) {
	if (gnss->num_blocks > 10) {
		return -2;
	}

	uint8_t buffer[4 + 8 * gnss->num_blocks];
	int ind = 0;

	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, gnss->num_ch_hw);
	ubx_put_U1(buffer, &ind, gnss->num_ch_use);
	ubx_put_U1(buffer, &ind, gnss->num_blocks);

	for (int i = 0;i < gnss->num_blocks;i++) {
		ubx_put_U1(buffer, &ind, gnss->blocks[i].gnss_id);
		ubx_put_U1(buffer, &ind, gnss->blocks[i].minTrkCh);
		ubx_put_U1(buffer, &ind, gnss->blocks[i].maxTrkCh);
		ubx_put_U1(buffer, &ind, 0);
		uint32_t flags = gnss->blocks[i].en ? 1 : 0;
		flags |= gnss->blocks[i].flags << 16;
		ubx_put_X4(buffer, &ind, flags);
	}

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_GNSS, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

int ublox_cfg_nmea(ubx_cfg_nmea *nmea) {
	uint8_t buffer[20];
	int ind = 0;

	uint8_t filter = 0;
	filter |= (nmea->posFilt ? 1 : 0) << 0;
	filter |= (nmea->mskPosFilt ? 1 : 0) << 1;
	filter |= (nmea->timeFilt ? 1 : 0) << 2;
	filter |= (nmea->dateFilt ? 1 : 0) << 3;
	filter |= (nmea->gpsOnlyFilt ? 1 : 0) << 4;
	filter |= (nmea->trackFilt ? 1 : 0) << 5;
	ubx_put_X1(buffer, &ind, filter);

	ubx_put_U1(buffer, &ind, nmea->nmeaVersion);
	ubx_put_U1(buffer, &ind, nmea->numSv);

	uint8_t flags = 0;
	flags |= (nmea->compat ? 1 : 0) << 0;
	flags |= (nmea->consider ? 1 : 0) << 1;
	flags |= (nmea->limit82 ? 1 : 0) << 2;
	flags |= (nmea->highPrec ? 1 : 0) << 3;
	ubx_put_X1(buffer, &ind, flags);

	uint32_t gnss_filter = 0;
	gnss_filter |= (nmea->disableGps ? 1 : 0) << 0;
	gnss_filter |= (nmea->disableSbas ? 1 : 0) << 1;
	gnss_filter |= (nmea->disableQzss ? 1 : 0) << 4;
	gnss_filter |= (nmea->disableGlonass ? 1 : 0) << 5;
	gnss_filter |= (nmea->disableBeidou ? 1 : 0) << 6;
	ubx_put_X4(buffer, &ind, gnss_filter);

	ubx_put_U1(buffer, &ind, nmea->svNumbering);
	ubx_put_U1(buffer, &ind, nmea->mainTalkerId);
	ubx_put_U1(buffer, &ind, nmea->gsvTalkerId);
	ubx_put_U1(buffer, &ind, 1);
	ubx_put_I1(buffer, &ind, nmea->bdsTalkerId[0]);
	ubx_put_I1(buffer, &ind, nmea->bdsTalkerId[1]);

	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_NMEA, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

int ublox_cfg_valset(unsigned char *values, int len,
		bool ram, bool bbr, bool flash) {
	uint8_t buffer[len + 4];
	int ind = 0;

	ubx_put_U1(buffer, &ind, 0);

	uint8_t mask = 0;
	mask |= (ram ? 1 : 0) << 0;
	mask |= (bbr ? 1 : 0) << 1;
	mask |= (flash ? 1 : 0) << 2;
	ubx_put_X1(buffer, &ind, mask);

	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);

	memcpy(buffer + ind, values, len);
	ind += len;

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_VALSET, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

void ublox_cfg_append_enable_gps(unsigned char *buffer, int *ind,
		bool en, bool en_l1c, bool en_l2c) {
	ubx_put_X4(buffer, ind, CFG_SIGNAL_GPS_ENA);
	ubx_put_U1(buffer, ind, en);
	ubx_put_X4(buffer, ind, CFG_SIGNAL_GPS_L1C_ENA);
	ubx_put_U1(buffer, ind, en_l1c);
	ubx_put_X4(buffer, ind, CFG_SIGNAL_GPS_L2C_ENA);
	ubx_put_U1(buffer, ind, en_l2c);
}

void ublox_cfg_append_enable_gal(unsigned char *buffer, int *ind,
		bool en, bool en_e1, bool en_e5b) {
	ubx_put_X4(buffer, ind, CFG_SIGNAL_GAL_ENA);
	ubx_put_U1(buffer, ind, en);
	ubx_put_X4(buffer, ind, CFG_SIGNAL_GAL_E1_ENA);
	ubx_put_U1(buffer, ind, en_e1);
	ubx_put_X4(buffer, ind, CFG_SIGNAL_GAL_E5B_ENA);
	ubx_put_U1(buffer, ind, en_e5b);
}

void ublox_cfg_append_enable_bds(unsigned char *buffer, int *ind,
		bool en, bool en_b1, bool en_b2) {
	ubx_put_X4(buffer, ind, CFG_SIGNAL_BDS_ENA);
	ubx_put_U1(buffer, ind, en);
	ubx_put_X4(buffer, ind, CFG_SIGNAL_BDS_B1_ENA);
	ubx_put_U1(buffer, ind, en_b1);
	ubx_put_X4(buffer, ind, CFG_SIGNAL_BDS_B2_ENA);
	ubx_put_U1(buffer, ind, en_b2);
}

void ublox_cfg_append_enable_glo(unsigned char *buffer, int *ind,
		bool en, bool en_l1, bool en_l2) {
	ubx_put_X4(buffer, ind, CFG_SIGNAL_GLO_ENA);
	ubx_put_U1(buffer, ind, en);
	ubx_put_X4(buffer, ind, CFG_SIGNAL_GLO_L1_ENA);
	ubx_put_U1(buffer, ind, en_l1);
	ubx_put_X4(buffer, ind, CFG_SIGNAL_GLO_L2_ENA);
	ubx_put_U1(buffer, ind, en_l2);
}

void ublox_cfg_append_uart1_baud(unsigned char *buffer, int *ind, uint32_t baud) {
	ubx_put_X4(buffer, ind, CFG_UART1_BAUDRATE);
	ubx_put_U4(buffer, ind, baud);
}

void ublox_cfg_append_rate(unsigned char *buffer, int *ind, uint16_t meas_ms, uint16_t nav) {
	ubx_put_X4(buffer, ind, CFG_RATE_MEAS);
	ubx_put_U2(buffer, ind, meas_ms);
	ubx_put_X4(buffer, ind, CFG_RATE_NAV);
	ubx_put_U2(buffer, ind, nav);
}

void ublox_cfg_append_u1(unsigned char *buffer, int *ind, uint32_t key, uint8_t val) {
	ubx_put_X4(buffer, ind, key);
	ubx_put_U1(buffer, ind, val);
}

static void proc_byte(uint8_t ch) {
	bool ch_used = false;

	// RTCM
//	if (!ch_used && m_decoder_state.line_pos == 0 && m_decoder_state.ubx_pos == 0) {
//		ch_used = rtcm3_input_data(ch, &m_rtcm_state) >= 0;
//	}

	// Ubx
	if (!ch_used && m_decoder_state.line_pos == 0) {
		int ubx_pos_last = m_decoder_state.ubx_pos;

		if (m_decoder_state.ubx_pos == 0) {
			if (ch == 0xB5) {
				m_decoder_state.ubx_pos++;
			}
		} else if (m_decoder_state.ubx_pos == 1) {
			if (ch == 0x62) {
				m_decoder_state.ubx_pos++;
				m_decoder_state.ubx_ck_a = 0;
				m_decoder_state.ubx_ck_b = 0;
			}
		} else if (m_decoder_state.ubx_pos == 2) {
			m_decoder_state.ubx_class = ch;
			m_decoder_state.ubx_ck_a += ch;
			m_decoder_state.ubx_ck_b += m_decoder_state.ubx_ck_a;
			m_decoder_state.ubx_pos++;
		} else if (m_decoder_state.ubx_pos == 3) {
			m_decoder_state.ubx_id = ch;
			m_decoder_state.ubx_ck_a += ch;
			m_decoder_state.ubx_ck_b += m_decoder_state.ubx_ck_a;
			m_decoder_state.ubx_pos++;
		} else if (m_decoder_state.ubx_pos == 4) {
			m_decoder_state.ubx_len = ch;
			m_decoder_state.ubx_ck_a += ch;
			m_decoder_state.ubx_ck_b += m_decoder_state.ubx_ck_a;
			m_decoder_state.ubx_pos++;
		} else if (m_decoder_state.ubx_pos == 5) {
			m_decoder_state.ubx_len |= ch << 8;
			m_decoder_state.ubx_ck_a += ch;
			m_decoder_state.ubx_ck_b += m_decoder_state.ubx_ck_a;
			m_decoder_state.ubx_pos++;
		} else if ((m_decoder_state.ubx_pos - 6) < m_decoder_state.ubx_len) {
			m_decoder_state.ubx[m_decoder_state.ubx_pos - 6] = ch;
			m_decoder_state.ubx_ck_a += ch;
			m_decoder_state.ubx_ck_b += m_decoder_state.ubx_ck_a;
			m_decoder_state.ubx_pos++;
		} else if ((m_decoder_state.ubx_pos - 6) == m_decoder_state.ubx_len) {
			if (ch == m_decoder_state.ubx_ck_a) {
				m_decoder_state.ubx_pos++;
			}
		} else if ((m_decoder_state.ubx_pos - 6) == (m_decoder_state.ubx_len + 1)) {
			if (ch == m_decoder_state.ubx_ck_b) {
				ubx_decode(m_decoder_state.ubx_class, m_decoder_state.ubx_id,
						m_decoder_state.ubx, m_decoder_state.ubx_len);
				m_decoder_state.ubx_pos = 0;
			}
		}

		if (ubx_pos_last != m_decoder_state.ubx_pos) {
			ch_used = true;
		} else {
			m_decoder_state.ubx_pos = 0;
		}
	}

	// NMEA
	if (!ch_used) {
		m_decoder_state.line[m_decoder_state.line_pos++] = ch;
		if (m_decoder_state.line_pos == LINE_BUFFER_SIZE) {
			m_decoder_state.line_pos = 0;
		}

		if (m_decoder_state.line_pos > 0 && m_decoder_state.line[m_decoder_state.line_pos - 1] == '\n') {
			m_decoder_state.line[m_decoder_state.line_pos] = '\0';
			m_decoder_state.line_pos = 0;
			nmea_decode_string((const char*)m_decoder_state.line);
		}
	}
}

static void reset_decoder_state(void) {
	memset(&m_decoder_state, 0, sizeof(decoder_state));
}

static void ubx_terminal_cmd_poll(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1], "UBX_NAV_SOL") == 0) {
			m_print_next_nav_sol = true;
			ublox_poll(UBX_CLASS_NAV, UBX_NAV_SOL);
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "UBX_NAV_RELPOSNED") == 0) {
			m_print_next_relposned = true;
			ublox_poll(UBX_CLASS_NAV, UBX_NAV_RELPOSNED);
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "UBX_NAV_SVIN") == 0) {
			m_print_next_svin = true;
			ublox_poll(UBX_CLASS_NAV, UBX_NAV_SVIN);
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "UBX_RXM_RAWX") == 0) {
			m_print_next_rawx = true;
			ublox_poll(UBX_CLASS_RXM, UBX_RXM_RAWX);
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "UBX_NAV_SAT") == 0) {
			m_print_next_nav_sat = true;
			ublox_poll(UBX_CLASS_NAV, UBX_NAV_SAT);
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "UBX_MON_VER") == 0) {
			m_print_next_mon_ver = true;
			ublox_poll(UBX_CLASS_MON, UBX_MON_VER);
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "UBX_CFG_GNSS") == 0) {
			m_print_next_cfg_gnss = true;
			ublox_poll(UBX_CLASS_CFG, UBX_CFG_GNSS);
			commands_printf("OK\n");
		} else {
			commands_printf("Wrong argument %s\n", argv[1]);
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}

static void ubx_encode_send(uint8_t class, uint8_t id, uint8_t *msg, int len) {
	static uint8_t ubx[UBX_BUFFER_SIZE];
	int ind = 0;
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;

	ubx[ind++] = 0xB5;
	ubx[ind++] = 0x62;

	ubx[ind] = class;
	ck_a += ubx[ind];
	ck_b += ck_a;
	ind++;

	ubx[ind] = id;
	ck_a += ubx[ind];
	ck_b += ck_a;
	ind++;

	ubx[ind] = len & 0xFF;
	ck_a += ubx[ind];
	ck_b += ck_a;
	ind++;

	ubx[ind] = (len >> 8) & 0xFF;
	ck_a += ubx[ind];
	ck_b += ck_a;
	ind++;

	for (int i = 0;i < len;i++) {
		ubx[ind] = msg[i];
		ck_a += ubx[ind];
		ck_b += ck_a;
		ind++;
	}

	ubx[ind++] = ck_a;
	ubx[ind++] = ck_b;

	ublox_send(ubx, ind);
}

/**
 * Wait for ack or nak.
 *
 * @param timeout_ms
 * The timeout for the wait, -1 = infinite.
 *
 * @return
 * 0: ack
 * 1: nak
 * -1: timeout
 *
 */
static int wait_ack_nak(int timeout_ms) {
	uint32_t to;

	if (timeout_ms >= 0) {
		to = timeout_ms;
	} else {
		to = portMAX_DELAY;
	}

	bool ret = xSemaphoreTake(wait_sem, to / portTICK_PERIOD_MS) == pdTRUE;

	if (ret && wait_was_ack) {
		return 0;
	} else if (ret && !wait_was_ack) {
		return 1;
	} else {
		return -1;
	}
}

static void ubx_decode(uint8_t class, uint8_t id, uint8_t *msg, int len) {
	switch (class) {
	case UBX_CLASS_NAV: {
		switch (id) {
		case UBX_NAV_SOL:
			ubx_decode_nav_sol(msg, len);
			break;
		case UBX_NAV_RELPOSNED:
			ubx_decode_relposned(msg, len);
			break;
		case UBX_NAV_SVIN:
			ubx_decode_svin(msg, len);
			break;
		case UBX_NAV_SAT:
			ubx_decode_nav_sat(msg, len);
			break;
		default:
			break;
		}
	} break;

	case UBX_CLASS_ACK: {
		switch (id) {
		case UBX_ACK_ACK:
			ubx_decode_ack(msg, len);
			break;

		case UBX_ACK_NAK:
			ubx_decode_nak(msg, len);
			break;

		default:
			break;
		}
	} break;

	case UBX_CLASS_RXM: {
		switch (id) {
		case UBX_RXM_RAWX:
			ubx_decode_rawx(msg, len);
			break;
		default:
			break;
		}
	} break;

	case UBX_CLASS_CFG: {
		switch (id) {
		case UBX_CFG_GNSS:
			ubx_decode_cfg_gnss(msg, len);
			break;
		default:
			break;
		}
	} break;

	case UBX_CLASS_MON: {
		switch (id) {
		case UBX_MON_VER:
			ubx_decode_mon_ver(msg, len);
			break;
		default:
			break;
		}
	} break;

	default:
		break;
	}
}

static void ubx_decode_nav_sol(uint8_t *msg, int len) {
	(void)len;

	static ubx_nav_sol sol;
	int ind = 0;
	uint8_t flags;

	sol.i_tow = ubx_get_U4(msg, &ind); // 0
	sol.f_tow = ubx_get_I4(msg, &ind); // 4
	sol.weel = ubx_get_I2(msg, &ind); // 8
	sol.gps_fix = ubx_get_U1(msg, &ind); // 10
	flags = ubx_get_X1(msg, &ind); // 11
	sol.gpsfixok = flags & 0x01;
	sol.diffsoln = flags & 0x02;
	sol.wknset = flags & 0x04;
	sol.towset = flags & 0x08;
	sol.ecef_x = (double)ubx_get_I4(msg, &ind) / D(100.0); // 12
	sol.ecef_y = (double)ubx_get_I4(msg, &ind) / D(100.0); // 16
	sol.ecef_z = (double)ubx_get_I4(msg, &ind) / D(100.0); // 20
	sol.p_acc = (float)ubx_get_U4(msg, &ind) / 100.0; // 24
	sol.ecef_vx = (float)ubx_get_I4(msg, &ind) / 100.0; // 28
	sol.ecef_vy = (float)ubx_get_I4(msg, &ind) / 100.0; // 32
	sol.ecef_vz = (float)ubx_get_I4(msg, &ind) / 100.0; // 36
	sol.s_acc = (float)ubx_get_U4(msg, &ind) / 100.0; // 40
	sol.p_dop = (float)ubx_get_U2(msg, &ind) * 0.01; // 44
	ind += 1; // 46
	sol.num_sv = ubx_get_U1(msg, &ind); // 47

	if (rx_nav_sol) {
		rx_nav_sol(&sol);
	}

	if (m_print_next_nav_sol) {
		m_print_next_nav_sol = false;
		commands_printf(
				"NAV_SOL RX\n"
				"num_sv: %d\n"
				"i_tow: %d ms\n"
				"week: %d\n"
				"fix: %d\n"
				"X: %.3f m\n"
				"Y: %.3f m\n"
				"Z: %.3f m\n"
				"p_acc: %.3f m\n"
				"VX: %.3f m/s\n"
				"VY: %.3f m/s\n"
				"VZ: %.3f m/s\n"
				"s_acc: %.3f m/s\n"
				"Fix OK: %d\n"
				"Diff Soln: %d\n"
				"Week valid: %d\n"
				"TOW valid: %d\n",
				sol.num_sv,
				sol.i_tow,
				sol.weel,
				sol.gps_fix,
				sol.ecef_x,
				sol.ecef_y,
				sol.ecef_z,
				(double)sol.p_acc,
				(double)sol.ecef_vx,
				(double)sol.ecef_vy,
				(double)sol.ecef_vz,
				(double)sol.s_acc,
				sol.gpsfixok,
				sol.diffsoln,
				sol.wknset,
				sol.towset);
	}
}

static void ubx_decode_relposned(uint8_t *msg, int len) {
	(void)len;

	static ubx_nav_relposned pos;
	int ind = 0;
	uint32_t flags;

	int version = ubx_get_U1(msg, &ind);
	ubx_get_U1(msg, &ind);

	pos.ref_station_id = ubx_get_U2(msg, &ind);
	pos.i_tow = ubx_get_U4(msg, &ind);
	pos.pos_n = (float)ubx_get_I4(msg, &ind) / 100.0;
	pos.pos_e = (float)ubx_get_I4(msg, &ind) / 100.0;
	pos.pos_d = (float)ubx_get_I4(msg, &ind) / 100.0;
	if (version == 1) {
		pos.pos_length = (float)ubx_get_I4(msg, &ind) / 100.0;
		pos.pos_heading = (float)ubx_get_I4(msg, &ind) / 100000.0;
		ind += 4;
	}

	pos.pos_n += (float)ubx_get_I1(msg, &ind) / 10000.0;
	pos.pos_e += (float)ubx_get_I1(msg, &ind) / 10000.0;
	pos.pos_d += (float)ubx_get_I1(msg, &ind) / 10000.0;
	if (version == 1)
		pos.pos_length += (float)ubx_get_I1(msg, &ind) / 10000.0;
	else
		ind += 1;

	pos.acc_n = (float)ubx_get_U4(msg, &ind) / 10000.0;
	pos.acc_e = (float)ubx_get_U4(msg, &ind) / 10000.0;
	pos.acc_d = (float)ubx_get_U4(msg, &ind) / 10000.0;
	if (version == 1) {
		pos.acc_length = (float)ubx_get_I4(msg, &ind) / 10000.0;
		pos.acc_heading = (float)ubx_get_I4(msg, &ind) / 100000.0;
		ind += 4;
	}

	flags = ubx_get_X4(msg, &ind);
	pos.fix_ok =                (flags >> 0) & 1;
	pos.diff_soln =             (flags >> 1) & 1;
	pos.rel_pos_valid =         (flags >> 2) & 1;
	pos.carr_soln =             (flags >> 3) & 3;
	pos.is_moving =             (flags >> 5) & 1;
	pos.ref_pos_miss =          (flags >> 6) & 1;
	pos.ref_obs_miss =          (flags >> 7) & 1;
	pos.rel_pos_heading_valid = (flags >> 8) & 1;
	pos.rel_pos_normalized =    (flags >> 9) & 1;

	if (rx_relposned) {
		rx_relposned(&pos);
	}

	if (m_print_next_relposned) {
		m_print_next_relposned = false;
		commands_printf(
				"NED RX\n"
				"i_tow: %d ms\n"
				"N: %.3f m\n"
				"E: %.3f m\n"
				"D: %.3f m\n"
				"Length: %.3f m\n"
				"Heading: %.3f\n"
				"N_Acc: %.3f m\n"
				"E_Acc: %.3f m\n"
				"D_Acc: %.3f m\n"
				"Length_Acc: %.3f m\n"
				"Heading_Acc: %.3f\n"
				"Fix OK: %d\n"
				"Diff Soln: %d\n"
				"Rel Pos Valid: %d\n"
				"Carr Soln: %d\n"
				"Is Moving: %d\n"
				"Ref Pos Miss: %d\n"
				"Ref Obs Miss: %d\n"
				"Heading Valid: %d\n"
				"Normalized: %d\n",
				pos.i_tow,
				(double)pos.pos_n, (double)pos.pos_e, (double)pos.pos_d, (double)pos.pos_length, (double)pos.pos_heading,
				(double)pos.acc_n, (double)pos.acc_e, (double)pos.acc_d, (double)pos.acc_length, (double)pos.acc_heading,
				pos.fix_ok, pos.diff_soln, pos.rel_pos_valid, pos.carr_soln, pos.is_moving,
				pos.ref_pos_miss, pos.ref_obs_miss, pos.rel_pos_heading_valid, pos.rel_pos_normalized);
	}
}

static void ubx_decode_svin(uint8_t *msg, int len) {
	(void)len;

	static ubx_nav_svin svin;
	int ind = 4;

	svin.i_tow = ubx_get_U4(msg, &ind);
	svin.dur = ubx_get_U4(msg, &ind);
	svin.meanX = (double)ubx_get_I4(msg, &ind) / D(100.0);
	svin.meanY = (double)ubx_get_I4(msg, &ind) / D(100.0);
	svin.meanZ = (double)ubx_get_I4(msg, &ind) / D(100.0);
	svin.meanX += (double)ubx_get_I1(msg, &ind) / D(10000.0);
	svin.meanY += (double)ubx_get_I1(msg, &ind) / D(10000.0);
	svin.meanZ += (double)ubx_get_I1(msg, &ind) / D(10000.0);
	ind += 1;
	svin.meanAcc = (float)ubx_get_U4(msg, &ind) / 10000.0;
	svin.obs = ubx_get_U4(msg, &ind);
	svin.valid = ubx_get_U1(msg, &ind);
	svin.active = ubx_get_U1(msg, &ind);

	if (rx_svin) {
		rx_svin(&svin);
	}

	if (m_print_next_svin) {
		m_print_next_svin = false;
		commands_printf(
				"SVIN RX\n"
				"i_tow: %d ms\n"
				"dur: %d s\n"
				"Mean X: %.3f m\n"
				"Mean Y: %.3f m\n"
				"Mean Z: %.3f m\n"
				"Mean ACC: %.3f m\n"
				"Valid: %d\n"
				"Active: %d\n",
				svin.i_tow, svin.dur,
				svin.meanX, svin.meanY, svin.meanZ, (double)svin.meanAcc,
				svin.valid, svin.active);
	}
}

static void ubx_decode_ack(uint8_t *msg, int len) {
	(void)len;

	int ind = 0;

	uint8_t cls_id = ubx_get_I1(msg, &ind);
	uint8_t msg_id = ubx_get_I1(msg, &ind);

	// TODO: Use these
	(void)cls_id;
	(void)msg_id;

	wait_was_ack = true;
	xSemaphoreGive(wait_sem);
}

static void ubx_decode_nak(uint8_t *msg, int len) {
	(void)len;

	int ind = 0;

	uint8_t cls_id = ubx_get_I1(msg, &ind);
	uint8_t msg_id = ubx_get_I1(msg, &ind);

	// TODO: Use these
	(void)cls_id;
	(void)msg_id;

	wait_was_ack = false;
	xSemaphoreGive(wait_sem);
}

// Note: Message version 0x01
static void ubx_decode_rawx(uint8_t *msg, int len) {
	(void)len;

	static ubx_rxm_rawx raw;
	int ind = 0;
	uint32_t flags;

	raw.rcv_tow = ubx_get_R8(msg, &ind);
	raw.week = ubx_get_U2(msg, &ind);
	raw.leaps = ubx_get_I1(msg, &ind);
	raw.num_meas = ubx_get_U1(msg, &ind);
	flags = ubx_get_X1(msg, &ind);
	raw.leap_sec = flags & 0x01;
	raw.clk_reset = flags & 0x02;

	ind = 16;

	if (raw.num_meas > 40) {
		commands_printf("Too many raw measurements to store in buffer: %d\n", raw.num_meas);
		return;
	}

	for (int i = 0;i < raw.num_meas;i++) {
		raw.obs[i].pr_mes = ubx_get_R8(msg, &ind);
		raw.obs[i].cp_mes = ubx_get_R8(msg, &ind);
		raw.obs[i].do_mes = ubx_get_R4(msg, &ind);
		raw.obs[i].gnss_id = ubx_get_U1(msg, &ind);
		raw.obs[i].sv_id = ubx_get_U1(msg, &ind);
		ind += 1;
		raw.obs[i].freq_id = ubx_get_U1(msg, &ind);
		raw.obs[i].locktime = ubx_get_U2(msg, &ind);
		raw.obs[i].cno = ubx_get_U1(msg, &ind);
		raw.obs[i].pr_stdev = ubx_get_X1(msg, &ind) & 0x0F;
		raw.obs[i].cp_stdev = ubx_get_X1(msg, &ind) & 0x0F;
		raw.obs[i].do_stdev = ubx_get_X1(msg, &ind) & 0x0F;
		flags = ubx_get_X1(msg, &ind);
		raw.obs[i].pr_valid = flags & 0x01;
		raw.obs[i].cp_valid = flags & 0x02;
		raw.obs[i].half_cyc_valid = flags & 0x04;
		raw.obs[i].half_cyc_sub = flags & 0x08;
		ind += 1;
	}

	if (rx_rawx) {
		rx_rawx(&raw);
	}

	if (m_print_next_rawx) {
		m_print_next_rawx = false;
		commands_printf(
				"RAWX RX\n"
				"tow: %.3f\n"
				"week: %d\n"
				"leap_sec: %d\n"
				"num_meas: %d\n"
				"pr_0: %.3f\n"
				"pr_1: %.3f\n",
				raw.rcv_tow, raw.week, raw.leap_sec, raw.num_meas,
				raw.obs[0].pr_mes, raw.obs[1].pr_mes);
	}
}

static void ubx_decode_nav_sat(uint8_t *msg, int len) {
	(void)len;

	static ubx_nav_sat sat; // NOTE: Store on heap.
	int ind = 0;

	sat.i_tow_ms = ubx_get_U4(msg, &ind);
	ubx_get_U1(msg, &ind);
	sat.num_sv = ubx_get_U1(msg, &ind);
	ubx_get_U1(msg, &ind);
	ubx_get_U1(msg, &ind);

	if (sat.num_sv > 128) {
		sat.num_sv = 128;
	}

	for (int i = 0;i < sat.num_sv;i++) {
		sat.sats[i].gnss_id = ubx_get_U1(msg, &ind);
		sat.sats[i].sv_id = ubx_get_U1(msg, &ind);
		sat.sats[i].cno = ubx_get_U1(msg, &ind);
		sat.sats[i].elev = ubx_get_I1(msg, &ind);
		sat.sats[i].azim = ubx_get_I2(msg, &ind);
		sat.sats[i].pr_res = (float)ubx_get_I2(msg, &ind) * 0.1;
		uint32_t flags = ubx_get_X4(msg, &ind);
		sat.sats[i].quality = (flags >> 0) & 0x07;
		sat.sats[i].used = (flags >> 3) & 0x01;
		sat.sats[i].health = (flags >> 4) & 0x03;
		sat.sats[i].diffcorr = (flags >> 6) & 0x01;
	}

	if (rx_nav_sat) {
		rx_nav_sat(&sat);
	}

	if (m_print_next_nav_sat) {
		m_print_next_nav_sat = false;
		int satsGps = 0;
		int satsGlo = 0;
		int satsGal = 0;
		int satsBds = 0;

		int visibleGps = 0;
		int visibleGlo = 0;
		int visibleGal = 0;
		int visibleBds = 0;

		for (int i = 0;i < sat.num_sv;i++) {
			ubx_nav_sat_info s = sat.sats[i];

			if (s.gnss_id == 0) {
				visibleGps++;
			} else if (s.gnss_id == 2) {
				visibleGal++;
			} else if (s.gnss_id == 3) {
				visibleBds++;
			} else if (s.gnss_id == 6) {
				visibleGlo++;
			}

			if (s.used && s.quality >= 4) {
				if (s.gnss_id == 0) {
					satsGps++;
				} else if (s.gnss_id == 2) {
					satsGal++;
				} else if (s.gnss_id == 3) {
					satsBds++;
				} else if (s.gnss_id == 6) {
					satsGlo++;
				}
			}
		}

		commands_printf(
				"         Visible   Used\n"
				"GPS:     %02d        %02d\n"
				"GLONASS: %02d        %02d\n"
				"Galileo: %02d        %02d\n"
				"BeiDou:  %02d        %02d\n"
				"Total:   %02d        %02d\n\n",
				visibleGps, satsGps,
				visibleGlo, satsGlo,
				visibleGal, satsGal,
				visibleBds, satsBds,
				visibleGps + visibleGlo + visibleGal + visibleBds,
				satsGps + satsGlo + satsGal + satsBds);
	}
}

static void ubx_decode_cfg_gnss(uint8_t *msg, int len) {
	(void)len;

	ubx_cfg_gnss cfg;
	int ind = 0;

	ubx_get_U1(msg, &ind);
	cfg.num_ch_hw = ubx_get_U1(msg, &ind);
	cfg.num_ch_use = ubx_get_U1(msg, &ind);
	cfg.num_blocks = ubx_get_U1(msg, &ind);

	if (cfg.num_blocks > 10) {
		cfg.num_blocks = 10;
	}

	for (int i = 0;i < cfg.num_blocks;i++) {
		cfg.blocks[i].gnss_id = ubx_get_U1(msg, &ind);
		cfg.blocks[i].minTrkCh = ubx_get_U1(msg, &ind);
		cfg.blocks[i].maxTrkCh = ubx_get_U1(msg, &ind);
		ubx_get_U1(msg, &ind);
		uint32_t flags = ubx_get_X4(msg, &ind);
		cfg.blocks[i].en = flags & 1;
		cfg.blocks[i].flags = flags >> 16 & 0xFF;
	}

	if (rx_gnss) {
		rx_gnss(&cfg);
	}

	if (m_print_next_cfg_gnss) {
		m_print_next_cfg_gnss = false;

		commands_printf(
				"CFG_GNSS RX\n"
				"TrkChHw   : %d\n"
				"TrkChUse  : %d\n"
				"Blocks    : %d\n",
				cfg.num_ch_hw, cfg.num_ch_use, cfg.num_blocks);

		for (int i = 0;i < cfg.num_blocks;i++) {
			commands_printf(
					"GNSS ID: %d, Enabled: %d\n"
					"MinTrkCh  : %d\n"
					"MaxTrkCh  : %d\n"
					"Flags     : %d\n",
					cfg.blocks[i].gnss_id,
					cfg.blocks[i].en,
					cfg.blocks[i].minTrkCh,
					cfg.blocks[i].maxTrkCh,
					cfg.blocks[i].flags);
		}
	}
}

static void ubx_decode_mon_ver(uint8_t *msg, int len) {
	if (m_print_next_mon_ver) {
		m_print_next_mon_ver = false;

		commands_printf(
				"MON_VER RX:\n"
				"SW: %s\n"
				"HW: %s\n"
				"Extensions:",
				msg, msg + 30);

		int ind = 40;

		while(ind < len) {
			commands_printf((const char*)msg + ind);
			ind += 30;
		}

		commands_printf(" ");
	}
}

static uint8_t ubx_get_U1(uint8_t *msg, int *ind) {
	return msg[(*ind)++];
}

static int8_t ubx_get_I1(uint8_t *msg, int *ind) {
	return (int8_t)msg[(*ind)++];
}

static uint8_t ubx_get_X1(uint8_t *msg, int *ind) {
	return msg[(*ind)++];
}

static uint16_t ubx_get_U2(uint8_t *msg, int *ind) {
	uint16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
					((uint16_t) msg[*ind]);
	*ind += 2;
	return res;
}

static int16_t ubx_get_I2(uint8_t *msg, int *ind) {
	int16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
					((uint16_t) msg[*ind]);
	*ind += 2;
	return res;
}

static uint16_t ubx_get_X2(uint8_t *msg, int *ind) {
	uint16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
					((uint16_t) msg[*ind]);
	*ind += 2;
	return res;
}

static uint32_t ubx_get_U4(uint8_t *msg, int *ind) {
	uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
					((uint32_t) msg[*ind + 2]) << 16 |
					((uint32_t) msg[*ind + 1]) << 8 |
					((uint32_t) msg[*ind]);
	*ind += 4;
	return res;
}

static int32_t ubx_get_I4(uint8_t *msg, int *ind) {
	int32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
					((uint32_t) msg[*ind + 2]) << 16 |
					((uint32_t) msg[*ind + 1]) << 8 |
					((uint32_t) msg[*ind]);
	*ind += 4;
	return res;
}

static uint32_t ubx_get_X4(uint8_t *msg, int *ind) {
	uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
					((uint32_t) msg[*ind + 2]) << 16 |
					((uint32_t) msg[*ind + 1]) << 8 |
					((uint32_t) msg[*ind]);
	*ind += 4;
	return res;
}

static float ubx_get_R4(uint8_t *msg, int *ind) {
	uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
					((uint32_t) msg[*ind + 2]) << 16 |
					((uint32_t) msg[*ind + 1]) << 8 |
					((uint32_t) msg[*ind]);
	*ind += 4;

	union asd {
	     float f;
	     uint32_t i;
	} x;

	x.i = res;

	return x.f;
}

static double ubx_get_R8(uint8_t *msg, int *ind) {
	uint64_t res =	((uint64_t) msg[*ind + 7]) << 56 |
					((uint64_t) msg[*ind + 6]) << 48 |
					((uint64_t) msg[*ind + 5]) << 40 |
					((uint64_t) msg[*ind + 4]) << 32 |
					((uint64_t) msg[*ind + 3]) << 24 |
					((uint64_t) msg[*ind + 2]) << 16 |
					((uint64_t) msg[*ind + 1]) << 8 |
					((uint64_t) msg[*ind]);
	*ind += 8;

	union asd {
	     double f;
	     uint64_t i;
	} x;

	x.i = res;

	return x.f;
}

static void ubx_put_U1(uint8_t *msg, int *ind, uint8_t data) {
	msg[(*ind)++] = data;
}

static void ubx_put_I1(uint8_t *msg, int *ind, int8_t data) {
	msg[(*ind)++] = data;
}

static void ubx_put_X1(uint8_t *msg, int *ind, uint8_t data) {
	msg[(*ind)++] = data;
}

static void ubx_put_U2(uint8_t *msg, int *ind, uint16_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
}

static void ubx_put_I2(uint8_t *msg, int *ind, int16_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
}

static void ubx_put_X2(uint8_t *msg, int *ind, uint16_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
}

static void ubx_put_U4(uint8_t *msg, int *ind, uint32_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
	msg[(*ind)++] = data >> 16;
	msg[(*ind)++] = data >> 24;
}

static void ubx_put_I4(uint8_t *msg, int *ind, int32_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
	msg[(*ind)++] = data >> 16;
	msg[(*ind)++] = data >> 24;
}

static void ubx_put_X4(uint8_t *msg, int *ind, uint32_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
	msg[(*ind)++] = data >> 16;
	msg[(*ind)++] = data >> 24;
}

static void ubx_put_R4(uint8_t *msg, int *ind, float data) {
	union asd {
		float f;
		uint64_t i;
	} x;

	x.f = data;

	msg[(*ind)++] = x.i;
	msg[(*ind)++] = x.i >> 8;
	msg[(*ind)++] = x.i >> 16;
	msg[(*ind)++] = x.i >> 24;
}

static void ubx_put_R8(uint8_t *msg, int *ind, double data) {
	union asd {
		double f;
		uint64_t i;
	} x;

	x.f = data;

	msg[(*ind)++] = x.i;
	msg[(*ind)++] = x.i >> 8;
	msg[(*ind)++] = x.i >> 16;
	msg[(*ind)++] = x.i >> 24;
	msg[(*ind)++] = x.i >> 32;
	msg[(*ind)++] = x.i >> 40;
	msg[(*ind)++] = x.i >> 48;
	msg[(*ind)++] = x.i >> 56;
}
