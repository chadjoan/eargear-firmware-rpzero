/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "shell.h"
#include "chprintf.h"

#include "ms8607.h"

// Pin allocations:
// - Pins 2 & 3 are allocated for I2C.
// - Pins 14 & 15 are allocated for UART. (see also: sdcard-boilerplate/config.txt)
// - Pins 22-27 might be nice to avoid, because they might be used for JTAG
//     debugging at some point. Right now (at least, 2022-01-02) it doesn't work
//     though, so it's not a big deal.

#define PROGRESS_LED_PAD_01  GPIO18_PAD
#define PROGRESS_LED_PAD_02  GPIO17_PAD
#define PROGRESS_LED_PAD_03  GPIO10_PAD
#define PROGRESS_LED_PAD_04  GPIO9_PAD
#define PROGRESS_LED_PAD_05  GPIO11_PAD
#define PROGRESS_LED_PAD_06  GPIO5_PAD
#define PROGRESS_LED_PAD_07  GPIO6_PAD
#define PROGRESS_LED_PAD_08  GPIO13_PAD
#define PROGRESS_LED_PAD_09  GPIO19_PAD
//#define PROGRESS_LED_PAD_10  GPIO19_PAD

#define PROGRESS_LED_PORT_01  GPIO18_PORT
#define PROGRESS_LED_PORT_02  GPIO17_PORT
#define PROGRESS_LED_PORT_03  GPIO10_PORT
#define PROGRESS_LED_PORT_04  GPIO9_PORT
#define PROGRESS_LED_PORT_05  GPIO11_PORT
#define PROGRESS_LED_PORT_06  GPIO5_PORT
#define PROGRESS_LED_PORT_07  GPIO6_PORT
#define PROGRESS_LED_PORT_08  GPIO13_PORT
#define PROGRESS_LED_PORT_09  GPIO19_PORT
//#define PROGRESS_LED_PORT_10  GPIO19_PORT


#define SHELL_WA_SIZE       THD_WA_SIZE(4096)

static BaseSequentialStream *bss;



uint8_t  handle_i2c_errors(I2CDriver *driver,  msg_t  stat,  char T_or_R);
uint8_t  handle_i2c_errors_(I2CDriver *driver,  msg_t  stat,  char T_or_R, i2cflags_t *errors);

//void i2c_master_init(void);
static enum ms8607_status i2c_controller_read(void *caller_context, ms8607_i2c_controller_packet *const packet)
{
	I2CDriver *i2c_driver = caller_context;
	msg_t  stat = i2cMasterReceive(
		i2c_driver, packet->address,
		packet->data, packet->data_length);

	i2cflags_t  errors;
	if ( handle_i2c_errors_(i2c_driver, stat, 'R', &errors) )
	{
		if ( errors == I2CD_ACK_FAILURE )
			return ms8607_status_callback_i2c_nack;
		else
			return ms8607_status_callback_error;
	}
	else
		return ms8607_status_ok;
}

static enum ms8607_status i2c_controller_write(void *caller_context, ms8607_i2c_controller_packet *const packet)
{
	msg_t   stat;
	uint8_t rxbuf[1];
	I2CDriver *i2c_driver = caller_context;

	stat = i2cMasterTransmit(
		i2c_driver, packet->address,
		packet->data, packet->data_length, rxbuf, 0);

	if ( handle_i2c_errors(i2c_driver, stat, 'T') )
		return ms8607_status_callback_error;
	else
		return ms8607_status_ok;
}

static enum ms8607_status i2c_controller_write_no_stop(void *caller_context, ms8607_i2c_controller_packet *const packet)
{
	(void)caller_context;
	(void)packet;
	chprintf((BaseSequentialStream *)&SD1,
		"ERROR: Attempt to call unimplemented function `i2c_master_write_packet_wait_no_stop`.\n");
	return ms8607_status_callback_error;
}

static enum ms8607_status  sleep_ms(void *caller_context, uint32_t ms)
{
	(void)caller_context;
	chThdSleepMilliseconds(ms);
	return ms8607_status_ok;
}

static void  print_string(void *caller_context, const char *text)
{
	(void)caller_context;
	chprintf(bss, "%s", text);
}

static void  print_int64(void *caller_context,  int64_t  number, uint8_t pad_width,  ms8607_bool  pad_with_zeroes)
{
	(void)caller_context;
	(void)pad_width;
	(void)pad_with_zeroes;
	chprintf(bss, "%d", (int)number); // TODO: pad_width and pad character
}



#ifdef EXTENDED_SHELL

#define TEST_WA_SIZE        THD_WA_SIZE(4096)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  UNUSED(argv);
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  UNUSED(argv);
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    addr    stack prio refs     state time    name\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %-8lu %s\r\n",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
			 states[tp->p_state], (uint32_t)tp->p_time, tp->p_name);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
  Thread *tp;

  UNUSED(argv);
  if (argc > 0) {
    chprintf(chp, "Usage: test\r\n");
    return;
  }
  tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriority(),
                           TestThread, chp);
  if (tp == NULL) {
    chprintf(chp, "out of memory\r\n");
    return;
  }
  chThdWait(tp);
}

#endif // EXTENDED_SHELL

static void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]) {
  UNUSED(argv);
  if (argc > 0) {
    chprintf(chp, "Usage: reboot\r\n");
    return;
  }

  /* Watchdog will cause reset after 1 tick.*/
  watchdog_start(1);
}

static const ShellCommand commands[] = {
#ifdef EXTENDED_SHELL
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"test", cmd_test},
#endif
  {"reboot", cmd_reboot},
  {NULL, NULL}
};

static const ShellConfig shell_config = {
	(BaseSequentialStream *)&SD1,
	commands
};

#if 0
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *p) {
	(void)p;
	chRegSetThreadName("blinker");
	while (TRUE) {
		palClearPad(ONBOARD_LED_PORT, ONBOARD_LED_PAD);
		chThdSleepMilliseconds(100);
		palSetPad(ONBOARD_LED_PORT, ONBOARD_LED_PAD);
		chThdSleepMilliseconds(900);
	}
	return 0;
}
#endif

#define TE_PRESSURE_TEMP_I2C_ADDR   (0x76)

static size_t i2c_tx_count  = 0;
static size_t i2c_rx_count  = 0;
static size_t i2c_fail_count = 0;


uint8_t  handle_i2c_errors_(I2CDriver *driver,  msg_t  stat,  char T_or_R, i2cflags_t *i2c_errors)
{
	*i2c_errors = I2CD_NO_ERROR;
	size_t cmd_count = 0;

	switch(T_or_R) {
		case 't': case 'T': cmd_count = i2c_tx_count; i2c_tx_count++; break;
		case 'r': case 'R': cmd_count = i2c_rx_count; i2c_rx_count++; break;
	}

	if ( stat == RDY_OK )
		return 0;

	if ( stat == RDY_RESET )
	{
		uint8_t caught = 0;
		i2cflags_t  errors_ = i2cGetErrors(driver);

		if ( errors_ & I2CD_BUS_ERROR ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Bus error.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors_ & I2CD_ARBITRATION_LOST ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Arbitration lost.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors_ & I2CD_ACK_FAILURE ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Acknowledgement failure.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors_ & I2CD_OVERRUN ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Overrun/Underrun.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors_ & I2CD_PEC_ERROR ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: PEC Error in reception.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors_ & I2CD_TIMEOUT ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Hardware timeout. (I2C_TIMEOUT)\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors_ & I2CD_SMB_ALERT ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: SMBus Alert.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( !caught ) {
			if ( errors_ == I2CD_NO_ERROR )
				chprintf(bss,
					"[%c%d.E%d] I2C Error: Function indicated error, but i2cGetErrors(...) returned I2C_NO_ERROR.\n",
					T_or_R, cmd_count, i2c_fail_count);
			else
				chprintf(bss,
					"[%c%d.E%d] I2C Error: Unknown error flag(s) returned: %04X.\n",
					T_or_R, cmd_count, i2c_fail_count, errors_);
		}

		*i2c_errors = errors_;
	}
	else
	if ( stat == RDY_TIMEOUT )
		chprintf(bss, "[%c%d.E%d] I2C Timeout\n",
			T_or_R, cmd_count, i2c_fail_count);
	else
	// if ( stat != RDY_OK )
		chprintf(bss, "[%c%d.E%d] I2C Error: Unknown return code %d\n",
			T_or_R, cmd_count, i2c_fail_count, stat);

	i2c_fail_count++;
	return 1;
}

uint8_t  handle_i2c_errors(I2CDriver *driver,  msg_t  stat,  char T_or_R)
{
	i2cflags_t  errors;
	uint8_t result = handle_i2c_errors_(driver, stat, T_or_R, &errors);
	return result;
}

#if 0
static WORKING_AREA(waThread2, 4096);
static msg_t Thread2(void *p) {
	msg_t   stat;
	uint8_t txbuf[4];
	uint8_t rxbuf[16];

	(void)p;
	chRegSetThreadName("i2c");

#if 0
	while (TRUE) {
		palClearPad(GPIO2_PORT, GPIO2_PAD);
		palClearPad(GPIO3_PORT, GPIO3_PAD);
		chThdSleepMilliseconds(1000);
		palSetPad(GPIO2_PORT, GPIO2_PAD);
		palSetPad(GPIO3_PORT, GPIO3_PAD);
		chThdSleepMilliseconds(2000);
	}
	return 0;
#endif

	I2CDriver  *i2c_driver = &I2CD;
	I2CConfig  i2c_config;
	i2c_config.ic_speed = 1500;
	i2cStart(i2c_driver, &i2c_config);

	while (TRUE) {
		size_t  fail_count_before = i2c_fail_count;
		//I2CConfig  i2c_config;
		//i2c_config.ic_speed = 100000; // ???
		//i2c_config.ic_speed = 1500; // ???

		// Send the RESET command to initialize the sensor.
		txbuf[0] = 0x1E; // RESET
		txbuf[1] = 0x00;
		//i2cStart(i2c_driver, &i2c_config);
		stat = i2cMasterTransmit(
			i2c_driver, TE_PRESSURE_TEMP_I2C_ADDR,
			txbuf, 1, rxbuf, 0);
		//i2cStop(i2c_driver);

		if ( handle_i2c_errors(i2c_driver, stat, 'T') ) {
			chThdSleepMilliseconds(1000);
			continue;
		}

		// RESET succeeded (presumably).
		// Now read the 14 bytes of PROM. (Really it's 7 uint16_t's.)
		size_t prom_offset_idx = 0;
		for (; prom_offset_idx < 14; prom_offset_idx += 2 )
		{
			txbuf[0] = 0xA0 + prom_offset_idx;
			txbuf[1] = 0x00;
			//i2cStart(i2c_driver, &i2c_config);
			stat = i2cMasterTransmit(
				i2c_driver, TE_PRESSURE_TEMP_I2C_ADDR,
				txbuf, 1, rxbuf, 0);
			//i2cStop(i2c_driver);

			// TODO: Should probably branch here, but not sure what to branch to.
			(void)handle_i2c_errors(i2c_driver, stat, 'T');

			rxbuf[prom_offset_idx+0] = 0x00;
			rxbuf[prom_offset_idx+1] = 0x00;
			//i2cStart(i2c_driver, &i2c_config);
			stat = i2cMasterReceive(
				i2c_driver, TE_PRESSURE_TEMP_I2C_ADDR,
				rxbuf + prom_offset_idx, 2);
			//i2cStop(i2c_driver);

			// TODO: Should probably branch here, but not sure what to branch to.
			(void)handle_i2c_errors(i2c_driver, stat, 'R');
		}
		rxbuf[prom_offset_idx] = 0x00;

		if ( fail_count_before < i2c_fail_count )
			chprintf(bss, "I2C: Errors were encountered during last PROM read.\n");
		else
		{
			// Success
			chprintf(bss, "I2C: Recieved these bytes of PROM (in hex) :\n");
			chprintf(bss, "I2C: %02X %02X %02X %02X %02X %02X\n",
				rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5]);
			chprintf(bss, "I2C: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				rxbuf[6], rxbuf[7], rxbuf[8], rxbuf[9], rxbuf[10], rxbuf[11], rxbuf[12], rxbuf[13]);
		}

		chThdSleepMilliseconds(1000);
	}
	i2cStop(i2c_driver);
	return 0;
}
#endif

/*
static int64_t ticks2microsecs(systime_t ticks)
{
	// 100 seconds worth of microseconds, in system ticks.
	static int64_t hundred_microsecs_as_ticks = 0;
	if ( hundred_microsecs_as_ticks == 0 )
		hundred_microsecs_as_ticks = US2ST(100L);

	int64_t tmp_ticks = ticks;
	int64_t microsecs = (tmp_ticks*100LL)/hundred_microsecs_as_ticks;
	return microsecs;
}
*/

static ms8607_host_functions  host_funcs = {0};

#if 0
static void chibi_ms8607_init(void)
{
	ms8607_init_host_functions(&host_funcs);
	//memset(&host_funcs, 0, sizeof(host_funcs));
	host_funcs.i2c_controller_read          = &i2c_controller_read;
	host_funcs.i2c_controller_write         = &i2c_controller_write;
	host_funcs.i2c_controller_write_no_stop = &i2c_controller_write_no_stop;
	host_funcs.sleep_ms = &sleep_ms;

	ms8607_init(&host_funcs);
}
#endif


void chibi_ms8607_assign_functions(ms8607_host_functions *deps, void *caller_context)
{
	(void)caller_context;
	deps->i2c_controller_read          = &i2c_controller_read;
	deps->i2c_controller_write         = &i2c_controller_write;
	deps->i2c_controller_write_no_stop = &i2c_controller_write_no_stop;
	deps->sleep_ms                     = &sleep_ms;
	deps->print_string                 = &print_string;
	deps->print_int64                  = &print_int64;
}

// =============================================================================
// Demo that dumps experiments with various user register settings.
// TODO: didn't I find some error in the macro definitions or datasheet regarding the resolution settings?
#if 0
static enum ms8607_status  print_user_register(ms8607_sensor *sensor, I2CDriver *i2c_driver)
{
	enum ms8607_status  sensor_status;
#define HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE               0x04
#define HSENSOR_USER_REG_OTP_RELOAD_DISABLE                 0x02
	uint8_t user_register = 0;
	sensor_status = ms8607_hsensor_read_user_register(sensor, &user_register, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `hsensor_read_user_register`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Unable to continue.\n");
		return sensor_status;
	}

	chprintf(bss, "  USER register contents:\n");

	if( user_register & 0x01 )
		chprintf(bss, "    (bit 0) humidity res : 1\n");
	else
		chprintf(bss, "    (bit 0) humidity res : 0\n");

	if( user_register & HSENSOR_USER_REG_OTP_RELOAD_DISABLE )
		chprintf(bss, "    (bit 1) reserved     : 1 (OTP_RELOAD_DISABLE)\n");
	else
		chprintf(bss, "    (bit 1) reserved     : 0 (OTP_RELOAD_DISABLE)\n");

	//uint8_t header_enabled = 0;
	if( user_register & HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE) {
		//heater_enabled = 1;
		chprintf(bss, "    (bit 2) heater       : enabled\n");
	}
	else {
		chprintf(bss, "    (bit 2) heater       : disabled\n");
	}

		chprintf(bss, "    (bit 3) reserved     : %d\n", (user_register & 0x08) >> 3);
		chprintf(bss, "    (bit 4) reserved     : %d\n", (user_register & 0x10) >> 4);
		chprintf(bss, "    (bit 5) reserved     : %d\n", (user_register & 0x20) >> 5);

	if( user_register & 0x40 ) {
		chprintf(bss, "    (bit 6) battery stat : 1 (Vdd < 2.25V)\n");
	} else {
		chprintf(bss, "    (bit 6) battery stat : 0 (Vdd > 2.25V)\n");
	}

		chprintf(bss, "    (bit 7) humidity res : %d\n", (user_register & 0x80) >> 7);

	chprintf(bss, "\n");

	return ms8607_status_ok;
}

//static WORKING_AREA(waThread1, 4096);
static WORKING_AREA(waThread1, 16384);
//static WORKING_AREA(waThread1, 65536);
static msg_t Thread1(void *p)
{
	// Program for testing default on-sensor configuration state, volatility,
	// and general reset-related mechanics.

	(void)p;
	chRegSetThreadName("i2c");

	I2CDriver  *i2c_driver = &I2CD;
	I2CConfig  i2c_config;
	i2c_config.ic_speed = 1500;
	i2cStart(i2c_driver, &i2c_config);

	ms8607_sensor  sensor;
	enum ms8607_status  sensor_status;


	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 host functions / integration.\n");
	sensor_status = ms8607_init_and_assign_host_functions(&host_funcs, NULL, &chibi_ms8607_assign_functions);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `ms8607_init_and_assign_host_functions`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Unable to continue.\n");
		return 1;
	}
	//chibi_ms8607_init();

	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 sensor object.\n");
	sensor_status = ms8607_init_sensor(&sensor, &host_funcs);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `ms8607_init_sensor`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Unable to continue.\n");
		return 1;
	}

	palSetPad(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01);

	chprintf(bss, "I2C.MS8607: (INFO)  Resetting sensor.\n");
	while ( true ) {
		sensor_status = ms8607_reset(&sensor, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Sensor reset failed.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Attempting another reset.\n");
	}
	palSetPad(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02);

	chprintf(bss, "I2C.MS8607: (INFO)  Retrieving sensor settings.\n");

	chprintf(bss, "I2C.MS8607: (INFO)  Resetting sensor.\n");
	while ( true ) {
		sensor_status = ms8607_reset(&sensor, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Sensor reset failed.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Attempting another reset.\n");
	}

	sensor_status = print_user_register(&sensor, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (INFO)  Giving up.\n");
		return 1;
	}

	// ================== RESOLUTION SET ======================
	chprintf(bss, "I2C.MS8607: (INFO)  Setting humidity resolution to 12b.\n");
	while ( true )
	{
		sensor_status = ms8607_set_humidity_resolution(&sensor, ms8607_humidity_resolution_12b, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to set humidity resolution.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying setting of humidity resolution to 10b.\n");
	}
	palSetPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);

	sensor_status = print_user_register(&sensor, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (INFO)  Giving up.\n");
		return 1;
	}

	// ================== RESOLUTION SET ======================
	chprintf(bss, "I2C.MS8607: (INFO)  Setting humidity resolution to 11b.\n");
	while ( true )
	{
		sensor_status = ms8607_set_humidity_resolution(&sensor, ms8607_humidity_resolution_11b, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to set humidity resolution.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying setting of humidity resolution to 10b.\n");
	}
	palSetPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);

	sensor_status = print_user_register(&sensor, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (INFO)  Giving up.\n");
		return 1;
	}

	// ================== RESOLUTION SET ======================
	chprintf(bss, "I2C.MS8607: (INFO)  Setting humidity resolution to 10b.\n");
	while ( true )
	{
		sensor_status = ms8607_set_humidity_resolution(&sensor, ms8607_humidity_resolution_10b, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to set humidity resolution.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying setting of humidity resolution to 10b.\n");
	}
	palSetPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);

	sensor_status = print_user_register(&sensor, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (INFO)  Giving up.\n");
		return 1;
	}

	// ================== RESOLUTION SET ======================
	chprintf(bss, "I2C.MS8607: (INFO)  Setting humidity resolution to 8b.\n");
	while ( true )
	{
		sensor_status = ms8607_set_humidity_resolution(&sensor, ms8607_humidity_resolution_8b, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to set humidity resolution.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying setting of humidity resolution to 10b.\n");
	}
	palSetPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);

	sensor_status = print_user_register(&sensor, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (INFO)  Giving up.\n");
		return 1;
	}

	chprintf(bss, "I2C.MS8607: (INFO)  Resetting sensor.\n");
	while ( true ) {
		sensor_status = ms8607_reset(&sensor, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Sensor reset failed.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Attempting another reset.\n");
	}

	sensor_status = print_user_register(&sensor, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (INFO)  Giving up.\n");
		return 1;
	}

	chprintf(bss, "I2C.MS8607: (INFO)  Enabling heater.\n");
	sensor_status = ms8607_enable_heater(&sensor, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `ms8607_enable_heater`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (INFO)  Giving up.\n");
		return 1;
	}

	sensor_status = print_user_register(&sensor, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (INFO)  Giving up.\n");
		return 1;
	}

	chprintf(bss, "I2C.MS8607: (INFO)  Resetting sensor.\n");
	while ( true ) {
		sensor_status = ms8607_reset(&sensor, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Sensor reset failed.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Attempting another reset.\n");
	}

	sensor_status = print_user_register(&sensor, i2c_driver);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (INFO)  Giving up.\n");
		return 1;
	}

	chprintf(bss, "I2C.MS8607: (INFO)  Halting.\n");
	i2cStop(i2c_driver);
	return 0;
}
#endif

// =============================================================================
// Attempt at proving a polling interface.
#if 0
//static WORKING_AREA(waThread1, 4096);
static WORKING_AREA(waThread1, 16384);
//static WORKING_AREA(waThread1, 65536);
static msg_t Thread1(void *p)
{
	(void)p;
	chRegSetThreadName("i2c");

	I2CDriver  *i2c_driver = &I2CD;
	I2CConfig  i2c_config;
	i2c_config.ic_speed = 1500;
	i2cStart(i2c_driver, &i2c_config);

	ms8607_sensor  sensor;
	enum ms8607_status  sensor_status;

	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 host functions / integration.\n");
	sensor_status = ms8607_init_and_assign_host_functions(&host_funcs, NULL, &chibi_ms8607_assign_functions);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `ms8607_init_and_assign_host_functions`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Unable to continue.\n");
		return 1;
	}
	//chibi_ms8607_init();

	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 sensor object.\n");
	sensor_status = ms8607_init_sensor(&sensor, &host_funcs);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `ms8607_init_sensor`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Unable to continue.\n");
		return 1;
	}

	palSetPad(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01);

	chprintf(bss, "I2C.MS8607: (INFO)  Resetting sensor.\n");
	while ( true ) {
		sensor_status = ms8607_reset(&sensor, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Sensor reset failed.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Attempting another reset.\n");
	}
	palSetPad(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02);

	chprintf(bss, "I2C.MS8607: (INFO)  Disabling heater.\n");
	while ( true ) {
		sensor_status = ms8607_disable_heater(&sensor, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to disable heater.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying heater disable.\n");
	}
	palSetPad(PROGRESS_LED_PORT_03, PROGRESS_LED_PAD_03);

	// TODO: Once we have a polling-type interface implemented, switch to
	// higher resolutions. The driver waits SECONDS for these results, with
	// 1 second being for the lowest resolution. By contrast, the datasheet
	// suggests that we can expect the highest resolution results in under
	// 20 MILLIseconds. The author of this driver might be assuming an
	// absolutely huge margin-of-error. And it's not even a polling interface;
	// it just kind of assumes that everything will work out.
	// (Now, even with polling, we should probably have some time-out interval,
	// and if that elapses without producing measurements, we should request
	// new measurements or something. Part of this project will involve using
	// a polling interface to see how quickly the sensor respondes /in practice/,
	// and that information would guide the choice of sensible time-out values.
	// Obviously, if there is any sustained failure to obtain readings, we
	// should DO SOMETHING ABOUT IT, like create a loud noise or something
	// to wake up the poor user. Crash-handling is a potentially complex subject,
	// but by even thinking about it, we'll be in better shape than the CPAP
	// machines themselves... last I checked, those things just give up if
	// there's ever a problem. Kinda scary, honestly. I wouldn't want my CPAP
	// machine to just /stop/ if it encountered errors. I'd want to be woken
	// up so I can DO something about it, or at least avoid being in a
	// vulnerable-to-the-death type of situation. You can't die from sleep
	// apnea when you're awake!  Ironically, the eargear might be in a position
	// to detect such things and provide such fallbacks.)

	chprintf(bss, "I2C.MS8607: (INFO)  Setting humidity resolution to 10b.\n");
	while ( true )
	{
		sensor_status = ms8607_set_humidity_resolution(&sensor, ms8607_humidity_resolution_10b, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to set humidity resolution.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying setting of humidity resolution to 10b.\n");
	}
	palSetPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);

	chprintf(bss, "I2C.MS8607: (INFO)  Setting pressure resolution to 2048.\n");
	ms8607_set_pressure_resolution(&sensor, ms8607_pressure_resolution_osr_2048, i2c_driver);
	palSetPad(PROGRESS_LED_PORT_05, PROGRESS_LED_PAD_05);

	chprintf(bss, "I2C.MS8607: (INFO)  Setting controller mode to NO HOLD.\n");
	ms8607_set_humidity_i2c_controller_mode(&sensor, ms8607_i2c_no_hold, i2c_driver);
	palSetPad(PROGRESS_LED_PORT_06, PROGRESS_LED_PAD_06);

	chprintf(bss, "I2C.MS8607: (INFO)  Humidity controller mode was set.\n");
	if (TRUE) {
		int32_t temperature = 0.0; // degC
		int32_t pressure    = 0.0; // mbar
		int32_t humidity    = 0.0; // %RH

		palClearPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
		palClearPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
		palSetPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);

		chprintf(bss, "\n");
		chprintf(bss, "I2C.MS8607: (INFO)  Retrieving TPH (temperature-pressure-humidity) readings.\n");
		sensor_status = ms8607_read_temperature_pressure_humidity_int32(
				&sensor, &temperature, &pressure, &humidity, i2c_driver);
		if ( sensor_status != ms8607_status_ok )
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
			if ( sensor_status != ms8607_status_callback_error )
				chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
			chprintf(bss, "I2C.MS8607: (ERROR) Failed to read TPH data.\n");
		}
		else
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
			chprintf(bss, "I2C.MS8607: (INFO)  TPH data received:\n");
			// chprintf(bss, "    Temperature = %.2f degC\n", temperature);
			// chprintf(bss, "    Pressure    = %.2f mbar\n", pressure);
			// chprintf(bss, "    Humidity    = %.2f %RH\n",  humidity);
			chprintf(bss, "    Temperature = %d.%d%d%d degC\n", (int)(temperature/1000), (int)((temperature/100)%10), (int)((temperature/10)%10), (int)(temperature%10) );
			chprintf(bss, "    Pressure    = %d.%d%d%d mbar\n", (int)(pressure/1000),    (int)((pressure/100)%10),    (int)((pressure/10)%10),    (int)(pressure%10) );
			chprintf(bss, "    Humidity    = %d.%d%d%d %%RH\n", (int)(humidity/1000),    (int)((humidity/100)%10),    (int)((humidity/10)%10),    (int)(humidity%10) );
		}
		chThdSleepMilliseconds(1000);
	}

	//systime_t  the_time_it_wudj = chVTGetSystemTime();
	int64_t        usecs = 0;

	chprintf(bss, "\n");
	chprintf(bss, "I2C.MS8607: (INFO)  Warm-up completed. LET'S GET DANGEROUS!\n");
	while (TRUE) {
		//sysinterval_t  the_time_it_idj;
		//time_usecs_t   usecs;
		//systime_t      the_time_it_idj;
		//int64_t        usecs;

		chprintf(bss, "\n");

		//the_time_it_idj = chVTTimeElapsedSinceX(the_time_it_wudj);
		//usecs = chTimeI2US(the_time_it_idj);
		//the_time_it_idj = chTimeNow();
		//usecs = ticks2microsecs(the_time_it_idj);
		//chprintf(bss, "I2C.MS8607: (INFO) %04d.%03d milliseconds elapsed.\n", (int)(usecs/1000), (int)(usecs%1000));
		chprintf(bss, "I2C.MS8607: (INFO) %04d milliseconds elapsed.\n", (int)(usecs/1000));

		sensor_status = psensor_request_temperature(&sensor, usecs, i2c_driver);
		if( sensor_status != ms8607_status_ok) {
			chprintf(bss, "I2C.MS8607: (ERROR) psensor_request_temperature(...) failed.\n");
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
			continue;
		}
		chprintf(bss, "I2C.MS8607: (INFO) sent temperature request.\n");

#define HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND               0xF5
		sensor_status = hsensor_write_command(&sensor, HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND, i2c_driver);
		if( sensor_status != ms8607_status_ok) {
			chprintf(bss, "I2C.MS8607: (ERROR) hsensor_write_command(...) failed.\n");
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
			chThdSleepMicroseconds(1000);
			usecs += 1000; // ugh.
			continue;
		}
		chprintf(bss, "I2C.MS8607: (INFO) sent humidity request.\n");

		uint32_t temperature = 0;
		float    humidity    = 0;
		int need_temperature = 1;
		int need_humidity    = 1;
		while (need_temperature || need_humidity)
		{
			//the_time_it_idj = chVTTimeElapsedSinceX(the_time_it_wudj);
			//usecs = chTimeI2US(the_time_it_idj);
			//the_time_it_idj = chTimeNow();
			//usecs = ticks2microsecs(the_time_it_idj);
			chprintf(bss, "I2C.MS8607: (INFO) %04d milliseconds elapsed.\n", (int)(usecs/1000));

			if ( need_humidity ) {
				sensor_status = hsensor_poll_relative_humidity(&sensor, &humidity, i2c_driver);
				if ( sensor_status == ms8607_status_ok ) {
					need_humidity = 0;
					chprintf(bss, "I2C.MS8607: (INFO) Got humidity reading.\n");
				}
				else if ( sensor_status == ms8607_status_waiting || sensor_status == ms8607_status_callback_i2c_nack ) {
					chprintf(bss, "I2C.MS8607: (INFO) Waited for humidity reading. Not yet.\n");
				}
				else {
					chprintf(bss, "I2C.MS8607: (ERROR) hsensor_poll_relative_humidity(...) failed.\n");
					chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
				}
			}

			if ( need_temperature ) {
				sensor_status = psensor_poll_raw_temperature(&sensor, usecs, &temperature, i2c_driver);
				if ( sensor_status == ms8607_status_ok ) {
					need_temperature = 0;
					chprintf(bss, "I2C.MS8607: (INFO) Got temperature reading.\n");
				}
				else if ( sensor_status == ms8607_status_waiting ) {
					chprintf(bss, "I2C.MS8607: (INFO) Waited for temperature reading. Not yet.\n");
				}
				else {
					chprintf(bss, "I2C.MS8607: (ERROR) psensor_poll_raw_temperature(...) failed.\n");
					chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
				}
			}

			chThdSleepMicroseconds(1000);
			usecs += 1000; // ugh.
		}

		if ( need_temperature || need_humidity ) {
			// Errors occurred. (They should have already been reported by now.)
			chThdSleepMicroseconds(1000);
			usecs += 1000; // ugh.
			continue;
		}

		chprintf(bss, "I2C.MS8607: (INFO)  TH data received:\n");
		chprintf(bss, "    Temperature = %d (unknown units)\n", temperature);
		chprintf(bss, "    Humidity    = %.2f %RH\n",  humidity);
	}

	// poor i2cStop statement can never execute.
	i2cStop(i2c_driver);
	return 0;
}
#endif

// =============================================================================
// Version that uses mostly-original driver API.
#if 1
//static WORKING_AREA(waThread1, 4096);
static WORKING_AREA(waThread1, 16384);
//static WORKING_AREA(waThread1, 65536);
static msg_t Thread1(void *p)
{
	(void)p;
	chRegSetThreadName("i2c");

	I2CDriver  *i2c_driver = &I2CD;
	I2CConfig  i2c_config;
	i2c_config.ic_speed = 1500;
	i2cStart(i2c_driver, &i2c_config);

	enum ms8607_status  sensor_status;
	ms8607_sensor       sensor;

#if 0
	// TODO: Probably delete this old code.
	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 driver.\n");
	chibi_ms8607_init();
	palSetPad(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01);
#endif

	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 host functions / integration.\n");
	sensor_status = ms8607_init_and_assign_host_functions(&host_funcs, NULL, &chibi_ms8607_assign_functions);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `ms8607_init_and_assign_host_functions`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Unable to continue.\n");
		return 1;
	}

	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 sensor object.\n");
	sensor_status = ms8607_init_sensor(&sensor, &host_funcs);
	if ( sensor_status != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `ms8607_init_sensor`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Unable to continue.\n");
		return 1;
	}
	palSetPad(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01);

	chprintf(bss, "I2C.MS8607: (INFO)  Resetting sensor.\n");
	while ( true ) {
		sensor_status = ms8607_reset(&sensor, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Sensor reset failed.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Attempting another reset.\n");
	}
	palSetPad(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02);

	chprintf(bss, "I2C.MS8607: (INFO)  Disabling heater.\n");
	while ( true ) {
		sensor_status = ms8607_disable_heater(&sensor, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to disable heater.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying heater disable.\n");
	}
	palSetPad(PROGRESS_LED_PORT_03, PROGRESS_LED_PAD_03);

	// TODO: Once we have a polling-type interface implemented, switch to
	// higher resolutions. The driver waits SECONDS for these results, with
	// 1 second being for the lowest resolution. By contrast, the datasheet
	// suggests that we can expect the highest resolution results in under
	// 20 MILLIseconds. The author of this driver might be assuming an
	// absolutely huge margin-of-error. And it's not even a polling interface;
	// it just kind of assumes that everything will work out.
	// (Now, even with polling, we should probably have some time-out interval,
	// and if that elapses without producing measurements, we should request
	// new measurements or something. Part of this project will involve using
	// a polling interface to see how quickly the sensor respondes /in practice/,
	// and that information would guide the choice of sensible time-out values.
	// Obviously, if there is any sustained failure to obtain readings, we
	// should DO SOMETHING ABOUT IT, like create a loud noise or something
	// to wake up the poor user. Crash-handling is a potentially complex subject,
	// but by even thinking about it, we'll be in better shape than the CPAP
	// machines themselves... last I checked, those things just give up if
	// there's ever a problem. Kinda scary, honestly. I wouldn't want my CPAP
	// machine to just /stop/ if it encountered errors. I'd want to be woken
	// up so I can DO something about it, or at least avoid being in a
	// vulnerable-to-the-death type of situation. You can't die from sleep
	// apnea when you're awake!  Ironically, the eargear might be in a position
	// to detect such things and provide such fallbacks.)

	chprintf(bss, "I2C.MS8607: (INFO)  Setting humidity resolution to 10b.\n");
	while ( true )
	{
		//sensor_status = ms8607_set_humidity_resolution(i2c_driver, ms8607_humidity_resolution_10b);
		sensor_status = ms8607_set_humidity_resolution(&sensor, ms8607_humidity_resolution_12b, i2c_driver);
		if ( sensor_status == ms8607_status_ok )
			break;
		if ( sensor_status != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to set humidity resolution.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying setting of humidity resolution to 10b.\n");
	}
	palSetPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);

	chprintf(bss, "I2C.MS8607: (INFO)  Setting pressure resolution to 2048.\n");
	ms8607_set_pressure_resolution(&sensor, ms8607_pressure_resolution_osr_2048, i2c_driver);
	palSetPad(PROGRESS_LED_PORT_05, PROGRESS_LED_PAD_05);

	chprintf(bss, "I2C.MS8607: (INFO)  Setting controller mode to NO HOLD.\n");
	ms8607_set_humidity_i2c_controller_mode(&sensor, ms8607_i2c_no_hold, i2c_driver);
	palSetPad(PROGRESS_LED_PORT_06, PROGRESS_LED_PAD_06);

	chprintf(bss, "I2C.MS8607: (INFO)  Humidity controller mode was set.\n");
	while (TRUE) {
#if 0
		float temperature = 0.0; // degC
		float pressure    = 0.0; // mbar
		float humidity    = 0.0; // %RH
#endif
		int32_t temperature = 0.0; // degC
		int32_t pressure    = 0.0; // mbar
		int32_t humidity    = 0.0; // %RH

		palClearPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
		palClearPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
		palSetPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);

		chprintf(bss, "\n");
		chprintf(bss, "I2C.MS8607: (INFO)  Retrieving TPH (temperature-pressure-humidity) readings.\n");
		sensor_status = ms8607_read_temperature_pressure_humidity_int32(
				&sensor, &temperature, &pressure, &humidity, i2c_driver);
		if ( sensor_status != ms8607_status_ok )
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
			if ( sensor_status != ms8607_status_callback_error )
				chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status));
			chprintf(bss, "I2C.MS8607: (ERROR) Failed to read TPH data.\n");
		}
		else
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
			chprintf(bss, "I2C.MS8607: (INFO)  TPH data received:\n");
			// chprintf(bss, "    Temperature = %.2f degC\n", temperature);
			// chprintf(bss, "    Pressure    = %.2f mbar\n", pressure);
			// chprintf(bss, "    Humidity    = %.2f %RH\n",  humidity);
			chprintf(bss, "    Temperature = %d.%d%d%d degC\n", (int)(temperature/1000), (int)((temperature/100)%10), (int)((temperature/10)%10), (int)(temperature%10) );
			chprintf(bss, "    Pressure    = %d.%d%d%d mbar\n", (int)(pressure/1000),    (int)((pressure/100)%10),    (int)((pressure/10)%10),    (int)(pressure%10) );
			chprintf(bss, "    Humidity    = %d.%d%d%d %%RH\n", (int)(humidity/1000),    (int)((humidity/100)%10),    (int)((humidity/10)%10),    (int)(humidity%10) );
		}
		chThdSleepMilliseconds(1000);
	}

	// poor i2cStop statement can never execute.
	i2cStop(i2c_driver);
	return 0;
}
#endif

#if 0
#define N_SENSORS  (4)
static float t[N_SENSORS];
static float p[N_SENSORS];
static float h[N_SENSORS];

void my_sensor_update_func(ms8607_sensor *sensors, float *temps, float *pressures, float *humidities, size_t n)
{
	size_t i = 0;
	for (; i < n; i++)
	{
		switch(ms8607_get_state(sensors[i]))
		{
			MS8607_STATE_DISCONNECTED:
				ms8607_reset(sensors[i]);
				// Set resolutions and stuff
				break;

			MS8607_STATE_READY:
				ms8607_send_request_for_temperature(sensors[i]);
				ms8607_send_request(sensors[i], MS8607_REQUEST_TEMPERATURE_AND_HUMIDITY);
				break;

			... pressure and humidity? full async api? etc ...
		}
	}
}

#define MS8607_REQUEST_TEMPERATURE               (1)
#define MS8607_REQUEST_PRESSURE                  (2)
#define MS8607_REQUEST_HUMIDITY                  (3)
#define MS8607_REQUEST_TEMPERATURE_AND_HUMIDITY  (4)
#define MS8607_REQUEST_HUMIDITY_AND_TEMPERATURE  (MS8607_REQUEST_TEMPERATURE_AND_HUMIDITY)
#define MS8607_REQUEST_PRESSURE_AND_HUMIDITY     (5)
#define MS8607_REQUEST_HUMIDITY_AND_PRESSURE     (MS8607_REQUEST_PRESSURE_AND_HUMIDITY)


	if(ms8607_poll_tph(sensor0, &t0, &p0, &h0)
	&& ms8607_poll_tph(sensor1, &t1, &p1, &h1)
	&& ms8607_poll_tph(sensor2, &t2, &p2, &h2))
	{
		
	}
	
	ms8607_poll_temperature_pressure_humidity(
#endif


/// Application entry point.
int main(void) {
	bss = (BaseSequentialStream *)&SD1;

	halInit();
	chSysInit();

	// Serial port initialization.
	sdStart(&SD1, NULL); 
	chprintf((BaseSequentialStream *)&SD1, "Main (SD1 started)\r\n");

	// Shell initialization.
#if 0
	shellInit();
	shellCreate(&shell_config, SHELL_WA_SIZE, NORMALPRIO + 1);
#endif

	// Set mode of onboard LEDs

	palSetPadMode(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_03, PROGRESS_LED_PAD_03, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_05, PROGRESS_LED_PAD_05, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_06, PROGRESS_LED_PAD_06, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09, PAL_MODE_OUTPUT);
	//palSetPadMode(PROGRESS_LED_PORT_10, PROGRESS_LED_PAD_10, PAL_MODE_OUTPUT);

	palClearPad(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01);
	palClearPad(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02);
	palClearPad(PROGRESS_LED_PORT_03, PROGRESS_LED_PAD_03);
	palClearPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);
	palClearPad(PROGRESS_LED_PORT_05, PROGRESS_LED_PAD_05);
	palClearPad(PROGRESS_LED_PORT_06, PROGRESS_LED_PAD_06);
	palClearPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
	palClearPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
	palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
	//palClearPad(PROGRESS_LED_PORT_10, PROGRESS_LED_PAD_10);

	// Set the JTAG TRST pin HIGH.
	// https://forums.raspberrypi.com/viewtopic.php?t=286115
	palSetPadMode(GPIO22_PORT, GPIO22_PAD, PAL_MODE_OUTPUT);
	palSetPad(GPIO22_PORT, GPIO22_PAD);

	//palSetPadMode(ONBOARD_LED_PORT, ONBOARD_LED_PAD, PAL_MODE_OUTPUT);

	// Creates the blinker thread.
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
/*
	// Creates the i2c thread.
	chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);
*/
	// Events servicing loop.
	chThdWait(chThdSelf());

	return 0;
}
