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


uint8_t  handle_i2c_errors(I2CDriver *driver,  msg_t  stat,  char T_or_R)
{
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
		i2cflags_t  errors = i2cGetErrors(driver);

		if ( errors & I2CD_BUS_ERROR ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Bus error.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors & I2CD_ARBITRATION_LOST ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Arbitration lost.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors & I2CD_ACK_FAILURE ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Acknowledgement failure.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors & I2CD_OVERRUN ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Overrun/Underrun.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors & I2CD_PEC_ERROR ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: PEC Error in reception.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors & I2CD_TIMEOUT ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: Hardware timeout. (I2C_TIMEOUT)\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( errors & I2CD_SMB_ALERT ) {
			chprintf(bss,
				"[%c%d.E%d] I2C Error: SMBus Alert.\n",
				T_or_R, cmd_count, i2c_fail_count);
			caught++;
		}

		if ( !caught ) {
			if ( errors == I2CD_NO_ERROR )
				chprintf(bss,
					"[%c%d.E%d] I2C Error: Function indicated error, but i2cGetErrors(...) returned I2C_NO_ERROR.\n",
					T_or_R, cmd_count, i2c_fail_count);
			else
				chprintf(bss,
					"[%c%d.E%d] I2C Error: Unknown error flag(s) returned: %04X.\n",
					T_or_R, cmd_count, i2c_fail_count, errors);
		}
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

//static WORKING_AREA(waThread1, 4096);
static WORKING_AREA(waThread1, 16384);
static msg_t Thread1(void *p)
{
	(void)p;
	chRegSetThreadName("i2c");

	I2CDriver  *i2c_driver = &I2CD;
	I2CConfig  i2c_config;
	i2c_config.ic_speed = 1500;
	i2cStart(i2c_driver, &i2c_config);

	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 driver.\n");
	ms8607_init();
	palSetPad(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01);

	chprintf(bss, "I2C.MS8607: (INFO)  Resetting sensor.\n");
	while ( ms8607_reset() != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) Sensor reset failed.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Attempting another reset.\n");
	}
	palSetPad(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02);

	chprintf(bss, "I2C.MS8607: (INFO)  Disabling heater.\n");
	while ( ms8607_disable_heater() != ms8607_status_ok ) {
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
	while ( ms8607_set_humidity_resolution(ms8607_humidity_resolution_10b)
		!= ms8607_status_ok )
	{
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to set humidity resolution.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying setting of humidity resolution to 10b.\n");
	}
	palSetPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);

	chprintf(bss, "I2C.MS8607: (INFO)  Setting pressure resolution to 2048.\n");
	ms8607_set_pressure_resolution(ms8607_pressure_resolution_osr_2048);
	palSetPad(PROGRESS_LED_PORT_05, PROGRESS_LED_PAD_05);

	chprintf(bss, "I2C.MS8607: (INFO)  Setting controller mode to NO HOLD.\n");
	ms8607_set_humidity_i2c_master_mode(ms8607_i2c_no_hold);
	palSetPad(PROGRESS_LED_PORT_06, PROGRESS_LED_PAD_06);

	chprintf(bss, "I2C.MS8607: (INFO)  Humidity controller mode was set.\n");
	while (TRUE) {
		enum ms8607_status  sensor_stat;
		float temperature = 0.0; // degC
		float pressure    = 0.0; // mbar
		float humidity    = 0.0; // %RH

		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (0/3)\n");
		palClearPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (1/3)\n");
		palClearPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (2/3)\n");
		palSetPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.0\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.01\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.012\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.0123\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.01234\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.012345\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.0123456\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.01234567\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.012345678\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with-some LEDs (3/3) ... done.0123456789\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with-some-LEDs (3/3) ... done.01234567890\n");
		chprintf(bss, "I2C.MS8607: (INFO)  About to mess with some LEDs (3/3) ... done.012345678901\n");

		chprintf(bss, "I2C.MS8607: (INFO)  Retrieving TPH (temperature-pressure-humidity) readings.\n");
		chprintf(bss, "boop\n\r\n\n\n");
		sensor_stat = ms8607_read_temperature_pressure_humidity(
				&temperature, &pressure, &humidity);
		if ( sensor_stat != ms8607_status_ok )
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
			chprintf(bss, "I2C.MS8607: (ERROR) Failed to read TPH data.\n");
			if ( sensor_stat == ms8607_status_crc_error )
				chprintf(bss, "I2C.MS8607: (ERROR) CRC Error.\n");
		}
		else
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
			chprintf(bss, "I2C.MS8607: (INFO)  TPH data received:\n");
			chprintf(bss, "    Temperature = %.2f degC\n", temperature);
			chprintf(bss, "    Pressure    = %.2f mbar\n", pressure);
			chprintf(bss, "    Humidity    = %.2f %RH\n",  humidity);
		}
		chThdSleepMilliseconds(1000);
	}

	// poor i2cStop statement can never execute.
	i2cStop(i2c_driver);
	return 0;
}

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
