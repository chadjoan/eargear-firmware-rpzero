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

#define SHELL_WA_SIZE       THD_WA_SIZE(4096)

static BaseSequentialStream *bss;

// -------------------------------------------------------------------------- //
// LEET BCM2835 HAX GO HERE
// (Ugh but really, someday I hope that either myself or someone else reports
// and/or fixes the corresponding bugs in upstream.)



// This is a copy-paste of the i2c_lld_start function in `ChibiOS-RPi/os/hal/platforms/BCM2835/i2c_lld.c`
// except the correct GPIOX_PAD numbers are used here.
// (The original function uses pins (pads) 0 and 1, but those are not exposed
// on the Raspberry Pi Zero: only pins 2 and 3 are exposed! The function seems
// OK otherwise, but using the incorrect pins was very problematic.)
static void hax_i2c_lld_start(I2CDriver *i2cp) {
  /* Set up GPIO pins for I2C */
  bcm2835_gpio_fnsel(GPIO2_PAD, GPFN_ALT0);
  bcm2835_gpio_fnsel(GPIO3_PAD, GPFN_ALT0);

  uint32_t speed = i2cp->config->ic_speed;
  if (speed != 0 && speed != 100000)
    i2cp->device->clockDivider = BSC_CLOCK_FREQ / i2cp->config->ic_speed;

  i2cp->device->control |= BSC_I2CEN;
}

static void hax_i2cStart(I2CDriver *i2cp, const I2CConfig *config) {

  chDbgCheck((i2cp != NULL) && (config != NULL), "i2cStart");
  chDbgAssert((i2cp->state == I2C_STOP) || (i2cp->state == I2C_READY) ||
              (i2cp->state == I2C_LOCKED),
              "i2cStart(), #1",
              "invalid state");

  chSysLock();
  i2cp->config = config;
  hax_i2c_lld_start(i2cp);
  i2cp->state = I2C_READY;
  chSysUnlock();
}


// This is a copy-paste of the i2c_lld_stop function in `ChibiOS-RPi/os/hal/platforms/BCM2835/i2c_lld.c`
// except the correct GPIOX_PAD numbers are used here.
// (The original function uses pins (pads) 0 and 1, but those are not exposed
// on the Raspberry Pi Zero: only pins 2 and 3 are exposed! The function seems
// OK otherwise, but using the incorrect pins was very problematic.)
static void hax_i2c_lld_stop(I2CDriver *i2cp) {
  /* Set GPIO pin function to default */
  bcm2835_gpio_fnsel(GPIO2_PAD, GPFN_IN);
  bcm2835_gpio_fnsel(GPIO3_PAD, GPFN_IN);

  i2cp->device->control &= ~BSC_I2CEN;
}


static void hax_i2cStop(I2CDriver *i2cp) {

  chDbgCheck(i2cp != NULL, "i2cStop");
  chDbgAssert((i2cp->state == I2C_STOP) || (i2cp->state == I2C_READY) ||
              (i2cp->state == I2C_LOCKED),
              "i2cStop(), #1",
              "invalid state");

  chSysLock();
  hax_i2c_lld_stop(i2cp);
  i2cp->state = I2C_STOP;
  chSysUnlock();
}


static void hax_i2c_lld_init(void) {
  I2C0.device = BSC1_ADDR;
  i2cObjectInit(&I2C0);
}
// -------------------------------------------------------------------------- //



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

#define TE_PRESSURE_TEMP_I2C_ADDR   (0x76)

static size_t i2c_tx_count  = 0;
static size_t i2c_rx_count  = 0;
static size_t i2c_fail_count = 0;


uint8_t  handle_i2c_errors(I2CDriver *driver,  msg_t  stat,  char T_or_R)
{
	if ( stat == RDY_OK )
		return 0;

	size_t cmd_count = 0;

	switch(T_or_R) {
		case 't': case 'T': cmd_count = i2c_tx_count; break;
		case 'r': case 'R': cmd_count = i2c_rx_count; break;
	}

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
	
	I2CConfig  i2c_config;
	i2c_config.ic_speed = 1500;
	//i2cStart(&I2C0, &i2c_config);
	hax_i2cStart(&I2C0, &i2c_config);

	while (TRUE) {
		size_t  fail_count_before = i2c_fail_count;
		//I2CConfig  i2c_config;
		//i2c_config.ic_speed = 100000; // ???
		//i2c_config.ic_speed = 1500; // ???

		// Send the RESET command to initialize the sensor.
		txbuf[0] = 0x1E; // RESET
		txbuf[1] = 0x00;
		//i2cStart(&I2C0, &i2c_config);
		stat = i2cMasterTransmit(
			&I2C0, TE_PRESSURE_TEMP_I2C_ADDR,
			txbuf, 1, rxbuf, 0);
		//i2cStop(&I2C0);

		if ( handle_i2c_errors(&I2C0, stat, 'T') ) {
			chThdSleepMilliseconds(1000);
			i2c_tx_count++;
			continue;
		}
		i2c_tx_count++;

		// RESET succeeded (presumably).
		// Now read the 14 bytes of PROM. (Really it's 7 uint16_t's.)
		size_t prom_offset_idx = 0;
		for (; prom_offset_idx < 14; prom_offset_idx += 2 )
		{
			txbuf[0] = 0xA0 + prom_offset_idx;
			txbuf[1] = 0x00;
			//i2cStart(&I2C0, &i2c_config);
			stat = i2cMasterTransmit(
				&I2C0, TE_PRESSURE_TEMP_I2C_ADDR,
				txbuf, 1, rxbuf, 0);
			//i2cStop(&I2C0);

			// TODO: Should probably branch here, but not sure what to branch to.
			(void)handle_i2c_errors(&I2C0, stat, 'T');
			i2c_tx_count++;

			rxbuf[prom_offset_idx+0] = 0x00;
			rxbuf[prom_offset_idx+1] = 0x00;
			//i2cStart(&I2C0, &i2c_config);
			stat = i2cMasterReceive(
				&I2C0, TE_PRESSURE_TEMP_I2C_ADDR,
				rxbuf + prom_offset_idx, 2);
			//i2cStop(&I2C0);

			// TODO: Should probably branch here, but not sure what to branch to.
			(void)handle_i2c_errors(&I2C0, stat, 'R');
			i2c_rx_count++;
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
	hax_i2cStop(&I2C0);
	return 0;
}


#if 0
void print_i2c_errors(i2cflags_t  flags)
{
	
}
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
	shellInit();
	shellCreate(&shell_config, SHELL_WA_SIZE, NORMALPRIO + 1);

	// Set mode of onboard LED
	palSetPadMode(ONBOARD_LED_PORT, ONBOARD_LED_PAD, PAL_MODE_OUTPUT);

	// OH GOD WHY DID THIS WORK
	hax_i2c_lld_init();

	// Creates the blinker thread.
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

	// Creates the i2c thread.
	chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

	// Events servicing loop.
	chThdWait(chThdSelf());

	return 0;
}
