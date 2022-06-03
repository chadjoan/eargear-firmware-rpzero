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
#include "ms5840.h"

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

#define I2C_EXPANDER_ADDR ((i2caddr_t)0x70)

static uint8_t  handle_i2c_errors(I2CDriver *driver,  msg_t  stat,  char T_or_R);
static uint8_t  handle_i2c_errors_(I2CDriver *driver,  msg_t  stat,  char T_or_R, i2cflags_t *errors);

typedef struct i2c_driver_meta
{
	BaseSequentialStream  *stdout;
	I2CDriver             *i2c_driver;
	uint8_t               last_i2c_expander_port_selection_bits;
}
i2c_driver_meta;

typedef struct i2c_context
{
	i2c_driver_meta  *meta;
	uint8_t          i2c_expander_port_selection_bits;
}
i2c_context;

static void i2c_driver_meta_init(
	i2c_driver_meta       *this,
	BaseSequentialStream  *stdout,
	I2CDriver             *driver)
{
	this->stdout     = stdout;
	this->i2c_driver = driver;
	this->last_i2c_expander_port_selection_bits = 0;
}

static void i2c_print_port_selection_bits(BaseSequentialStream *stdout, uint8_t bits)
{
	for ( int16_t bit_index = 7; bit_index >= 0; bit_index-- )
	{
		uint8_t the_bit = (bits >> bit_index) & 1;
		if ( the_bit )
			chprintf(stdout, "1");
		else
			chprintf(stdout, "0");
	}
}

static i2cflags_t i2c_controller_mux(i2c_context *target, char T_or_R)
{
	i2c_driver_meta  *meta = target->meta;

	if ( meta->last_i2c_expander_port_selection_bits == target->i2c_expander_port_selection_bits )
	{
		//chprintf(meta->stdout, "I2C.TCA9548A: (INFO)  I2C multiplexer already has correct ports selected. No change.\n");
		return I2CD_NO_ERROR;
	}
	else
	{
		i2cflags_t  errors;
		msg_t       stat;
		uint8_t     txbuf[1];
		uint8_t     rxbuf[1];

		chprintf(meta->stdout, "I2C.TCA9548A: (INFO)  I2C multiplexer select bits changing:\n");
		chprintf(meta->stdout, "    ");
		i2c_print_port_selection_bits(meta->stdout, meta->last_i2c_expander_port_selection_bits);
		chprintf(meta->stdout, " -> ");
		i2c_print_port_selection_bits(meta->stdout, target->i2c_expander_port_selection_bits);
		chprintf(meta->stdout, "\n");

		txbuf[0] = target->i2c_expander_port_selection_bits;
		stat = i2cMasterTransmit(
			meta->i2c_driver, I2C_EXPANDER_ADDR, txbuf, sizeof(txbuf), rxbuf, 0);

		if ( handle_i2c_errors_(meta->i2c_driver, stat, T_or_R, &errors) ) {
			chprintf(meta->stdout, "I2C.TCA9548A: (ERROR) Problem when communicating with I2C multiplexer.\n");
			chprintf(meta->stdout, "I2C.TCA9548A: (ERROR) Unable to continue.\n");
			return errors;
		}

		meta->last_i2c_expander_port_selection_bits = target->i2c_expander_port_selection_bits;
		return I2CD_NO_ERROR;
	}
}

//void i2c_master_init(void);
static enum ms8607_status i2c_error_to_ms8607_error(i2cflags_t  errors)
{
	if ( errors == I2CD_ACK_FAILURE )
		return ms8607_status_callback_i2c_nack;
	else
		return ms8607_status_callback_error;
}

static enum ms8607_status i2c_controller_read_ms8607(void *caller_context, ms8607_i2c_controller_packet *const packet)
{
	i2cflags_t   errors;
	i2c_context  *ctx = caller_context;

	errors = i2c_controller_mux(ctx, 'R');
	if ( errors != I2CD_NO_ERROR ) {
		// Don't return NACK, as we didn't even have
		// a chance to communicate with the ms8607.
		return ms8607_status_callback_error;
	}

	msg_t  stat = i2cMasterReceive(
		ctx->meta->i2c_driver, packet->address,
		packet->data, packet->data_length);

	if ( handle_i2c_errors_(ctx->meta->i2c_driver, stat, 'R', &errors) )
		return i2c_error_to_ms8607_error(errors);
	else
		return ms8607_status_ok;
}

static enum ms8607_status i2c_controller_write_ms8607(void *caller_context, ms8607_i2c_controller_packet *const packet)
{
	msg_t   stat;
	uint8_t rxbuf[1];
	i2cflags_t   errors;
	i2c_context  *ctx = caller_context;

	errors = i2c_controller_mux(ctx, 'T');
	if ( errors != I2CD_NO_ERROR ) {
		// Don't return NACK, as we didn't even have
		// a chance to communicate with the ms8607.
		return ms8607_status_callback_error;
	}

	stat = i2cMasterTransmit(
		ctx->meta->i2c_driver, packet->address,
		packet->data, packet->data_length, rxbuf, 0);

	if ( handle_i2c_errors(ctx->meta->i2c_driver, stat, 'T') )
		return ms8607_status_callback_error;
	else
		return ms8607_status_ok;
}

static enum ms8607_status i2c_controller_write_no_stop_ms8607(void *caller_context, ms8607_i2c_controller_packet *const packet)
{
	i2c_context *ctx = caller_context;
	(void)packet;
	chprintf(ctx->meta->stdout,
		"ERROR: Attempt to call unimplemented function `i2c_master_write_packet_wait_no_stop`.\n");
	return ms8607_status_callback_error;
}

static enum ms8607_status  sleep_ms_ms8607(void *caller_context, uint32_t ms)
{
	(void)caller_context;
	chThdSleepMilliseconds(ms);
	return ms8607_status_ok;
}

static void  print_string_ms8607(void *caller_context, const char *text)
{
	i2c_context *ctx = caller_context;
	chprintf(ctx->meta->stdout, "%s", text);
}

static void  print_int64_ms8607(void *caller_context,  int64_t  number, uint8_t pad_width,  ms8607_bool  pad_with_zeroes)
{
	i2c_context *ctx = caller_context;
	(void)pad_width;
	(void)pad_with_zeroes;
	chprintf(ctx->meta->stdout, "%d", (int)number); // TODO: pad_width and pad character
}




static enum ms5840_status i2c_error_to_ms5840_error(i2cflags_t  errors)
{
	if ( errors == I2CD_ACK_FAILURE )
		return ms5840_status_callback_i2c_nack;
	else
		return ms5840_status_callback_error;
}

static enum ms5840_status i2c_controller_read_ms5840(void *caller_context, ms5840_i2c_controller_packet *const packet)
{
	i2cflags_t   errors;
	i2c_context  *ctx = caller_context;

	errors = i2c_controller_mux(ctx, 'R');
	if ( errors != I2CD_NO_ERROR ) {
		// Don't return NACK, as we didn't even have
		// a chance to communicate with the ms5840.
		return ms5840_status_callback_error;
	}

	msg_t  stat = i2cMasterReceive(
		ctx->meta->i2c_driver, packet->address,
		packet->data, packet->data_length);

	if ( handle_i2c_errors_(ctx->meta->i2c_driver, stat, 'R', &errors) )
		return i2c_error_to_ms5840_error(errors);
	else
		return ms5840_status_ok;
}

static enum ms5840_status i2c_controller_write_ms5840(void *caller_context, ms5840_i2c_controller_packet *const packet)
{
	msg_t   stat;
	uint8_t rxbuf[1];
	i2cflags_t   errors;
	i2c_context  *ctx = caller_context;

	errors = i2c_controller_mux(ctx, 'T');
	if ( errors != I2CD_NO_ERROR ) {
		// Don't return NACK, as we didn't even have
		// a chance to communicate with the ms5840.
		return ms5840_status_callback_error;
	}

	stat = i2cMasterTransmit(
		ctx->meta->i2c_driver, packet->address,
		packet->data, packet->data_length, rxbuf, 0);

	if ( handle_i2c_errors(ctx->meta->i2c_driver, stat, 'T') )
		return ms5840_status_callback_error;
	else
		return ms5840_status_ok;
}

static enum ms5840_status  sleep_ms_ms5840(void *caller_context, uint32_t ms)
{
	(void)caller_context;
	chThdSleepMilliseconds(ms);
	return ms5840_status_ok;
}

static void  print_string_ms5840(void *caller_context, const char *text)
{
	i2c_context *ctx = caller_context;
	chprintf(ctx->meta->stdout, "%s", text);
}

static void  print_int64_ms5840(void *caller_context,  int64_t  number, uint8_t pad_width,  ms5840_bool  pad_with_zeroes)
{
	i2c_context *ctx = caller_context;
	(void)pad_width;
	(void)pad_with_zeroes;
	chprintf(ctx->meta->stdout, "%d", (int)number); // TODO: pad_width and pad character
}


#if 0
// For now, we aren't using the shell.
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
#endif

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

static ms8607_host_functions  host_funcs_ms8607 = {0};

void chibi_ms8607_assign_functions(ms8607_host_functions *deps, void *caller_context)
{
	(void)caller_context;
	deps->i2c_controller_read          = &i2c_controller_read_ms8607;
	deps->i2c_controller_write         = &i2c_controller_write_ms8607;
	deps->i2c_controller_write_no_stop = &i2c_controller_write_no_stop_ms8607;
	deps->sleep_ms                     = &sleep_ms_ms8607;
	deps->print_string                 = &print_string_ms8607;
	deps->print_int64                  = &print_int64_ms8607;
}

static ms5840_host_functions  host_funcs_ms5840 = {0};

void chibi_ms5840_assign_functions(ms5840_host_functions *deps, void *caller_context)
{
	(void)caller_context;
	deps->i2c_controller_read          = &i2c_controller_read_ms5840;
	deps->i2c_controller_write         = &i2c_controller_write_ms5840;
	deps->sleep_ms                     = &sleep_ms_ms5840;
	deps->print_string                 = &print_string_ms5840;
	deps->print_int64                  = &print_int64_ms5840;
}


//static WORKING_AREA(waThread1, 4096);
static WORKING_AREA(waThread1, 16384);
//static WORKING_AREA(waThread1, 65536);
static msg_t Thread1(void *p)
{
#if 0
	msg_t       stat;
	uint8_t     txbuf[1];
	uint8_t     rxbuf[1];
	uint8_t     i2c_expander_port_selection_bits = 0xFF;
#endif

	(void)p;
	chRegSetThreadName("i2c");

	I2CDriver        *i2c_driver = &I2CD1;
	i2c_driver_meta  i2c_meta;

	i2c_driver_meta_init(&i2c_meta, (BaseSequentialStream *)&SD1, i2c_driver);

	i2c_context  i2c_5840;
	i2c_context  i2c_8607;

	i2c_5840.meta = &i2c_meta;
	i2c_8607.meta = &i2c_meta;

	i2c_5840.i2c_expander_port_selection_bits = 0x02;
	i2c_8607.i2c_expander_port_selection_bits = 0x01;

	I2CConfig  i2c_config;
	i2c_config.ic_speed = 1500;
	i2cStart(i2c_meta.i2c_driver, &i2c_config);

#if 0
	chprintf(bss, "I2C.TCA9548A: (INFO)  Setting up I2C multiplexer.\n");

#if 0
	i2cflags_t  errors;
	chprintf(bss, "I2C.TCA9548A: (INFO)  Reading I2C multiplexer register.\n");
	stat = i2cMasterReceive(
		i2c_meta.i2c_driver, I2C_EXPANDER_ADDR, rxbuf, sizeof(rxbuf));

	if ( handle_i2c_errors_(i2c_meta.i2c_driver, stat, 'R', &errors) )
	{
		if ( errors == I2CD_ACK_FAILURE )
			chprintf(bss, "I2C.TCA9548A: (ERROR) Problem when communicating with I2C multiplexer: ACK FAILURE\n");
		else
			chprintf(bss, "I2C.TCA9548A: (ERROR) Problem when communicating with I2C multiplexer: (unknown)\n");
		chprintf(bss, "I2C.TCA9548A: (ERROR) Unable to continue.\n");
		return 1;
	}
	i2c_expander_port_selection_bits = rxbuf[0];
	chprintf(bss, "I2C.TCA9548A: (INFO)  Register data received:\n");
	chprintf(bss, "    0x%02X\n", i2c_expander_port_selection_bits);
#endif

	chprintf(bss, "I2C.TCA9548A: (INFO)  Writing I2C multiplexer register.\n");
	i2c_expander_port_selection_bits = 0x01;
	txbuf[0] = i2c_expander_port_selection_bits;
	stat = i2cMasterTransmit(
		i2c_meta.i2c_driver, I2C_EXPANDER_ADDR, txbuf, sizeof(txbuf), rxbuf, 0);

	if ( handle_i2c_errors(i2c_meta.i2c_driver, stat, 'T') ) {
		chprintf(bss, "I2C.TCA9548A: (ERROR) Problem when communicating with I2C multiplexer.\n");
		chprintf(bss, "I2C.TCA9548A: (ERROR) Unable to continue.\n");
		//return 1;
	}

#if 0
	chprintf(bss, "I2C.TCA9548A: (INFO)  Reading I2C multiplexer register.\n");
	stat = i2cMasterReceive(
		i2c_meta.i2c_driver, I2C_EXPANDER_ADDR, rxbuf, sizeof(rxbuf));

	if ( handle_i2c_errors_(i2c_meta.i2c_driver, stat, 'R', &errors) )
	{
		if ( errors == I2CD_ACK_FAILURE )
			chprintf(bss, "I2C.TCA9548A: (ERROR) Problem when communicating with I2C multiplexer: ACK FAILURE\n");
		else
			chprintf(bss, "I2C.TCA9548A: (ERROR) Problem when communicating with I2C multiplexer: (unknown)\n");
		chprintf(bss, "I2C.TCA9548A: (ERROR) Unable to continue.\n");
		return 1;
	}
	i2c_expander_port_selection_bits = rxbuf[0];
	chprintf(bss, "I2C.TCA9548A: (INFO)  Register data received:\n");
	chprintf(bss, "    0x%02X\n", i2c_expander_port_selection_bits);
#endif
#endif

	enum ms5840_status  sensor_status_ms5840;
	ms5840_sensor       sensor_ms5840;

	enum ms8607_status  sensor_status_ms8607;
	ms8607_sensor       sensor_ms8607;

	chprintf(bss, "I2C.MS5840: (INFO)  Initializing MS5840 host functions / integration.\n");
	sensor_status_ms5840 = ms5840_init_and_assign_host_functions(&host_funcs_ms5840, NULL, &chibi_ms5840_assign_functions);
	if ( sensor_status_ms5840 != ms5840_status_ok ) {
		chprintf(bss, "I2C.MS5840: (ERROR) In function `ms5840_init_and_assign_host_functions`:\n");
		chprintf(bss, "I2C.MS5840: (ERROR) %s\n", ms5840_stringize_error(sensor_status_ms5840));
		chprintf(bss, "I2C.MS5840: (ERROR) Unable to continue.\n");
		return 1;
	}

	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 host functions / integration.\n");
	sensor_status_ms8607 = ms8607_init_and_assign_host_functions(&host_funcs_ms8607, NULL, &chibi_ms8607_assign_functions);
	if ( sensor_status_ms8607 != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `ms8607_init_and_assign_host_functions`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status_ms8607));
		chprintf(bss, "I2C.MS8607: (ERROR) Unable to continue.\n");
		return 1;
	}

	chprintf(bss, "I2C.MS5840: (INFO)  Initializing MS5840 sensor object.\n");
	sensor_status_ms5840 = ms5840_init_sensor(&sensor_ms5840, &host_funcs_ms5840);
	if ( sensor_status_ms5840 != ms5840_status_ok ) {
		chprintf(bss, "I2C.MS5840: (ERROR) In function `ms5840_init_sensor`:\n");
		chprintf(bss, "I2C.MS5840: (ERROR) %s\n", ms5840_stringize_error(sensor_status_ms5840));
		chprintf(bss, "I2C.MS5840: (ERROR) Unable to continue.\n");
		return 1;
	}


	chprintf(bss, "I2C.MS8607: (INFO)  Initializing MS8607 sensor object.\n");
	sensor_status_ms8607 = ms8607_init_sensor(&sensor_ms8607, &host_funcs_ms8607);
	if ( sensor_status_ms8607 != ms8607_status_ok ) {
		chprintf(bss, "I2C.MS8607: (ERROR) In function `ms8607_init_sensor`:\n");
		chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status_ms8607));
		chprintf(bss, "I2C.MS8607: (ERROR) Unable to continue.\n");
		return 1;
	}
	palSetPad(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01);

	chprintf(bss, "I2C.MS5840: (INFO)  Resetting sensor.\n");
	while ( true ) {
		sensor_status_ms5840 = ms5840_reset(&sensor_ms5840, &i2c_5840);
		if ( sensor_status_ms5840 == ms5840_status_ok )
			break;
		if ( sensor_status_ms5840 != ms5840_status_callback_error )
			chprintf(bss, "I2C.MS5840: (ERROR) %s\n", ms5840_stringize_error(sensor_status_ms5840));
		chprintf(bss, "I2C.MS5840: (ERROR) Sensor reset failed.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS5840: (INFO)  Attempting another reset.\n");
	}

	chprintf(bss, "I2C.MS8607: (INFO)  Resetting sensor.\n");
	while ( true ) {
		sensor_status_ms8607 = ms8607_reset(&sensor_ms8607, &i2c_8607);
		if ( sensor_status_ms8607 == ms8607_status_ok )
			break;
		if ( sensor_status_ms8607 != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status_ms8607));
		chprintf(bss, "I2C.MS8607: (ERROR) Sensor reset failed.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Attempting another reset.\n");
	}
	palSetPad(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02);

	chprintf(bss, "I2C.MS8607: (INFO)  Disabling heater.\n");
	while ( true ) {
		sensor_status_ms8607 = ms8607_disable_heater(&sensor_ms8607, &i2c_8607);
		if ( sensor_status_ms8607 == ms8607_status_ok )
			break;
		if ( sensor_status_ms8607 != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status_ms8607));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to disable heater.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying heater disable.\n");
	}
	palSetPad(PROGRESS_LED_PORT_03, PROGRESS_LED_PAD_03);

	chprintf(bss, "I2C.MS8607: (INFO)  Setting humidity resolution to 10b.\n");
	while ( true )
	{
		//sensor_status_ms8607 = ms8607_set_humidity_resolution(i2c_driver, ms8607_humidity_resolution_10b);
		sensor_status_ms8607 = ms8607_set_humidity_resolution(&sensor_ms8607, ms8607_humidity_resolution_12b, &i2c_8607);
		if ( sensor_status_ms8607 == ms8607_status_ok )
			break;
		if ( sensor_status_ms8607 != ms8607_status_callback_error )
			chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status_ms8607));
		chprintf(bss, "I2C.MS8607: (ERROR) Failed to set humidity resolution.\n");
		chThdSleepMilliseconds(1000);
		chprintf(bss, "I2C.MS8607: (INFO)  Retrying setting of humidity resolution to 10b.\n");
	}
	palSetPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);

	chprintf(bss, "I2C.MS5840: (INFO)  Setting pressure resolution to 2048.\n");
	ms5840_set_pressure_resolution(&sensor_ms5840, ms5840_pressure_resolution_osr_2048, &i2c_5840);
	chprintf(bss, "I2C.MS8607: (INFO)  Setting pressure resolution to 2048.\n");
	ms8607_set_pressure_resolution(&sensor_ms8607, ms8607_pressure_resolution_osr_2048, &i2c_8607);
	palSetPad(PROGRESS_LED_PORT_05, PROGRESS_LED_PAD_05);

	chprintf(bss, "I2C.MS8607: (INFO)  Setting controller mode to NO HOLD.\n");
	ms8607_set_humidity_i2c_controller_mode(&sensor_ms8607, ms8607_i2c_no_hold, &i2c_8607);
	palSetPad(PROGRESS_LED_PORT_06, PROGRESS_LED_PAD_06);

	while (TRUE) {
		int32_t temperature = 0; // degC
		int32_t pressure    = 0; // mbar
		int32_t humidity    = 0; // %RH

		palClearPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
		palClearPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
		palSetPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);

		chprintf(bss, "\n");
		chprintf(bss, "I2C.MS5840: (INFO)  Retrieving TP (temperature-pressure) readings.\n");
		sensor_status_ms5840 = ms5840_read_temperature_pressure_int32(
				&sensor_ms5840, &temperature, &pressure, &i2c_5840);
		if ( sensor_status_ms5840 != ms5840_status_ok )
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
			if ( sensor_status_ms5840 != ms5840_status_callback_error )
				chprintf(bss, "I2C.MS5840: (ERROR) %s\n", ms5840_stringize_error(sensor_status_ms5840));
			chprintf(bss, "I2C.MS5840: (ERROR) Failed to read TP data.\n");
		}
		else
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
			chprintf(bss, "I2C.MS5840: (INFO)  TP data received:\n");
			chprintf(bss, "    Temperature = %d.%d%d%d degC\n", (int)(temperature/1000), (int)((temperature/100)%10), (int)((temperature/10)%10), (int)(temperature%10) );
			chprintf(bss, "    Pressure    = %d.%d%d%d mbar\n", (int)(pressure/1000),    (int)((pressure/100)%10),    (int)((pressure/10)%10),    (int)(pressure%10) );
		}

		temperature = 0; // degC
		pressure    = 0; // mbar
		humidity    = 0; // %RH

		chprintf(bss, "\n");
		chprintf(bss, "I2C.MS8607: (INFO)  Retrieving TPH (temperature-pressure-humidity) readings.\n");
		sensor_status_ms8607 = ms8607_read_temperature_pressure_humidity_int32(
				&sensor_ms8607, &temperature, &pressure, &humidity, &i2c_8607);
		if ( sensor_status_ms8607 != ms8607_status_ok )
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
			if ( sensor_status_ms8607 != ms8607_status_callback_error )
				chprintf(bss, "I2C.MS8607: (ERROR) %s\n", ms8607_stringize_error(sensor_status_ms8607));
			chprintf(bss, "I2C.MS8607: (ERROR) Failed to read TPH data.\n");
		}
		else
		{
			palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
			palSetPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
			chprintf(bss, "I2C.MS8607: (INFO)  TPH data received:\n");
			chprintf(bss, "    Temperature = %d.%d%d%d degC\n", (int)(temperature/1000), (int)((temperature/100)%10), (int)((temperature/10)%10), (int)(temperature%10) );
			chprintf(bss, "    Pressure    = %d.%d%d%d mbar\n", (int)(pressure/1000),    (int)((pressure/100)%10),    (int)((pressure/10)%10),    (int)(pressure%10) );
			chprintf(bss, "    Humidity    = %d.%d%d%d %%RH\n", (int)(humidity/1000),    (int)((humidity/100)%10),    (int)((humidity/10)%10),    (int)(humidity%10) );
		}
		chThdSleepMilliseconds(1000);
	}

	// poor i2cStop statement can never execute.
	i2cStop(i2c_meta.i2c_driver);
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

	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

	// Events servicing loop.
	chThdWait(chThdSelf());

	return 0;
}
