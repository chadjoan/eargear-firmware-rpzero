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

#include <stdint.h>

#include "ms8607.h"
#include "ms5840.h"

//#define TRACE_I2C_TRANSMIT (1)
//#define TRACE_I2C_RECEIVE  (1)

// TODO: Add safety that stops operating the pump if we don't hear from
//   the chamber's pressure sensor for about a second. (It should be a period
//   of time short enough that the pressure can't rise to uncomfortable or
//   painful levels within that period of time. So this does depend somewhat
//   on the size of the system being pressurized, and the flow rate of the pump.
//   In this case, we can already tell that it takes at least a short while
//   (a few seconds) to reach mildly high pressures (40-60 mbar or cmH2O),
//   so we don't want to wait very long, but also we don't have to react
//   immediately to small temporary hiccups in the sensors or I2C comms.)
// TODO: Alarm buzzer when problems!
//
// TODO: Display layout thoughts:
//
//  "PRESSURE (MBAR~CMH2O)        ZERO:  988.0"
//  "CPAP:  988.0 | PUMP: 1001.2 | RET: 1000.5" <- 168 pixels w/ minimal font, ugh
//  "        +0.0 |        +13.2 |       +12.5"    (Too big for 128x64 display.)
//
//  "PRESSURE (MBAR~CMH2O)"
//  "ZERO:  988.01"
//  "CPAP:  +11.07"
//  "PUMP:  +13.23"
//  " RET:  +12.57"


// These calibration numbers must be determined by running a version of this
// software compiled in "calibration mode" (TODO: macro name, instructions),
// which will yield the numbers that these macros must expand to.
//
// These numbers are specific to each physical sensor used in the
// device. If you change a sensor, you must recalibrate.
//
// The calibration must be done with all sensors experiencing identical
// physical conditions (same pressure, temperature, humidity, etc).
// The sure-fire way to do this is to simply place them next to each other
// on the same bench/table/whatever, with all of them exposed to the same
// room air.
//
#define CALIBRATION_FOR_CPAP    (3129)
#define CALIBRATION_FOR_PUMP    (-273)
#define CALIBRATION_FOR_RETURN (-2854)

#define CALIBRATION_PREFIX  ("CALIBRATION_FOR_")

static uint32_t pump_drive_resonant_frequency = 23000;
static PWMConfig pwm_config;

#ifndef EARGEAR_SENSOR_CALIBRATION
	#ifndef EARGEAR_PUMP_OPERATION
		#define EARGEAR_PUMP_OPERATION (1)
	#endif
#endif

// Pin allocations:
// - Pins 2 & 3 are allocated for I2C.
// - Pins 14 & 15 are allocated for UART. (see also: sdcard-boilerplate/config.txt)
// - Pin 18 is used for PWM. (currently hardcoded in os/hal/platforms/BCM2835/pwm_lld.(c|h))
// - Pins 22-27 might be nice to avoid, because they might be used for JTAG
//     debugging at some point. Right now (at least, 2022-01-02) it doesn't work
//     though, so it's not a big deal.

//#define PROGRESS_LED_PAD_01  GPIO18_PAD
#define PROGRESS_LED_PAD_02  GPIO17_PAD
#define PROGRESS_LED_PAD_03  GPIO10_PAD
#define PROGRESS_LED_PAD_04  GPIO9_PAD
#define PROGRESS_LED_PAD_05  GPIO11_PAD
#define PROGRESS_LED_PAD_06  GPIO5_PAD
#define PROGRESS_LED_PAD_07  GPIO6_PAD
#define PROGRESS_LED_PAD_08  GPIO13_PAD
#define PROGRESS_LED_PAD_09  GPIO19_PAD
//#define PROGRESS_LED_PAD_10  GPIO19_PAD

//#define PROGRESS_LED_PORT_01  GPIO18_PORT
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

//static BaseSequentialStream *bss;
BaseSequentialStream *bss;

#ifdef EARGEAR_SENSOR_CALIBRATION
	#define MAX_NUM_READINGS_PER_SENSOR (1024)
#else
	// This can be made larger if needed.
	// Right now it's not needed, and it'll help me flush out boundary-handling-bugs.
	#define MAX_NUM_READINGS_PER_SENSOR (16)
#endif

static void print_thousandths(BaseSequentialStream *bss, int32_t number)
{
	int32_t abs = number;
	const char *sign = "";
	if ( number < 0 ) {
		abs = -number;
		sign = "-";
	}
	chprintf(bss, "%s%d.%d%d%d", sign, (int)(abs/1000), (int)((abs/100)%10), (int)((abs/10)%10), (int)(abs%10) );
}

static void print_uint8_as_hex(BaseSequentialStream  *bss,  uint8_t  the_byte)
{
	char buffer[20];
	uint8_t buflen = sizeof(buffer);
	char *hexstr = tepht_uint64_to_hex(the_byte, NULL, buffer, buflen, 2, '0');
	chprintf(bss, "%s", hexstr);
}

static void print_uint8_as_binary(BaseSequentialStream  *bss,  uint8_t  the_byte)
{
	char buffer[80];
	uint8_t buflen = sizeof(buffer);
	char *binstr = tepht_uint64_to_binary(the_byte, NULL, buffer, buflen, 8, '0');
	chprintf(bss, "%s", binstr);
}

#if 0
/// Returns the result of `(a - b)`,
static systime_t subtract_wrap_saturate(systime_t a, systime_b)
{
}

#define subtract_wrap_saturate(a,b) ( \
	(sizeof(a) == 8 || sizeof(b) == 8) ? subtract_wrap_saturate_64((uint64_t)(a), (uint64_t)(b)) : \
	(sizeof(a) == 4 || sizeof(b) == 4) ? subtract_wrap_saturate_32((uint32_t)(a), (uint32_t)(b)) : \
	(sizeof(a) == 2 || sizeof(b) == 2) ? subtract_wrap_saturate_16((uint16_t)(a), (uint16_t)(b)) : \
	subtract_wrap_saturate_8((uint8_t)(a), (uint8_t)(b)) )
#endif

static uint64_t ticks2microsecs(systime_t ticks)
{
	// 100 seconds worth of microseconds, in system ticks.
	static int64_t hundred_microsecs_as_ticks = 0;
	if ( hundred_microsecs_as_ticks == 0 )
		hundred_microsecs_as_ticks = US2ST(100L);

	int64_t tmp_ticks = ticks;
	int64_t microsecs = (tmp_ticks*100LL)/hundred_microsecs_as_ticks;
	return (uint64_t)microsecs;
}

// TODO: how much wrap-around threat?
static systime_t  system_time_at_zero = 0;

static void calculate_system_time_reference(void)
{
	system_time_at_zero = chTimeNow();
}

static uint64_t calculate_system_time_in_usecs(void)
{
	systime_t      time_now;
	uint64_t       usecs;

	time_now = chTimeNow();
	usecs = ticks2microsecs(time_now - system_time_at_zero);

	return usecs;
}


// Measurements are in thousandths of millibars.
#if 0
static int32_t ms8607_pressure_data[MAX_NUM_READINGS_PER_SENSOR];
static int32_t ms5840_pressure_data[MAX_NUM_READINGS_PER_SENSOR];
static size_t  ms8607_pressure_data_latest_index = MAX_NUM_READINGS_PER_SENSOR;
static size_t  ms5840_pressure_data_latest_index = MAX_NUM_READINGS_PER_SENSOR;
#endif


// The maximum is the amount of sensors that could ever be plugged into this
// thing. We might actually plug in fewer (maybe an optional sensor failed,
// or we are testing something), but it's nice to know how many COULD be plugged.
//
// Use this number when allocating memory for sensor-related structures (usually
// the length of arrays of structs or variables), and also use this when
// initializing said memory. Do not use it for loops that attempt to actually
// communicate with the sensor.
#define  PT_SENSOR_COUNT_MAX  (3)

// The amount of sensors that are /actually/ plugged in to the device. This
// might be less than the maximum if we have a failed optional sensor or
// we're testing something.
//
// Use this variable when iterating over sensors for anything that doesn't
// involve memory allocation or memory initialization. Definitely use this
// to terminate any loops where communication (I2C) is involved.
#define  PT_SENSOR_COUNT_USED (3)

#define  PT_SENSOR_CPAP_IDX    (0)
#define  PT_SENSOR_PUMP_IDX    (1)
#define  PT_SENSOR_RETURN_IDX  (2)

static const char *sensor_id_to_name(size_t sensor_id)
{
	switch ( sensor_id )
	{
		case PT_SENSOR_CPAP_IDX:   return "CPAP";
		case PT_SENSOR_PUMP_IDX:   return "Pump";
		case PT_SENSOR_RETURN_IDX: return "Return";
	}
	return "<Not Implemented>";
}

#ifndef EARGEAR_SENSOR_CALIBRATION
static int32_t sensor_id_to_calibration_value(size_t sensor_id)
{
	switch ( sensor_id )
	{
		case PT_SENSOR_CPAP_IDX:   return CALIBRATION_FOR_CPAP;
		case PT_SENSOR_PUMP_IDX:   return CALIBRATION_FOR_PUMP;
		case PT_SENSOR_RETURN_IDX: return CALIBRATION_FOR_RETURN;
	}
	return 0;
}
#endif




typedef struct circular_buffer
{
	size_t   latest_index;
	size_t   num_values;
	int32_t  values[MAX_NUM_READINGS_PER_SENSOR];
}
circular_buffer;

static circular_buffer  pressure_data[PT_SENSOR_COUNT_MAX];

static void init_sensor_data_buffers(void)
{
	// This loop will probably work if it terminates at PT_SENSOR_COUNT_USED,
	// but just to be safe, we're using PT_SENSOR_COUNT_MAX instead to ensure
	// that ALL of the sensor-related memory is initialized. This would make
	// things fail in a more predictable way if there is ever any mistake
	// (this makes it so that any access to the memory allocated for unused
	// sensors would then reference initialized memory (well-defined C99 behavior)
	// instead of uninitialized memory (undefined behavior)).
	size_t i;
	for ( i = 0; i < PT_SENSOR_COUNT_MAX; i++ )
	{
		pressure_data[i].latest_index = MAX_NUM_READINGS_PER_SENSOR;
		pressure_data[i].num_values = 0;
	}
}

static void insert_sensor_value(circular_buffer *data, int32_t value)
{
	size_t i = data->latest_index;

	// This avoids using the modulus operator, because we are populating the
	// circular buffer in order from highest index to lowest, and size_t
	// is not a signed type, nor do we want it to be. This avoids doing
	// modulus on a negative value, and it avoids wrap-around at 0,
	// which may-or-may-not align well with the chosen MAX_NUM_READINGS_PER_SENSOR
	// (if we counted up, modulus could probably work if the buffer was
	// always a power-of-2 size, but I don't want to assume that).
	// So this is a safe/paranoid way to do things:
	// just manually keep values in-range at all times.
	//
	// This choice also allows modulus to be safely used for reading from
	// the circular buffer (assuming a small/finite number of laps); in other
	// words it is slightly more complicated to insert into the buffer than
	// it is to read from the buffer, which is a good trade-off, really.
	if ( i == 0 )
		i = MAX_NUM_READINGS_PER_SENSOR - 1;
	else
		i--;

	data->values[i] = value;
	data->latest_index = i;
	if ( data->num_values < MAX_NUM_READINGS_PER_SENSOR )
		data->num_values++;
}


typedef struct measurement
{
	const char *sensor_name;
	int32_t    uncalibrated_value;
	int32_t    value;
}
measurement;

static void calculate_measurement_from_reading(measurement *m, int32_t reading, size_t which_sensor)
{
	m->sensor_name = sensor_id_to_name(which_sensor);
	m->uncalibrated_value = reading;

#ifndef EARGEAR_SENSOR_CALIBRATION
	m->value = reading + sensor_id_to_calibration_value(which_sensor);
#else
	m->value = reading;
#endif
}

static int64_t sum_sensor_data_range(BaseSequentialStream *stdout, circular_buffer  *data,  size_t start_at, size_t n)
{
	size_t   buffer_size = MAX_NUM_READINGS_PER_SENSOR;

	// TODO: I'd like to use a proper formatter for size_t, like %zu, but
	// I don't think the 2012 version of ChibiOS supports this?
	if ( n > data->num_values && n <= buffer_size ) {
		chprintf(stdout,
			"ERROR: overflow in `sum_sensor_data_range`: value of `to`, %d, is larger than number of elements, %d.\n",
			(int)n, (int)data->num_values);
		n = data->num_values;
	}
	else
	if ( n > buffer_size ) {
		chprintf(stdout,
			"ERROR: overflow in `sum_sensor_data_range`: value of `to`, %d, is larger than buffer size, %d.\n",
			(int)n, (int)buffer_size);
		n = buffer_size;
	}

	size_t   latest = data->latest_index;
	size_t   i;
	int64_t  sum = 0;
	size_t   end_at = start_at + n;
	for ( i = start_at; i < end_at; i++ ) {
		int64_t value = data->values[(latest+i) % buffer_size];
		sum += value;
	}
	return sum;
}

// ifdef is used to prevent "`sum_sensor_data_n` not used" warning message.
#ifdef EARGEAR_PUMP_OPERATION
static int64_t sum_sensor_data_n(BaseSequentialStream *stdout, circular_buffer  *data,  size_t n)
{
	return sum_sensor_data_range(stdout, data, 0, n);
}
#endif

// ifdef is used to prevent "`sum_sensor_data` not used" warning message.
#ifdef EARGEAR_SENSOR_CALIBRATION
static int64_t sum_sensor_data(BaseSequentialStream *stdout, circular_buffer  *data)
{
	return sum_sensor_data_range(stdout, data, 0, data->num_values);
}
#endif

#define intN_minimum(a,b)  (((a) > (b)) ? (b) : (a))

#ifdef EARGEAR_SENSOR_CALIBRATION
static void on_pressure_measurement(BaseSequentialStream *stdout, const measurement *m, size_t sensor_id)
{
	int64_t  counts[PT_SENSOR_COUNT_MAX];
	int32_t  avgs[PT_SENSOR_COUNT_MAX];
	int64_t  sum;
	int64_t  all_sensor_sum;
	int64_t  all_sensor_count;
	int32_t  all_sensor_avg;
	int32_t  offset;
	size_t i;

	(void)sensor_id;
	chprintf(stdout, "\n");

	chprintf(stdout, "New pressure reading from %s: ", m->sensor_name);
	print_thousandths(stdout, m->uncalibrated_value);
	chprintf(stdout,      " mbar (uncalibrated)\n\n");

	all_sensor_sum = 0;
	all_sensor_count = 0;

	for ( i = 0; i < PT_SENSOR_COUNT_USED; i++ )
	{
		sum       = sum_sensor_data(stdout, &pressure_data[i]);
		counts[i] = pressure_data[i].num_values;
		avgs[i]   = (int32_t)(sum / counts[i]);

		all_sensor_sum   += sum;
		all_sensor_count += counts[i];
	}

	all_sensor_avg = (int32_t)(all_sensor_sum / all_sensor_count);

	for ( i = 0; i < PT_SENSOR_COUNT_USED; i++ )
	{
		offset = all_sensor_avg - avgs[i];
		chprintf(stdout,
			"Calibration value for %s%s is (%d), taken from %d samples\n",
			CALIBRATION_PREFIX, sensor_id_to_name(i), (int)offset, (int)counts[i]);
	}
}
#endif

#ifdef EARGEAR_PUMP_OPERATION
static uint64_t last_time = 0;
static uint8_t  time_started = 0;
static uint8_t  pump_on = 0;
static void on_pressure_measurement(BaseSequentialStream *stdout, const measurement *m, size_t sensor_id)
{
	(void)sensor_id;

	int32_t latest_pressures[PT_SENSOR_COUNT_MAX];
	int32_t filtered_pressures[PT_SENSOR_COUNT_MAX];

	size_t  i;
	size_t  sum_of_counts = 0;
	size_t  lowest_count  = SIZE_MAX;
	for ( i = 0; i < PT_SENSOR_COUNT_USED; i++ )
	{
		size_t  this_count = pressure_data[i].num_values;
		sum_of_counts += this_count;
		lowest_count  = intN_minimum(lowest_count, this_count);
	}

	if ( lowest_count == 0 )
	{
		if ( sum_of_counts == 0 )
			chprintf(stdout, "ERROR: `on_pressure_measurement` called when there are no pressure measurements.\n");
		else
			chprintf(stdout, "on_pressure_measurement: Measurement received, but not all sensors have reported yet. Still warming up.\n");
		return;
	}

	for ( i = 0; i < PT_SENSOR_COUNT_MAX; i++ )
	{
		circular_buffer  *data_set = &pressure_data[i];

		latest_pressures[i] = data_set->values[data_set->latest_index];

		if ( lowest_count < 4 ) // and > 0
		{
			// We'll just use the latest reading for the first 4 readings.
			// This would give us noisy data, but we only have to live with it
			// for a miniscule amount of time (just 4 measurements).
			filtered_pressures[i] = latest_pressures[i];
		}
		else
		if ( lowest_count < 8 ) // and >= 4
		{
			// We have at least 4 measurements.
			// They are all pretty recent, so we just average them.
			size_t   n = 4;
			int64_t  sum = sum_sensor_data_n(stdout, data_set, n);
			filtered_pressures[i] = sum / n;
		}
		else
		// if ( lowest_count >= 8 )
		{
			// Now that we have more data, we include some older measurements,
			// but give them lower weight.
			// This will further helps smooth things out, but still allow the
			// value to respond rapidly if things change quickly.
			// (This is like an exponentially-weighted moving average, but much
			// simpler and more approximate.)
			size_t   n = 4;
			int64_t  sum_0 = sum_sensor_data_range(stdout, data_set, n*0, n);
			int64_t  sum_1 = sum_sensor_data_range(stdout, data_set, n*1, n);
			filtered_pressures[i] = (sum_0*3 + sum_1*1)/(n*(3+1));
		}
	}

	chprintf(stdout, "\n");

	uint64_t time_now = calculate_system_time_in_usecs();
	if ( time_started == 0 ) {
		last_time = time_now;
		time_started = 1;
	}

	chprintf(stdout, "Pressure reading from %s: ", m->sensor_name);
	print_thousandths(stdout, m->value);
	chprintf(stdout,     " mbar   (");
	print_thousandths(stdout, m->uncalibrated_value);
	chprintf(stdout,     " mbar, uncalibrated)");
	chprintf(stdout,     " at time %d usecs", time_now);
	chprintf(stdout,     "\n");

	chprintf(stdout, "Effective target (CPAP) pressure:  ");
	print_thousandths(stdout, filtered_pressures[PT_SENSOR_CPAP_IDX]);
	chprintf(stdout,     " mbar   (");
	print_thousandths(stdout, latest_pressures[PT_SENSOR_CPAP_IDX]);
	chprintf(stdout,     " mbar, latest, calibrated)\n");

	chprintf(stdout, "Effective chamber (ear) pressure:  ");
	print_thousandths(stdout, filtered_pressures[PT_SENSOR_PUMP_IDX]);
	chprintf(stdout,     " mbar   (");
	print_thousandths(stdout, latest_pressures[PT_SENSOR_PUMP_IDX]);
	chprintf(stdout,     " mbar, latest, calibrated)\n");

	chprintf(stdout, "Effective return (ear) pressure:   ");
	print_thousandths(stdout, filtered_pressures[PT_SENSOR_RETURN_IDX]);
	chprintf(stdout,     " mbar   (");
	print_thousandths(stdout, latest_pressures[PT_SENSOR_RETURN_IDX]);
	chprintf(stdout,     " mbar, latest, calibrated)\n");

	int32_t pressure_drop = filtered_pressures[PT_SENSOR_PUMP_IDX] - filtered_pressures[PT_SENSOR_RETURN_IDX];
	chprintf(stdout, "Pressure drop in tubing:           ");
	print_thousandths(stdout, pressure_drop);
	chprintf(stdout,     " mbar\n");

	// Simple calculation, for now:
	// Ear pressure is the average of the outgoing and returning pressures.
	// This is based on the assumption that the ear is approximately halfway
	// along the length of tubing that runs from the pump unit to the return
	// unit (ergo, the pressure the ear experiences should be halfway between
	// these two pressures).
	// TODO: This assumption could be violated in the case of sensor failure or
	// missing sensor. We should probably make the code robust in such situations.
	int32_t ear_pressure =
		(filtered_pressures[PT_SENSOR_PUMP_IDX] + filtered_pressures[PT_SENSOR_RETURN_IDX]) / 2;

	int32_t pressure_diff = ear_pressure - filtered_pressures[PT_SENSOR_CPAP_IDX];
	chprintf(stdout, "Pressure difference-to-target:     ");
	print_thousandths(stdout, pressure_diff);
	chprintf(stdout,     " mbar\n");

	uint64_t time_diff = time_now - last_time;
	chprintf(stdout, "Time difference (ms):              ");
	print_thousandths(stdout, (uint32_t)time_diff);
	chprintf(stdout,     "\n");

#if 0
	// Output that can be used to find the best drive-resonant-frequency.
	//chprintf(stdout, "pwm_config->period   == %d\r\n", pwm_config.period);
	chprintf(stdout, "pwm target frequency == %d\r\n", pump_drive_resonant_frequency);
	chprintf(stdout, "pwm result frequency == %d\r\n", pwm_config.frequency / pwm_config.period);
#endif

#if 0
	// TODO: Figure out what this should be.
	// (Smoothing the sensor inputs could allow us to reduce this number somewhat,
	// simply because repeated measurements tend to converge on expectation values...)
	int32_t margin_for_error_ubar = 300; // microbar (thousandths of mbar)

	if ( pressure_diff > margin_for_error_ubar )
		pump_on = 1;
	else
		pump_on = 0;
#endif

	// TODO: This is just for testing.
	// (The "CPAP" sensor will be at atmospheric pressure, and we want to see
	// if the pump can be drive a small chamber up to 20.000 mbar.)

	if ( pressure_diff < 400000 )
		pump_on = 1;
	else
		pump_on = 0;

	chprintf(stdout, "Pump set to ON?                    ");
	if ( pump_on )
		chprintf(stdout, "YES\n");
	else
		chprintf(stdout, "NO\n");
}
#endif


static void handle_pressure_reading(BaseSequentialStream *stdout, int32_t reading, size_t which_sensor)
{
	measurement m;
	calculate_measurement_from_reading(&m, reading, which_sensor);
	insert_sensor_value(&pressure_data[which_sensor], m.value);
	on_pressure_measurement(stdout, &m, which_sensor);
}

#define I2C_EXPANDER_ADDR ((i2caddr_t)0x70)

static uint8_t  handle_i2c_errors(I2CDriver *driver,  msg_t  stat,  char T_or_R);
static uint8_t  handle_i2c_errors_(I2CDriver *driver,  msg_t  stat,  char T_or_R, i2cflags_t *errors);

typedef struct i2c_driver_meta
{
	BaseSequentialStream  *stdout;
	I2CDriver             *i2c_driver;
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
}

#if 0
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
#endif

static msg_t  i2c_controller_transmit(
	BaseSequentialStream  *stdout,
	I2CDriver      *i2c_driver,
	i2caddr_t      addr,
	const uint8_t  *tx_buffer, size_t tx_buffer_size,
	uint8_t        *rx_buffer, size_t rx_buffer_size)
{
#ifdef TRACE_I2C_TRANSMIT
	size_t i;
	if ( 0 < rx_buffer_size ) {
		chprintf(stdout, "I2C Transmit to address 0x\n");
		print_uint8_as_hex(stdout, addr);
		chprintf(stdout, ":\n");
		chprintf(stdout, "    Bytes to transmit, in binary:");
	} else {
		chprintf(stdout, "I2C Transmit to address 0x");
		print_uint8_as_hex(stdout, addr);
		chprintf(stdout, ", in binary:\n    ");
	}

	for ( i = 0; i < tx_buffer_size; i++ ) {
		chprintf(stdout, " ");
		print_uint8_as_binary(stdout, tx_buffer[i]);
	}

	chprintf(stdout, "\n");
#else
	(void)stdout;
#endif

	msg_t stat = i2cMasterTransmit(
		i2c_driver, addr,
		tx_buffer, tx_buffer_size, rx_buffer, rx_buffer_size);

#ifdef TRACE_I2C_TRANSMIT
	if ( stat == RDY_OK && 0 < rx_buffer_size ) {
		chprintf(stdout, "    Received bytes, in binary:   ", addr);
		for ( i = 0; i < rx_buffer_size; i++ ) {
			chprintf(stdout, " ");
			print_uint8_as_binary(stdout, rx_buffer[i]);
		}
		chprintf(stdout, "\n");
	} // Else, let caller report the error.
#endif
	return stat;
}

static msg_t  i2c_controller_receive(
	BaseSequentialStream  *stdout,
	I2CDriver      *i2c_driver,
	i2caddr_t      addr,
	uint8_t        *rx_buffer, size_t rx_buffer_size)
{
	msg_t stat = i2cMasterReceive(
		i2c_driver, addr, rx_buffer, rx_buffer_size);

#ifdef TRACE_I2C_RECEIVE
	if ( stat == RDY_OK ) {
		chprintf(stdout, "I2C Receive from address 0x", addr);
		print_uint8_as_hex(stdout, addr);
		chprintf(stdout, ", in binary:\n    ");
		size_t i;
		for ( i = 0; i < rx_buffer_size; i++ ) {
			chprintf(stdout, " ");
			print_uint8_as_binary(stdout, rx_buffer[i]);
		}
		chprintf(stdout, "\n");
	} // Else, let caller report the error.
#else
	(void)stdout;
#endif
	return stat;
}

static i2cflags_t i2c_controller_mux(i2c_context *target, char T_or_R)
{
	i2c_driver_meta  *meta = target->meta;

	i2cflags_t  errors;
	msg_t       stat;
	uint8_t     txbuf[1];
	uint8_t     rxbuf[1];

	txbuf[0] = target->i2c_expander_port_selection_bits;
	stat = i2c_controller_transmit(
		meta->stdout, meta->i2c_driver, I2C_EXPANDER_ADDR, txbuf, sizeof(txbuf), rxbuf, 0);

	if ( handle_i2c_errors_(meta->i2c_driver, stat, T_or_R, &errors) ) {
		chprintf(meta->stdout, "I2C.TCA9548A: (ERROR) Problem when communicating with I2C multiplexer.\n");
		chprintf(meta->stdout, "I2C.TCA9548A: (ERROR) Unable to continue.\n");
		return errors;
	}

	return I2CD_NO_ERROR;
}

//void i2c_master_init(void);
static tepht_status_u16  i2c_error_to_tepht_error(i2cflags_t  errors)
{
	if ( errors == I2CD_ACK_FAILURE )
		return tepht_status_callback_i2c_nack;
	else
		return tepht_status_callback_error;
}

static tepht_status_u16  i2c_controller_read_for_tepht(void *caller_context, tepht_i2c_controller_packet *const packet)
{
	i2cflags_t   errors;
	i2c_context  *ctx = caller_context;

	errors = i2c_controller_mux(ctx, 'R');
	if ( errors != I2CD_NO_ERROR ) {
		// Don't return NACK, as we didn't even have
		// a chance to communicate with the sensor.
		return tepht_status_callback_error;
	}

	msg_t  stat = i2c_controller_receive(
		ctx->meta->stdout, ctx->meta->i2c_driver, packet->address,
		packet->data, packet->data_length);

	if ( handle_i2c_errors_(ctx->meta->i2c_driver, stat, 'R', &errors) )
		return i2c_error_to_tepht_error(errors);
	else
		return tepht_status_ok;
}

static tepht_status_u16  i2c_controller_write_for_tepht(void *caller_context, tepht_i2c_controller_packet *const packet)
{
	msg_t   stat;
	uint8_t rxbuf[1];
	i2cflags_t   errors;
	i2c_context  *ctx = caller_context;

	errors = i2c_controller_mux(ctx, 'T');
	if ( errors != I2CD_NO_ERROR ) {
		// Don't return NACK, as we didn't even have
		// a chance to communicate with the sensor.
		return tepht_status_callback_error;
	}

	stat = i2c_controller_transmit(
		ctx->meta->stdout, ctx->meta->i2c_driver, packet->address,
		packet->data, packet->data_length, rxbuf, 0);

	if ( handle_i2c_errors(ctx->meta->i2c_driver, stat, 'T') )
		return tepht_status_callback_error;
	else
		return tepht_status_ok;
}

static tepht_status_u16  i2c_controller_write_no_stop_for_tepht(void *caller_context, tepht_i2c_controller_packet *const packet)
{
	i2c_context *ctx = caller_context;
	(void)packet;
	chprintf(ctx->meta->stdout,
		"ERROR: Attempt to call unimplemented function `i2c_master_write_packet_wait_no_stop`.\n");
	return tepht_status_callback_error;
}

static tepht_status_u16  sleep_ms_for_tepht(void *caller_context, uint32_t ms)
{
	(void)caller_context;
	chThdSleepMilliseconds(ms);
	return tepht_status_ok;
}

static void  print_string_for_tepht(void *caller_context, const char *text)
{
	i2c_context *ctx = caller_context;
	chprintf(ctx->meta->stdout, "%s", text);
}

static void  print_int64_for_tepht(void *caller_context,  int64_t  number, uint8_t pad_width,  tepht_bool  pad_with_zeroes)
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


static uint8_t  handle_i2c_errors_(I2CDriver *driver,  msg_t  stat,  char T_or_R, i2cflags_t *i2c_errors)
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

static uint8_t  handle_i2c_errors(I2CDriver *driver,  msg_t  stat,  char T_or_R)
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

static tepht_host_functions  host_funcs_for_tepht = {0};

static void chibi_tepht_assign_functions(tepht_host_functions *deps, void *caller_context)
{
	(void)caller_context;
	deps->i2c_controller_read          = &i2c_controller_read_for_tepht;
	deps->i2c_controller_write         = &i2c_controller_write_for_tepht;
	deps->i2c_controller_write_no_stop = &i2c_controller_write_no_stop_for_tepht;
	deps->sleep_ms                     = &sleep_ms_for_tepht;
	deps->print_string                 = &print_string_for_tepht;
	deps->print_int64                  = &print_int64_for_tepht;
}

// TODO: ugh, we gotta wrap lines at 110 characters! (chprintf doesn't seem to handle them any longer than that; some sort of output buffering issue?)

#if 0
typedef struct diagnostic_printer {
	BaseSequentialStream  *stdout;
	const char            *prefix;
	size_t                num_line_chars_printed;
	char                  last_char_printed;
} diagnostic_printer;
#endif

static void chibi_print_tepht_string(void *printer_context, const char* str, size_t len)
{
	BaseSequentialStream *stdout = printer_context;
	chSequentialStreamWrite(stdout, (const uint8_t*)str, len);
	//chprintf(stdout, "%.*s", (int)len, str);
}

static void chibi_print_tepht_error_line(BaseSequentialStream *stdout, const char *prefix, tepht_error_info  einfo)
{
	chprintf(stdout, "%s", prefix);
	tepht_print_error(einfo, &chibi_print_tepht_string, stdout);
	chprintf(stdout, "\n");
}


typedef enum  pt_sensor_type {
	pt_sensor_type_ms8607 = 0,
	pt_sensor_type_ms5840
} pt_sensor_type;

typedef struct pt_sensor_instance {
	pt_sensor_type      sensor_type;
	union {
		ms8607_sensor   as_ms8607;
		ms5840_sensor   as_ms5840;
	} sensor;
} pt_sensor_instance;


static uint8_t  probe_for_ms8607_by_doing_init_and_reset(
	BaseSequentialStream  *stdout,
	pt_sensor_instance    *sensor_instance,
	tepht_pt_sensor       *sensor_ref,
	i2c_context           *i2c_context
	)
{
	tepht_error_info  einfo;
	ms8607_sensor     *sensor_ms8607 = &sensor_instance->sensor.as_ms8607;

	chprintf(stdout, "I2C.MS8607: (INFO)  Initializing MS8607 sensor object.\n");
	einfo = ms8607_init_sensor(sensor_ms8607, &host_funcs_for_tepht);
	if ( tepht_is_error(einfo) ) {
		chprintf(stdout, "I2C.MS8607: (ERROR) In function `ms8607_init_sensor`:\n");
		chibi_print_tepht_error_line(
			stdout,      "I2C.MS8607: (ERROR) ", einfo);
		chprintf(stdout, "I2C.MS8607: (ERROR) This step didn't involve I/O; it is an internal error.\n");
		chprintf(stdout, "I2C.MS8607: (ERROR) This will be retried as a different sensor type. It might work.\n");
		return 1;
	}

	einfo = ms8607_reset(sensor_ms8607, i2c_context);
	if ( tepht_is_error(einfo) ) {
		if ( tepht_error_get_status(einfo) == tepht_status_callback_error ) {
			chprintf(stdout, "I2C.MS8607: (INFO) In function `ms8607_reset`:\n");
			chibi_print_tepht_error_line(
				stdout,      "I2C.MS8607: (INFO) ", einfo);
		}
		chprintf(stdout, "I2C.MS8607: (INFO) Sensor reset failed.\n");
		chprintf(stdout, "I2C.MS8607: (INFO) This might mean that the sensor connected isn't an MS8607.\n");
		chprintf(stdout, "I2C.MS8607: (INFO) A different driver will be attempted.\n");
		return 1;
	}

	// Success.
	sensor_instance->sensor_type = pt_sensor_type_ms8607;
	*sensor_ref = ms8607_to_pt_sensor_interface(sensor_ms8607);

	chprintf(stdout, "I2C.MS8607: (INFO)  Disabling heater.\n");
	einfo = ms8607_disable_heater(sensor_ms8607, i2c_context);
	if ( tepht_is_error(einfo) ) {
		if ( tepht_error_get_status(einfo) != tepht_status_callback_error ) {
			chprintf(stdout, "I2C.MS8607: (WARNING) In function `ms8607_disable_heater`:\n");
			chibi_print_tepht_error_line(
				stdout,  "I2C.MS8607: (WARNING) ", einfo);
		}
		chprintf(stdout, "I2C.MS8607: (WARNING) Failed to disable heater.\n");
	}

	chprintf(stdout, "I2C.MS8607: (INFO)  Setting humidity resolution to 10b.\n");
	einfo = ms8607_set_humidity_resolution(sensor_ms8607, ms8607_humidity_resolution_10b, i2c_context);
	//einfo = ms8607_set_humidity_resolution(sensor_ms8607, ms8607_humidity_resolution_12b, i2c_context);
	if ( tepht_is_error(einfo) ) {
		if ( tepht_error_get_status(einfo) != tepht_status_callback_error ) {
			chprintf(stdout, "I2C.MS8607: (WARNING) In function `ms8607_set_humidity_resolution`:\n");
			chibi_print_tepht_error_line(
				stdout,  "I2C.MS8607: (WARNING) ", einfo);
		}
		chprintf(stdout, "I2C.MS8607: (WARNING) Failed to set humidity resolution.\n");
	}

	chprintf(stdout, "I2C.MS8607: (INFO)  Setting controller mode to NO HOLD.\n");
	ms8607_set_humidity_i2c_controller_mode(sensor_ms8607, ms8607_i2c_no_hold, i2c_context);

	return 0;
}

static uint8_t  init_and_reset_ms5840(
	BaseSequentialStream  *stdout,
	pt_sensor_instance    *sensor_instance,
	tepht_pt_sensor       *sensor_ref,
	i2c_context           *i2c_context
	)
{
	tepht_error_info  einfo;
	ms5840_sensor     *sensor_ms5840 = &sensor_instance->sensor.as_ms5840;

	chprintf(stdout, "I2C.MS5840: (INFO)  Initializing MS5840 sensor object.\n");
	einfo = ms5840_init_sensor(sensor_ms5840, &host_funcs_for_tepht);
	if ( tepht_is_error(einfo) ) {
		chprintf(stdout, "I2C.MS5840: (ERROR) In function `ms5840_init_sensor`:\n");
		chibi_print_tepht_error_line(
			stdout,      "I2C.MS5840: (ERROR) ", einfo);
		chprintf(stdout, "I2C.MS5840: (ERROR) Unable to continue.\n");
		return 1;
	}

	einfo = ms5840_reset(sensor_ms5840, i2c_context);
	if ( tepht_is_error(einfo) ) {
		if ( tepht_error_get_status(einfo) != tepht_status_callback_error ) {
			chprintf(stdout, "I2C.MS5840: (ERROR) In function `ms5840_reset`:\n");
			chibi_print_tepht_error_line(
				stdout,      "I2C.MS5840: (ERROR) ", einfo);
		}
		chprintf(stdout, "I2C.MS5840: (ERROR) Sensor reset failed.\n");
		chprintf(stdout, "I2C.MS5840: (ERROR) Unable to continue.\n");
		return 1;
	}

	// Success
	sensor_instance->sensor_type = pt_sensor_type_ms5840;
	*sensor_ref = ms5840_to_pt_sensor_interface(&sensor_instance->sensor.as_ms5840);
	return 0;
}

static uint8_t init_and_reset_sensor(
	BaseSequentialStream  *stdout,
	pt_sensor_instance    *sensor_instance,
	tepht_pt_sensor       *sensor_ref,
	i2c_context           *i2c_context
	)
{
	// We don't know whether the port is connected to an MS8607 or an MS5840
	// (at least at this point in the design when we are prototyping and
	// it's convenient to be able to use different sensors interchangably).
	// However, the MS8607 has a humidity sensor (on it's own I2C address!),
	// which means we can use that humidity sensor to distinguish between
	// the two possibilities. (And we DO need to do that:
	// Even if the pressure-temperature parts of these sensors share
	// the same I2C address and communicate the same way, we still have
	// to chose the correct driver, because the calculations for transforming
	// the raw readings are different between the two packages!)
	// So we will chose the correct driver by attempting to contact an ms8607 first.
	// If we're connected to an MS8607, then that attempt should succeed.
	// If we're connected to an MS5840, then that attempt will fail, because
	// the MS5840 will not respond to the request to reset the humidity sensor.
	// (TODO: We should probably have the code make sure any failures
	// /actually/ happened for humidity-sensor reasons, though that will 
	// probably involve changing the driver itself, as the current MS8607 driver
	// doesn't expose humidity sensor functionality as separate functions
	// in its API.)
	// So if the first attempt fails, we will failover to the MS5840,
	// which should then succeed, not unless there is nothing connected
	// or there is something else wrong with the I2C connection.
	//
	// Note that if we did this in the opposite order (try for MS5840 driver
	// first, then try the MS8607 driver), we would never be able
	// to initialize MS8607 sensors with the MS8607 driver, because it would
	// init "successfully" with the MS5840 driver instead, which would be
	// really bad, because then we could get misleading readings.

	uint8_t have_errors;
	have_errors =
		probe_for_ms8607_by_doing_init_and_reset(stdout, sensor_instance, sensor_ref, i2c_context);
	if ( !have_errors )
		return 0;

	have_errors =
		init_and_reset_ms5840(stdout, sensor_instance, sensor_ref, i2c_context);
	if ( !have_errors )
		return 0;

	return 1;
}

static WORKING_AREA(waThread1, 16384);
static msg_t  thread_main_for_i2c_sensors(void *p)
{
#if 0
	msg_t       stat;
	uint8_t     txbuf[1];
	uint8_t     rxbuf[1];
	uint8_t     i2c_expander_port_selection_bits = 0xFF;
#endif

	(void)p;
	chRegSetThreadName("i2c_sensors");
	size_t i;
	tepht_error_info    einfo;

	I2CDriver        *i2c_driver = &I2CD1;
	i2c_driver_meta  i2c_meta;

	i2c_driver_meta_init(&i2c_meta, (BaseSequentialStream *)&SD1, i2c_driver);

	i2c_context  pt_sensor_contexts[PT_SENSOR_COUNT_MAX];

	// Everything gets the same i2c_driver_meta because everything is on
	// the same I2C pins that are wired to the same I2C multiplexer.
	// (Use PT_SENSOR_COUNT_MAX instead of PT_SENSOR_COUNT_USED to ensure that
	// these structures are initialized with anything at all, even if we
	// don't plan to use them, because this makes failure modes more predictable.)
	for ( i = 0; i < PT_SENSOR_COUNT_MAX; i++ )
		pt_sensor_contexts[i].meta = &i2c_meta;

	// Manually assign port expander selection bits.
	// (This could be done with something like `1 << (i+1)`,
	// but this way makes the outcome look a little more clear.
	pt_sensor_contexts[PT_SENSOR_CPAP_IDX].i2c_expander_port_selection_bits   = 0x01;
	pt_sensor_contexts[PT_SENSOR_PUMP_IDX].i2c_expander_port_selection_bits   = 0x02;
	pt_sensor_contexts[PT_SENSOR_RETURN_IDX].i2c_expander_port_selection_bits = 0x04;

	// MS8607+MS5840 driver initialization, but only the bit(s) that
	// don't actually require I2C connectivity.
	chprintf(bss, "I2C.TEPHT: (INFO)  Initializing PHT (MS8607+MS5840) host functions / integration.\n");
	einfo = tepht_init_and_assign_host_functions(&host_funcs_for_tepht, NULL, &chibi_tepht_assign_functions);
	if ( tepht_is_error(einfo) ) {
		chprintf(bss, "I2C.TEPHT: (ERROR) In function `tepht_init_and_assign_host_functions`:\n");
		chibi_print_tepht_error_line(
			bss,      "I2C.TEPHT: (ERROR) ", einfo);
		chprintf(bss, "I2C.TEPHT: (ERROR) Unable to continue.\n");
		return 1;
	}

	// Configure ChibiOS's I2C system for our I2C pins; turn on the I2C peripheral.
	I2CConfig  i2c_config;
	i2c_config.ic_speed = 1500;
	i2cStart(i2c_meta.i2c_driver, &i2c_config);

#if 0
	chprintf(bss, "I2C.TCA9548A: (INFO)  Setting up I2C multiplexer.\n");

#if 0
	i2cflags_t  errors;
	chprintf(bss, "I2C.TCA9548A: (INFO)  Reading I2C multiplexer register.\n");
	stat = i2c_controller_receive(
		bss, i2c_meta.i2c_driver, I2C_EXPANDER_ADDR, rxbuf, sizeof(rxbuf));

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
	stat = i2c_controller_transmit(
		bss, i2c_meta.i2c_driver, I2C_EXPANDER_ADDR, txbuf, sizeof(txbuf), rxbuf, 0);

	if ( handle_i2c_errors(i2c_meta.i2c_driver, stat, 'T') ) {
		chprintf(bss, "I2C.TCA9548A: (ERROR) Problem when communicating with I2C multiplexer.\n");
		chprintf(bss, "I2C.TCA9548A: (ERROR) Unable to continue.\n");
		//return 1;
	}

#if 0
	chprintf(bss, "I2C.TCA9548A: (INFO)  Reading I2C multiplexer register.\n");
	stat = i2c_controller_receive(
		bss, i2c_meta.i2c_driver, I2C_EXPANDER_ADDR, rxbuf, sizeof(rxbuf));

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

	pt_sensor_instance  pt_sensor_instances[PT_SENSOR_COUNT_MAX];
	tepht_pt_sensor     pt_sensors[PT_SENSOR_COUNT_MAX];

	// TODO:
	// This should be a state-machine.
	// It should always-try-until-success.
	// If it encounters a disconnection-like failure partway into operation,
	// it should de-escalate into a non-operating state and start trying
	// to communicate with that sensor until it gets readings, at which point
	// it can re-escalate into the operating state.
	for ( i = 0; i < PT_SENSOR_COUNT_USED; i++ )
	{
		int8_t have_errors = init_and_reset_sensor(
			bss, &pt_sensor_instances[i], &pt_sensors[i], &pt_sensor_contexts[i]);
		if ( have_errors )
			return 1;
	}

	for ( i = 0; i < PT_SENSOR_COUNT_USED; i++ )
	{
		if ( pt_sensor_instances[i].sensor_type == pt_sensor_type_ms5840 )
			chprintf(bss, "I2C.MS5840: (INFO)  pt_sensor_instances[%d] == pt_sensor_type_ms5840\n", (int)i);
		else
		if ( pt_sensor_instances[i].sensor_type == pt_sensor_type_ms8607 )
			chprintf(bss, "I2C.MS8607: (INFO)  pt_sensor_instances[%d] == pt_sensor_type_ms8607\n", (int)i);
		else
		// enum member that didn't exist when writing this if-else statement.
			chprintf(bss, "I2C.??????: (WARNING)  pt_sensor_instances[%d] == <unknown pt_sensor_type: %d>\n",
				(int)i, (int)pt_sensor_instances[i].sensor_type);
	}

	palSetPad(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02);
	palSetPad(PROGRESS_LED_PORT_03, PROGRESS_LED_PAD_03);
	palSetPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);

	for ( i = 0; i < PT_SENSOR_COUNT_USED; i++ )
	{
		if ( pt_sensor_instances[i].sensor_type == pt_sensor_type_ms5840 ) {
			chprintf(bss, "I2C.MS5840: (INFO)  Sensor #%d: Setting pressure resolution to 2048.\n", (int)i);
			ms5840_set_pressure_resolution(
				&pt_sensor_instances[i].sensor.as_ms5840, ms5840_pressure_resolution_osr_2048, &pt_sensor_contexts[i]);
		}
		else
		if ( pt_sensor_instances[i].sensor_type == pt_sensor_type_ms8607 ) {
			chprintf(bss, "I2C.MS8607: (INFO)  Sensor #%d: Setting pressure resolution to 2048.\n", (int)i);
			ms8607_set_pressure_resolution(
				&pt_sensor_instances[i].sensor.as_ms8607, ms8607_pressure_resolution_osr_2048, &pt_sensor_contexts[i]);
		}
		else
		{ // enum member that didn't exist when writing this if-else statement.
			chprintf(bss, "I2C.??????: (WARNING)  Sensor #%d: Couldn't set pressure resolution. Not implemented for this sensor model.\n", (int)i);
		}
	}

	palSetPad(PROGRESS_LED_PORT_05, PROGRESS_LED_PAD_05);
	palSetPad(PROGRESS_LED_PORT_06, PROGRESS_LED_PAD_06);

	while (TRUE)
	{
		for ( i = 0; i < PT_SENSOR_COUNT_USED; i++ )
		{
			int32_t temperature = 0; // thousandths of degC
			int32_t pressure    = 0; // thousandths of mbar
			//int32_t humidity    = 0; // thousandths of %RH

			palClearPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
			palClearPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
			palSetPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);

			chprintf(bss, "\n");
			chprintf(bss, "I2C.TEPHT: (INFO)  Retrieving TP (temperature-pressure) readings from sensor %d (%s).\n",
				(int)i, sensor_id_to_name(i) );
			einfo = tepht_pt_sensor_read_temperature_pressure_int32(
				pt_sensors[i], &temperature, &pressure, &pt_sensor_contexts[i]);
			if ( tepht_is_error(einfo) )
			{
				palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
				palSetPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
				//if ( tepht_error_get_status(einfo) != tepht_status_callback_error ) {
					chibi_print_tepht_error_line(
						bss,  "I2C.TEPHT: (ERROR) ", einfo);
				//}
				chprintf(bss, "I2C.TEPHT: (ERROR) Failed to read TP data from sensor %d (%s).\n",
					(int)i, sensor_id_to_name(i) );
			}
			else
			{
				palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
				palSetPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
				chprintf(bss, "I2C.TEPHT: (INFO)  TP data received:\n");
				chprintf(bss, "    Temperature = %d.%d%d%d degC\n", (int)(temperature/1000), (int)((temperature/100)%10), (int)((temperature/10)%10), (int)(temperature%10) );
				chprintf(bss, "    Pressure    = %d.%d%d%d mbar\n", (int)(pressure/1000),    (int)((pressure/100)%10),    (int)((pressure/10)%10),    (int)(pressure%10) );
				handle_pressure_reading(bss, pressure, i);
			}
		} // for ( i = 0; i < PT_SENSOR_COUNT_USED; i++ )
	} // while(TRUE)

	// poor i2cStop statement can never execute.
	i2cStop(i2c_meta.i2c_driver);
	return 0;
}

#ifdef EARGEAR_PUMP_OPERATION
// This demo does a frequency sweep with PWM on GPIO18.
//
// It uses 50% duty cycle always.
// It starts with a waveform at 21.5kHz, then sweeps up to 24.5kHz.
//
// These are the tolerances on the muRata MZB3004T04 micropump that
// I am aiming to drive with this PWM, hence that specific band of frequencies.
// (Note that the raw PWM signal coming from a logic pin wouldn't be able to
// drive such a pump, but I do intend to send that through a MOSFET to switch
// a higher voltage (ex: 12V) power source, then pass that power signal
// through something like an LC filter to smooth the waveform into something
// more sinusoidal, so that the pump gets the power signal that it wants.)
//
// In playing with this, I also discovered that the clock frequency behind
// the PWM circuit shouldn't be set to 19.2MHz. For some reason that causes
// a very low frequency PWM, like a few Hz (but it was still frequency-sweeping,
// so the code wasn't just confused with the other demo). I set it to 9.6MHz
// instead, and everything worked fine. Dividing it by 8 also worked, but
// generally it is useful to use the highest clock possible, since that gives
// the resulting PWM waveform the highest fidelity.
//
static PWMConfig pwm_config = {
  9600000,     // PWM clock frequency; max is 19.2MHz. (Ignored by ChibiOS-RPi's original pwm_lld.c, which used 600kHz by default. Now adjustable.)
  893,         // PWM period that driver starts with when pwmStart is called. `clock_freq / period = pwm_freq`.
  NULL,        // Period callback.
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  }
};

static void set_pwm_config_freq_on_bcm2835(PWMConfig *config, uint32_t target_freq)
{
	config->period = config->frequency / target_freq;
}

static WORKING_AREA(waThread2, 16384);
static msg_t  thread_main_for_pump_control(void *p) {
	(void)p;
	chRegSetThreadName("pump_control");

	// NOTE: The PWM driver hardcodes the use of pin 18 on the BCM2835
	//   as the PWM pin. Thus, these couple functions will set GPIO18's
	//   alt-mode accordingly (though it might already be set in config.txt).
	pwmInit();
	pwmObjectInit(&PWMD1);

	// `duty_cycle` is an integer from 0 - 10000, with 10000 meaning 100%.
	uint32_t  duty_cycle = 5000;

	// muRata MZB3004T04 resonant frequency range is 21.5kHz to 24.5kHz.
	// So we'll start at the bottom of that range and sweep up to the top.
	pump_drive_resonant_frequency
	                     = 23300; // in Hz
	uint32_t freq_bottom = 21750; // in Hz
	uint32_t freq_top    = 24250; // in Hz
	set_pwm_config_freq_on_bcm2835(&pwm_config, pump_drive_resonant_frequency);

#if 0
	// muRata MZB1001T02 resonant frequency range is 24.0kHz to 27.0kHz.
	// So we'll start at the bottom of that range and sweep up to the top.
	pump_drive_resonant_frequency
	                     = 25500; // in Hz
	uint32_t freq_bottom = 24000; // in Hz
	uint32_t freq_top    = 27000; // in Hz
	set_pwm_config_freq_on_bcm2835(&pwm_config, pump_drive_resonant_frequency);
#endif

	uint8_t pump_was_on = 0;

	while (TRUE) {
		if ( pump_on && !pump_was_on )
		{
			pwmStart(&PWMD1, &pwm_config);
			PWM_CTL |= PWM0_MODE_MS;
			pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));

			palSetPad(ONBOARD_LED_PORT, ONBOARD_LED_PAD);
			pump_was_on = 1;
		}
		else
		if ( !pump_on && pump_was_on )
		{
			pump_was_on = 0;
			palClearPad(ONBOARD_LED_PORT, ONBOARD_LED_PAD);

			pwmDisableChannel(&PWMD1, 0);
			pwmStop(&PWMD1);

			// debounce: prevent cycling the pump at weird frequencies
			chThdSleepMilliseconds(2000);
		}
		else
		if ( pump_on && pump_was_on )
		{
			palClearPad(ONBOARD_LED_PORT, ONBOARD_LED_PAD);
			chThdSleepMilliseconds(200);
			palSetPad(ONBOARD_LED_PORT, ONBOARD_LED_PAD);
			chThdSleepMilliseconds(1800);

			(void)freq_bottom;
			(void)freq_top;
#if 0
			// Extra delay for when we need time for the pressure in the
			// test chamber to adjust to the last frequency change.
			chThdSleepMilliseconds(30000);

			// Change frequency.
			pwmDisableChannel(&PWMD1, 0);
			pwmStop(&PWMD1);

			pump_drive_resonant_frequency += 250;
			if ( pump_drive_resonant_frequency > freq_top )
				pump_drive_resonant_frequency = freq_bottom;

			set_pwm_config_freq_on_bcm2835(&pwm_config, pump_drive_resonant_frequency);
			BaseSequentialStream *output_stream = (BaseSequentialStream *)&SD1;
			(void)output_stream;
			#if 0
			chprintf(output_stream, "pwm_config->period   == %d\r\n", pwm_config.period);
			chprintf(output_stream, "pwm target frequency == %d\r\n", pump_drive_resonant_frequency);
			chprintf(output_stream, "pwm result frequency == %d\r\n", pwm_config.frequency / pwm_config.period);
			#endif

			pwmStart(&PWMD1, &pwm_config);
			PWM_CTL |= PWM0_MODE_MS;
			pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));
#endif
		}
		// else
		// if ( !pump_on && !pump_was_on )
		// { do nothing }
	}

	pwmDisableChannel(&PWMD1, 0);
	pwmStop(&PWMD1);
	return 0;
}
#endif // EARGEAR_PUMP_OPERATION

#if 0
// For the time being, it has been easier to implement this by modifying
// the internals of functions like `on_pressure_measurement` (called indirectly
// from the `thread_main_for_i2c_sensors` thread) than it has been
// to implement the below `thread_main_for_sensor_calibration` thread.
// Nonetheless, the below comment on calibration is probably my better
// explanation of why we do calibration, so I am leaving this comment around.

#ifdef EARGEAR_SENSOR_CALIBRATION
// We can program the chip to run in a "calibration" mode that takes readings
// from the pressure sensors and then calculates offsets that can be added
// to their readings (during normal operations) to get uniform results.
//
// This is necessary because the sensors used (ms8607, ms5840, etc) do not
// have perfect accuracy, and may be off by something like 2-4 mbar each.
//
// Remember: 1 millibar is approximately the same as 1 cmH2O, so this
// deviation is significant on the scale of something like CPAP treatment
// and other biological pressures.
//
// We don't care if they are perfect on an absolute scale, but we do need them
// to be within, say, 1 mbar of each other when reading the same "zero"
// at atmospheric pressure. THIS is doable, because the sensors are more
// *precise* than they are *accurate*. So even though they might disagree
// about what room-pressure is, they will do so by a consistent amount.
// This is what allows us to use a calibration routine to bring them into
// consistent agreement.
//
// Testing so far indicates that the sensors only experience about 0.1-0.2 mbar
// of "noise" typically, with about a maximum of 0.5 mbar of noise. This should
// be good enough for know when target-pressure has been reached, assuming
// we've done this calibration routine.
//
static WORKING_AREA(waThread2, 16384);
static msg_t  thread_main_for_sensor_calibration(void *p) {
	(void)p;
	chRegSetThreadName("calibration_output");

//TODO: implement this

	return 0;
}
#endif
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

	//palSetPadMode(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_03, PROGRESS_LED_PAD_03, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_05, PROGRESS_LED_PAD_05, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_06, PROGRESS_LED_PAD_06, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08, PAL_MODE_OUTPUT);
	palSetPadMode(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09, PAL_MODE_OUTPUT);
	//palSetPadMode(PROGRESS_LED_PORT_10, PROGRESS_LED_PAD_10, PAL_MODE_OUTPUT);

	//palClearPad(PROGRESS_LED_PORT_01, PROGRESS_LED_PAD_01);
	palClearPad(PROGRESS_LED_PORT_02, PROGRESS_LED_PAD_02);
	palClearPad(PROGRESS_LED_PORT_03, PROGRESS_LED_PAD_03);
	palClearPad(PROGRESS_LED_PORT_04, PROGRESS_LED_PAD_04);
	palClearPad(PROGRESS_LED_PORT_05, PROGRESS_LED_PAD_05);
	palClearPad(PROGRESS_LED_PORT_06, PROGRESS_LED_PAD_06);
	palClearPad(PROGRESS_LED_PORT_07, PROGRESS_LED_PAD_07);
	palClearPad(PROGRESS_LED_PORT_08, PROGRESS_LED_PAD_08);
	palClearPad(PROGRESS_LED_PORT_09, PROGRESS_LED_PAD_09);
	//palClearPad(PROGRESS_LED_PORT_10, PROGRESS_LED_PAD_10);

	calculate_system_time_reference();
	init_sensor_data_buffers();

	//(void)calculate_system_time_in_usecs();

	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, thread_main_for_i2c_sensors, NULL);

#ifdef EARGEAR_PUMP_OPERATION
	chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, thread_main_for_pump_control, NULL);
#endif

//#ifdef EARGEAR_SENSOR_CALIBRATION
//	chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, thread_main_for_sensor_calibration, NULL);
//#endif

	// Events servicing loop.
	chThdWait(chThdSelf());

	return 0;
}
