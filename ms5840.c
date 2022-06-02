/// \file ms5840.c
///
/// \brief MS5840 Temperature sensor driver source file
///
/// Copyright (c) 2016 Measurement Specialties. All rights reserved.
///
/// For details on programming, refer to ms5840 datasheet :
/// https://www.tme.eu/Document/365c00a8a24c1e520dd30a20356b788d/ENG_DS_MS5840-02BA_A5.pdf
///
///

#include "ms5840.h"

#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

// PSENSOR device address
#define PSENSOR_ADDR                                        0x76 //0b1110110

// PSENSOR device commands
#define PSENSOR_RESET_COMMAND                               0x1E
#define PSENSOR_START_PRESSURE_ADC_CONVERSION               0x40
#define PSENSOR_START_TEMPERATURE_ADC_CONVERSION            0x50
#define PSENSOR_READ_ADC                                    0x00

#define PSENSOR_CONVERSION_OSR_MASK                         0x0F

#define PSENSOR_CONVERSION_TIME_OSR_256                     1000
#define PSENSOR_CONVERSION_TIME_OSR_512                     2000
#define PSENSOR_CONVERSION_TIME_OSR_1024                    3000
#define PSENSOR_CONVERSION_TIME_OSR_2048                    5000
#define PSENSOR_CONVERSION_TIME_OSR_4096                    9000
#define PSENSOR_CONVERSION_TIME_OSR_8192                    18000

// PSENSOR commands
#define PROM_ADDRESS_READ_ADDRESS_0                         0xA0

// Coefficients indexes for temperature and pressure computation
#define CRC_INDEX                                           0
#define PRESSURE_SENSITIVITY_INDEX                          1
#define PRESSURE_OFFSET_INDEX                               2
#define TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX            3
#define TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX                 4
#define REFERENCE_TEMPERATURE_INDEX                         5
#define TEMP_COEFF_OF_TEMPERATURE_INDEX                     6

// Global constants
static uint32_t psensor_conversion_time[6] = {
		PSENSOR_CONVERSION_TIME_OSR_256,
		PSENSOR_CONVERSION_TIME_OSR_512,
		PSENSOR_CONVERSION_TIME_OSR_1024,
		PSENSOR_CONVERSION_TIME_OSR_2048,
		PSENSOR_CONVERSION_TIME_OSR_4096,
		PSENSOR_CONVERSION_TIME_OSR_8192
	};

// Static functions
// Pressure sensor functions
static enum ms5840_status psensor_write_command(ms5840_sensor *sensor, uint8_t, void *caller_context);
static enum ms5840_status psensor_read_eeprom_coeff(ms5840_sensor *sensor, uint8_t, uint16_t*, void *caller_context);
static enum ms5840_status psensor_read_eeprom(ms5840_sensor *sensor, void *caller_context);
static enum ms5840_status psensor_conversion_and_read_adc(ms5840_sensor *sensor, uint8_t, uint32_t *, void *caller_context);
static bool psensor_crc_check (uint16_t *n_prom, uint8_t crc);
static enum ms5840_status psensor_read_pressure_and_temperature(ms5840_sensor *sensor, int32_t *, int32_t *, void *caller_context);

static enum ms5840_status
	i2c_controller_read_unimpl(void *caller_context, ms5840_i2c_controller_packet *const packet)
{
	(void)caller_context;
	(void)packet;
	return ms5840_status_i2c_read_unimplemented;
}

static enum ms5840_status
	i2c_controller_write_unimpl(void *caller_context, ms5840_i2c_controller_packet *const packet)
{
	(void)caller_context;
	(void)packet;
	return ms5840_status_i2c_write_unimplemented;
}

static enum ms5840_status
	sleep_ms_unimpl(void *caller_context, uint32_t milliseconds)
{
	(void)caller_context;
	(void)milliseconds;
	return ms5840_status_sleep_ms_unimplemented;
}

static void print_string_stub(void *caller_context, const char *text)
{
	(void)caller_context;
	(void)text;
}

static void print_int64_stub(void *caller_context,  int64_t number, uint8_t pad_width,  ms5840_bool  pad_with_zeroes)
{
	(void)caller_context;
	(void)number;
	(void)pad_width;
	(void)pad_with_zeroes;
}

/// \brief Initializes the `ms5840_host_functions` struct; this should be called
///        *before* assigning function pointers into the structure.
///
static enum ms5840_status ms5840_init_host_functions(ms5840_host_functions *deps)
{
	if ( deps == NULL )
		return ms5840_status_null_argument;

	deps->validated_ = 0;

	deps->i2c_controller_read           = &i2c_controller_read_unimpl;
	deps->i2c_controller_write          = &i2c_controller_write_unimpl;
	deps->sleep_ms                      = &sleep_ms_unimpl;
	deps->print_string                  = &print_string_stub;
	deps->print_int64                   = &print_int64_stub;

	return ms5840_status_ok;
}

static enum ms5840_status  ms5840_validate_mandatory_depends(ms5840_host_functions *deps)
{
	assert(deps != NULL);

	if ( deps->validated_ )
		return ms5840_status_ok;

	if ( deps->i2c_controller_read == NULL
	||   deps->i2c_controller_read == &i2c_controller_read_unimpl )
		return ms5840_status_i2c_read_unimplemented;
	else
	if ( deps->i2c_controller_write == NULL
	||   deps->i2c_controller_write == &i2c_controller_write_unimpl )
		return ms5840_status_i2c_write_unimplemented;
	else
	if ( deps->sleep_ms == NULL
	||   deps->sleep_ms == &sleep_ms_unimpl )
		return ms5840_status_sleep_ms_unimplemented;

	// Under no condition should any of the function pointers be NULL.
	// Even unimplemented things should be assigned error handlers or stubs
	// by the `ms5840_init_host_functions` function.
	if( deps->i2c_controller_read == NULL
	||  deps->i2c_controller_write == NULL
	||  deps->sleep_ms == NULL
	||  deps->print_string == NULL
	||  deps->print_int64 == NULL )
		return ms5840_status_null_host_function;

	// If we've made it to the end of this function, then it is at least plausible
	// that the driver can use this host functions object to do *something*.
	// (More specific features of the driver might require functions that we
	// didn't check here. However, absent a more complicated configuration system,
	// we'll just have to check those later, at point-of-use.)
	deps->validated_ = 1;
	return ms5840_status_ok;
}

/// \brief    Creates a `ms5840_host_functions` object (the `dependencies` parameter)
///           to store function pointers that implement the driver's dependencies.
///
/// \details  The purpose of this function is to create a `ms5840_host_functions`
///           object, which can then be used by the MS5840 driver to satisfy
///           its dependencies.
///
///           This must be called before calling the `ms5840_init_sensor`
///           function, as the `ms5840_init_sensor` function requires the
///           `ms5840_host_functions` object that is populated by this function.
///
///           This function works in 3 steps:
///
///           (1) It initializes the `ms5840_host_functions` object
///             given by the `dependencies` parameter. This places the object
///             into a known state so that the 3rd phase of this function can
///             know which functions were implemented by the caller/host.
///
///           (2) It calls the given `assign_functions` callback on `dependencies`.
///             The callback should create function pointers from host functions
///             that implement the various requirements of the MS5840 driver,
///             such as I2C I/O and timing mechanisms. Those function pointers
///             should be assigned to the various members of the `dependencies`
///             structure. See the `ms5840_host_functions` for details on the
///             necessary functions.
///
///           (3) After the callback returns, this function then validates
///             the resulting `ms5840_host_functions` object to ensure that
///             minimal requirements are met. Appropriate error codes are
///             returned if the driver's dependencies were not satisfied.
///
///           The `assign_functions` callback shall NOT assign NULL to any
///           members of the `ms5840_host_functions *dependencies` structure.
///           When a function pointer is optional and no implementation
///           is available, `assign_functions` should leave that member
///           unmodified.
///
///           The `ms5840_init_and_assign_host_functions` function will
///           have already assigned stubs (and missing requirement detectors)
///           to the members of the `ms5840_host_functions` structure.
///
///           This function is reentrant, idempotent, non-blocking,
///           and does not perform any I/O. These properties assume that
///           the `assign_functions` callback also possesses the same
///           corresponding properties. This function is thread-safe
///           as long as, during this function's execution, no other threads
///           read from or write to the objects pointed to by this function's
///           arguments.
///
/// \param[out] ms5840_host_functions* : Struct with callbacks that implement I2C controller functions.
/// \param[in] void* caller_context : This is passed to the `assign_functions`
///           callback's `caller_context` parameter.
/// \param[in] void (*assign_functions)(ms5840_host_functions *dependencies, void *caller_context):
///           Pointer to a caller-implemented function that shall assign pointers
///           to implementation functions that are required by the driver.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_null_argument : Returned if the `dependencies` or `assign_functions` parameters were NULL.
///       - ms5840_status_null_host_function : Returned if NULL was assigned to any of the members of `dependencies`.
///       - ms5840_status_i2c_read_unimplemented : Returned if the `i2c_controller_read` function was not assigned.
///       - ms5840_status_i2c_write_unimplemented : Returned if the `i2c_controller_write` function was not assigned.
///       - ms5840_status_sleep_ms_unimplemented : Returned if the `sleep_ms` function was not assigned.
///
enum ms5840_status ms5840_init_and_assign_host_functions(
	ms5840_host_functions *dependencies,
	void *caller_context,
	void (*assign_functions)(ms5840_host_functions *dependencies, void *caller_context)
	)
{
	enum ms5840_status status;

	// The call to `ms5840_init_host_functions` will enforce that `dependencies` is non-NULL.
	status = ms5840_init_host_functions(dependencies);
	if ( status != ms5840_status_ok )
		return status;

	// It's OK if `caller_context` is NULL.
	// Whether that's required to be non-NULL or not is up to the caller, so
	// they would have to enforce that from within `assign_functions`, if they
	// wanted such a thing.
	// (`caller_context` being NULL is actually pretty likely, in this case!
	// The host functions are likely to be known at compile-time, so it
	// would be unnecessary to use the dynamic (run-time) information
	// referenced by the `caller_context` object to compute their values.)

	assign_functions(dependencies, caller_context);

	return ms5840_validate_mandatory_depends(dependencies);
}

/// \brief    Initializes a new `ms5840_sensor` object.
///
/// \details  This function's purpose is to place the given `new_sensor`
///           argument into a predictable state and provide it with some
///           default configuration. This function does not actually communicate
///           with the sensor in any way.
///
///           This must be called after `ms5840_init_and_assign_host_functions`
///           has been called at least once (to obtain the argument for the
///           `depends_to_use` parameter), and it must be called before any
///           other function that requires a `ms5840_sensor*` type parameter
///           (usually `ms5840_reset`).
///
///           This function is reentrant, idempotent, non-blocking,
///           and does not perform any I/O. This function is thread-safe
///           as long as, during this function's execution, no other threads
///           read from or write to that call's `new_sensor` instance, and
///           no other threads write to that call's `depends_to_use` instance.
///
/// \param[out] ms5840_sensor *new_sensor : The new sensor object.
/// \param[in]  const ms5840_host_functions *depends_to_use : Specifies the
///           functions that this sensor can call to do things such as I2C I/O
///           and timing.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok : Sensor object was initailized successfully
///       - ms5840_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms5840_status_null_argument : The pointer provided for the `depends_to_use` parameter was NULL.
///       - ms5840_status_i2c_read_unimplemented : Returned if the `i2c_controller_read` function was not assigned.
///       - ms5840_status_i2c_write_unimplemented : Returned if the `i2c_controller_write` function was not assigned.
///       - ms5840_status_sleep_ms_unimplemented : Returned if the `sleep_ms` function was not assigned.
///
enum ms5840_status  ms5840_init_sensor(ms5840_sensor *new_sensor,  ms5840_host_functions *depends_to_use)
{
	enum ms5840_status status;

	if ( new_sensor == NULL )
		return ms5840_status_null_sensor;

	if ( depends_to_use == NULL )
		return ms5840_status_null_argument;

	status = ms5840_validate_mandatory_depends(depends_to_use);
	if ( status != ms5840_status_ok )
		return status;

	ms5840_sensor *s = new_sensor;
	s->host_funcs                  = depends_to_use;

	// Defaults
	s->psensor_resolution_osr      = ms5840_pressure_resolution_osr_8192;
	s->psensor_coeff_read          = false;

	uint8_t i = 0;
	for (; i < MS5840_COEFFICIENT_COUNT+1; i++)
		s->eeprom_coeff[i] = 0;

	return ms5840_status_ok;
}

/// \brief Check whether MS5840 device is connected
///
/// \param[in] ms5840_sensor *sensor : Sensor object to test for connectivity.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return bool : status of MS5840
///       - true : Device is present
///       - false : Device is not acknowledging I2C address
///
bool ms5840_is_connected(ms5840_sensor *sensor,  void *caller_context)
{
	assert( sensor != NULL );

	enum ms5840_status status;

	ms5840_i2c_controller_packet transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 0,
		.data        = NULL,
	};

	/* Do the transfer */
	status = sensor->host_funcs->i2c_controller_write(caller_context, &transfer);
	if( status != ms5840_status_ok)
		return false;

	return true;
}

/// \brief Reset the MS5840 device
///
/// \details
///           This function is reentrant. It blocks and performs I2C I/O.
///           It is thread-safe as long as, during this function's execution,
///           no other threads read from or write to the given `sensor` instance.
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to be reset.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok : I2C transfer completed successfully
///       - ms5840_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - ms5840_status_callback_error : Error occurred within a ms5840_host_functions function
///
enum ms5840_status  ms5840_reset(ms5840_sensor *sensor,  void *caller_context)
{
	if ( sensor == NULL )
		return ms5840_status_null_sensor;

	enum ms5840_status status;

	status = psensor_write_command(sensor, PSENSOR_RESET_COMMAND, caller_context);
	if( status != ms5840_status_ok)
		return status;

	return ms5840_status_ok;
}

/// \brief    Reads the temperature and pressure values.
///
/// \details  The results are returned in thousanths as a way to preserve
///           the sensor's resolution while using integer types.
///
/// \param[in] ms5840_sensor* sensor : The sensor to use for measuring
/// \param[out] int32_t* : Thousanths of degC temperature value
/// \param[out] int32_t* : Microbar pressure value (thousanths of millibar)
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok : I2C transfer completed successfully
///       - ms5840_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms5840_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - ms5840_status_callback_error : Error occurred within a ms5840_host_functions function
///       - ms5840_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - ms5840_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - ms5840_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
enum ms5840_status ms5840_read_temperature_pressure_int32(ms5840_sensor *sensor, int32_t *t, int32_t *p,  void *caller_context)
{
	if ( sensor == NULL )
		return ms5840_status_null_sensor;

	if ( t == NULL || p == NULL )
		return ms5840_status_null_argument;

	enum ms5840_status status;

	status = psensor_read_pressure_and_temperature(sensor, t, p, caller_context);
	if(status != ms5840_status_ok)
		return status;

	return ms5840_status_ok;
}

/// \brief    Reads the temperature and pressure values.
///
/// \param[in] ms5840_sensor* sensor : The sensor to use for measuring
/// \param[out] float* : degC temperature value
/// \param[out] float* : mbar pressure value
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok : I2C transfer completed successfully
///       - ms5840_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms5840_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - ms5840_status_callback_error : Error occurred within a ms5840_host_functions function
///       - ms5840_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - ms5840_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - ms5840_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
enum ms5840_status ms5840_read_temperature_pressure_float32(ms5840_sensor *sensor, float *t, float *p, void *caller_context)
{
	enum ms5840_status  status;
	int32_t t_;
	int32_t p_;

	// NULL status of the `sensor` parameter will be checked by `ms5840_read_temperature_pressure_i32`.

	if ( t == NULL || p == NULL )
		return ms5840_status_null_argument;

	status = ms5840_read_temperature_pressure_int32(sensor, &t_, &p_, caller_context);
	*t = ((float)t_) / 1000;
	*p = ((float)p_) / 1000;

	return status;
}

/******************** Functions from Pressure sensor ********************/

///
/// \brief Writes the Pressure Sensor 8-bits command with the value passed
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to send the command to
/// \param[in] uint8_t : Command value to be written.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok : I2C transfer completed successfully
///       - ms5840_status_callback_error : Error occurred within a ms5840_host_functions function
///
static enum ms5840_status psensor_write_command(ms5840_sensor *sensor, uint8_t cmd, void *caller_context)
{
	assert( sensor != NULL );
	uint8_t data[1];

	data[0] = cmd;

	ms5840_i2c_controller_packet transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 1,
		.data        = data,
	};

	/* Do the transfer */
	return sensor->host_funcs->i2c_controller_write(caller_context, &transfer);
}

/// \brief Set pressure ADC resolution.
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to configure
/// \param[in] ms5840_pressure_resolution : Resolution requested
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///         (As of this writing, this function does not do any I2C I/O and
///         does not call any host functions, so `caller_context` is unused here,
///         but nonetheless provided for sake of future-proofing.)
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok
///       - ms5840_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///
///
enum ms5840_status  ms5840_set_pressure_resolution(ms5840_sensor *sensor, enum ms5840_pressure_resolution res, void *caller_context)
{
	(void)caller_context;
	if ( sensor == NULL )
		return ms5840_status_null_sensor;

	sensor->psensor_resolution_osr = res;
	return ms5840_status_ok;
}

/// \brief Reads the psensor EEPROM coefficient stored at address provided.
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to retrieve EEPROM coefficients from
/// \param[in] uint8_t : Address of coefficient in EEPROM
/// \param[out] uint16_t* : Value read in EEPROM
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok : All operations completed successfully
///       - ms5840_status_callback_error : Error occurred within a ms5840_host_functions function
///       - ms5840_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///
static enum ms5840_status psensor_read_eeprom_coeff(ms5840_sensor *sensor, uint8_t command, uint16_t *coeff, void *caller_context)
{
	assert( sensor != NULL );
	assert( coeff != NULL );

	enum ms5840_status status;
	uint8_t buffer[2];

	buffer[0] = 0;
	buffer[1] = 0;

	/* Read data */
	ms5840_i2c_controller_packet read_transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 2,
		.data        = buffer,
	};

	// Send the conversion command
	status = psensor_write_command(sensor, command, caller_context);
	if(status != ms5840_status_ok)
		return status;

	status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if(status != ms5840_status_ok)
		return status;

	*coeff = (buffer[0] << 8) | buffer[1];

	if (*coeff == 0)
		return ms5840_status_eeprom_is_zero;

	return ms5840_status_ok;
}

/// \brief Reads the ms5840 EEPROM coefficients to store them for computation.
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to retrieve EEPROM coefficients from
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok : All operations completed successfully
///       - ms5840_status_callback_error : Error occurred within a ms5840_host_functions function
///       - ms5840_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - ms5840_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///
static enum ms5840_status psensor_read_eeprom(ms5840_sensor *sensor, void *caller_context)
{
	assert( sensor != NULL );
	enum ms5840_status status;
	uint8_t i;

	for( i=0 ; i< MS5840_COEFFICIENT_COUNT ; i++)
	{
		status = psensor_read_eeprom_coeff(
			sensor, PROM_ADDRESS_READ_ADDRESS_0 + i*2, sensor->eeprom_coeff+i, caller_context);
		if(status != ms5840_status_ok)
			return status;
	}

	if( !psensor_crc_check( sensor->eeprom_coeff, (sensor->eeprom_coeff[CRC_INDEX] & 0xF000)>>12 ) )
		return ms5840_status_eeprom_crc_error;

	sensor->psensor_coeff_read = true;

	return ms5840_status_ok;
}

/// \brief Triggers conversion and reading of ADC value
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to read the adc from
/// \param[in] uint8_t : Command used for conversion (will determine Temperature vs Pressure and osr)
/// \param[out] uint32_t* : ADC value.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok : All operations completed successfully
///       - ms5840_status_callback_error : Error occurred within a ms5840_host_functions function
///       - ms5840_status_measurement_invalid : I2C transfer(s) completed, but data received was invalid
///
static enum ms5840_status psensor_conversion_and_read_adc(ms5840_sensor *sensor, uint8_t cmd, uint32_t *adc, void *caller_context)
{
	assert( sensor != NULL );
	assert( adc != NULL );

	enum ms5840_status status;
	uint8_t buffer[3];

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    ms5840_i2c_controller_packet read_transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 3,
		.data        = buffer,
	};

	// TODO: implement?
#if 0
	status = psensor_write_command(sensor, cmd, caller_context);
	if( status != ms5840_status_ok)
		return status;

	while ( microsecs() < psensor_conversion_time[ (cmd & PSENSOR_CONVERSION_OSR_MASK)/2 ] )
		spin();

	// Send the read command
	status = psensor_write_command(sensor, PSENSOR_READ_ADC, caller_context);
	if( status != ms5840_status_ok)
		return status;

    status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if( status != ms5840_status_ok )
		return status;

	// TODO: If failure, should probably send reset sequence?
	// API seems to leak problems, because datasheet is saying that all bets are
	// off if we talk to the thing while it's converting. But... what if we didn't
	// wait long enough? How do we recover? It might not be enough to say,
	// "It didn't work" and move on with life, because now we might get bogus
	// data from the thing. It'd be a good idea to reset and reload EEPROM at
	// that point.
	// TODO: 1-deep command queueing: the driver always knows how long it will
	//   be before it can perform another action, and will not allow other
	//   actions to be performed. Probably make sure the state-machine is
	//   simplified compared to the datasheet.
#endif
	status = psensor_write_command(sensor, cmd, caller_context);
	// 20ms wait for conversion
	sensor->host_funcs->sleep_ms(caller_context, psensor_conversion_time[ (cmd & PSENSOR_CONVERSION_OSR_MASK)/2 ]/1000);
	if( status != ms5840_status_ok)
		return status;

	// Send the read command
	status = psensor_write_command(sensor, PSENSOR_READ_ADC, caller_context);
	if( status != ms5840_status_ok)
		return status;

    status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if( status != ms5840_status_ok )
		return status;

	*adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

	return status;
}

/// \brief Compute temperature and pressure
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to read temperature and pressure from
/// \param[out] float* : Celsius Degree temperature value
/// \param[out] float* : mbar pressure value
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms5840_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms5840_status : status of MS5840
///       - ms5840_status_ok : All operations completed successfully
///       - ms5840_status_callback_error : Error occurred within a ms5840_host_functions function
///       - ms5840_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - ms5840_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - ms5840_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
static enum ms5840_status psensor_read_pressure_and_temperature(ms5840_sensor *sensor, int32_t *temperature, int32_t *pressure, void *caller_context)
{
	assert( sensor      != NULL );
	assert( temperature != NULL );
	assert( pressure    != NULL );

	enum ms5840_status status = ms5840_status_ok;
	uint32_t adc_temperature, adc_pressure;
	int32_t dT, TEMP;
	int64_t OFF, SENS, P, T2, OFF2, SENS2;
	uint8_t cmd;

	// If first time adc is requested, get EEPROM coefficients
	if( sensor->psensor_coeff_read == false )
		status = psensor_read_eeprom(sensor, caller_context);
	if( status != ms5840_status_ok)
		return status;

	// First read temperature
	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_TEMPERATURE_ADC_CONVERSION;
	status = psensor_conversion_and_read_adc(sensor, cmd, &adc_temperature, caller_context);
	if( status != ms5840_status_ok)
		return status;

	// Now read pressure
	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_PRESSURE_ADC_CONVERSION;
	status = psensor_conversion_and_read_adc(sensor, cmd, &adc_pressure, caller_context);
	if( status != ms5840_status_ok)
		return status;

	if (adc_temperature == 0 || adc_pressure == 0)
		return ms5840_status_measurement_invalid;

	// Difference between actual and reference temperature = D2 - Tref
	dT = (int32_t)adc_temperature - ( (int32_t)sensor->eeprom_coeff[REFERENCE_TEMPERATURE_INDEX] <<8 );

	// Actual temperature = 2000 + dT * TEMPSENS
	TEMP = 2000 + ((int64_t)dT * (int64_t)sensor->eeprom_coeff[TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;

	// Second order temperature compensation
	if( TEMP < 2000 )
	{
		T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
		OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;

		if( TEMP < -1500 )
		{
			OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
		}
	}
	else
	{
		T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
		OFF2 = 0 ;
		SENS2 = 0 ;
	}

	// OFF = OFF_T1 + TCO * dT
	OFF = ( (int64_t)(sensor->eeprom_coeff[PRESSURE_OFFSET_INDEX]) << 17 )
		+ ( ( (int64_t)(sensor->eeprom_coeff[TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT ) >> 6 ) ;
	OFF -= OFF2 ;

	// Sensitivity at actual temperature = SENS_T1 + TCS * dT
	SENS = ( (int64_t)sensor->eeprom_coeff[PRESSURE_SENSITIVITY_INDEX] << 16 )
		+ ( ((int64_t)sensor->eeprom_coeff[TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 7 ) ;
	SENS -= SENS2 ;

	// Temperature compensated pressure = D1 * SENS - OFF
	P = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15 ;

	//*temperature = ( (float)TEMP - T2 ) / 100;
	//*pressure = (float)P / 100;

	*temperature = 10*( TEMP - T2 );
	*pressure = 10*P;

	return status;
}

/// \brief CRC check
///
/// \param[in] uint16_t *: List of EEPROM coefficients
/// \param[in] uint8_t : crc to compare
///
/// \return bool : TRUE if CRC is OK, FALSE if KO
///
static bool psensor_crc_check (uint16_t *n_prom, uint8_t crc)
{
	assert( n_prom != NULL );
	uint8_t cnt, n_bit;
	uint16_t n_rem, crc_read;

	n_rem = 0x00;
	crc_read = n_prom[0];
	n_prom[MS5840_COEFFICIENT_COUNT] = 0;
	n_prom[0] = (0x0FFF & (n_prom[0]));    // Clear the CRC byte

	for( cnt = 0 ; cnt < (MS5840_COEFFICIENT_COUNT+1)*2 ; cnt++ ) {

		// Get next byte
		if (cnt%2 == 1)
			n_rem ^=  n_prom[cnt>>1] & 0x00FF ;
		else
			n_rem ^=  n_prom[cnt>>1]>>8 ;

		for( n_bit = 8; n_bit > 0 ; n_bit-- ) {

			if( n_rem & 0x8000 )
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem <<= 1;
		}
	}
	n_rem >>= 12;
	n_prom[0] = crc_read;

	return  ( n_rem == crc );
}

const char *ms5840_stringize_error(enum ms5840_status error_code)
{
	switch(error_code)
	{
		case ms5840_status_ok:
			return "Status OK, success. (ms5840_status_ok)";

		case ms5840_status_waiting:
			return "Status OK, but waiting for sensor to respond. (ms5840_status_waiting)";

		case ms5840_status_null_argument:
			return "Error: A function in the ms5840 driver was called with a NULL argument when a non-NULL argument was required. (ms5840_status_null_argument)";

		case ms5840_status_null_sensor:
			return "Error: A function in the ms5840 driver was given a NULL pointer to a `ms5840_sensor` object. These are required to be non-NULL. (ms5840_status_null_sensor)";

		case ms5840_status_null_host_function:
			return "Error: A member of the `ms5840_host_functions` structure was NULL. This could lead to crashing and unpredictable behavior later on. (ms5840_status_null_host_function)";

		case ms5840_status_callback_error:
			return "Error occurred within a function dispatched from the `ms5840_host_functions` structure. (ms5840_status_callback_error)";

		case ms5840_status_callback_i2c_nack:
			return "I2C transfer did not complete. Peripheral responded with NACK. (ms5840_status_callback_i2c_nack)";

		case ms5840_status_eeprom_is_zero: // Formerly ms5840_status_crc_error
			return "One or more EEPROM coefficients were received as 0, preventing measurement. (ms5840_status_eeprom_is_zero)";

		case ms5840_status_eeprom_crc_error: // Formerly ms5840_status_crc_error
			return "EEPROM coefficients retrieved from the MS5840 did not pass CRC check. (ms5840_status_eeprom_crc_error)";

		case ms5840_status_measurement_invalid: // Formerly ms5840_status_i2c_transfer_error
			return "EEPROM is OK and I2C transfer completed, but data received was invalid. (ms5840_status_measurement_invalid)";

		case ms5840_status_i2c_read_unimplemented:
			return "An implementation for the `ms5840_host_functions.i2c_controller_read`"
				" function was not provided, but was needed to complete an operation.";

		case ms5840_status_i2c_write_unimplemented:
			return "An implementation for the `ms5840_host_functions.i2c_controller_write`"
				" function was not provided, but was needed to complete an operation.";

		case ms5840_status_sleep_ms_unimplemented:
			return "An implementation for the `ms5840_host_functions.sleep_ms`"
				" function was not provided, but was needed to complete an operation.";

		default: break;
	}
	return "Error code is not a valid ms5840_status.";
}

#ifdef __cplusplus
}
#endif
