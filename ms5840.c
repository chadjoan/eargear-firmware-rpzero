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
#include <string.h>

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
static tepht_status      psensor_write_command(ms5840_sensor *sensor, uint8_t, void *caller_context);
static tepht_error_info  psensor_read_eeprom_coeff(ms5840_sensor *sensor, uint8_t, uint16_t*, void *caller_context);
static tepht_error_info  psensor_read_eeprom(ms5840_sensor *sensor, void *caller_context);
static tepht_error_info  psensor_conversion_and_read_adc(ms5840_sensor *sensor, uint8_t, uint32_t *, void *caller_context);
static bool              psensor_crc_check (uint16_t *n_prom, uint8_t crc);
static tepht_error_info  psensor_read_pressure_and_temperature(ms5840_sensor *sensor, int32_t *, int32_t *, void *caller_context);


static const char *ms5840_get_device_model_name(void *ms5840_sensor) {
	(void)ms5840_sensor;
	return "MS5840";
}

static const char *ms5840_get_driver_prefix(void *ms5840_sensor) {
	(void)ms5840_sensor;
	return "ms5840";
}

static tepht_driver_context_accessor_vtable  ms5840_dca_vtable =
	{
		.get_device_model_name = ms5840_get_device_model_name,
		.get_driver_prefix     = ms5840_get_driver_prefix
	};

static tepht_driver_context_accessor  ms5840_default_dca =
	{
		.self   = NULL,
		.vtable = &ms5840_dca_vtable
	};

static inline
	const tepht_driver_context_accessor  *ms5840_get_dca(ms5840_sensor *sensor)
{
	if ( sensor != NULL )
		return &(sensor->context_accessor);
	else
		return &ms5840_default_dca;
}

static inline tepht_error_info  ms5840_success(ms5840_sensor *sensor) {
	return tepht_success(ms5840_get_dca(sensor));
}

// This must be a macro to preserve __FILE__ and __LINE__ information.
#define  ms5840_error(sensor, status)  (tepht_error(ms5840_get_dca(sensor), (status)))

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
/// \param[in]  const tepht_host_functions *depends_to_use : Specifies the
///           functions that this sensor can call to do things such as I2C I/O
///           and timing.
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok : Sensor object was initailized successfully
///       - tepht_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - tepht_status_null_argument : The pointer provided for the `depends_to_use` parameter was NULL.
///       - tepht_status_i2c_read_unimplemented : Returned if the `i2c_controller_read` function was not assigned.
///       - tepht_status_i2c_write_unimplemented : Returned if the `i2c_controller_write` function was not assigned.
///       - tepht_status_sleep_ms_unimplemented : Returned if the `sleep_ms` function was not assigned.
///
tepht_error_info  ms5840_init_sensor(ms5840_sensor *new_sensor,  tepht_host_functions *depends_to_use)
{
	tepht_status status;

	if ( new_sensor == NULL )
		return ms5840_error(new_sensor, tepht_status_null_sensor);

	if ( depends_to_use == NULL )
		return ms5840_error(new_sensor, tepht_status_null_argument);

	status = tepht_validate_mandatory_depends(depends_to_use);
	if ( status != tepht_status_ok )
		return ms5840_error(new_sensor, status);

	ms5840_sensor  const_init_sensor = {
		.host_funcs              = depends_to_use,

		.context_accessor.self   = new_sensor,
		.context_accessor.vtable = &ms5840_dca_vtable,

		// Defaults
		.psensor_resolution_osr  = ms5840_pressure_resolution_osr_8192,
		.psensor_coeff_read      = false
	};

	memcpy(new_sensor, &const_init_sensor, sizeof(ms5840_sensor));

	uint8_t i = 0;
	for (; i < MS5840_COEFFICIENT_COUNT+1; i++)
		new_sensor->eeprom_coeff[i] = 0;

	return ms5840_success(new_sensor);
}

/// \brief Check whether MS5840 device is connected
///
/// \param[in] ms5840_sensor *sensor : Sensor object to test for connectivity.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_bool : status of MS5840
///       - true : Device is present
///       - false : Device is not acknowledging I2C address
///
tepht_bool  ms5840_is_connected(ms5840_sensor *sensor,  void *caller_context)
{
	assert( sensor != NULL );

	tepht_status status;

	tepht_i2c_controller_packet  transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 0,
		.data        = NULL,
	};

	/* Do the transfer */
	status = sensor->host_funcs->i2c_controller_write(caller_context, &transfer);
	if( status != tepht_status_ok)
		return false;

	return true;
}

static tepht_bool  ms5840_is_connected__virtual(void *self,  void *caller_context) {
	return ms5840_is_connected(self, caller_context);
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
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms5840_reset(ms5840_sensor *sensor,  void *caller_context)
{
	if ( sensor == NULL )
		return ms5840_error(sensor, tepht_status_null_sensor);

	tepht_status status;
	status = psensor_write_command(sensor, PSENSOR_RESET_COMMAND, caller_context);
	if( status != tepht_status_ok )
		return ms5840_error(sensor, status);

	return ms5840_success(sensor);
}

static tepht_error_info  ms5840_reset__virtual(void *self,  void *caller_context) {
	return ms5840_reset(self, caller_context);
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
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
tepht_error_info  ms5840_read_temperature_pressure_int32(ms5840_sensor *sensor, int32_t *t, int32_t *p,  void *caller_context)
{
	if ( sensor == NULL )
		return ms5840_error(sensor, tepht_status_null_sensor);

	if ( t == NULL || p == NULL )
		return ms5840_error(sensor, tepht_status_null_argument);

	tepht_error_info  einfo;
	einfo = psensor_read_pressure_and_temperature(sensor, t, p, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	return ms5840_success(sensor);
}

static tepht_error_info  ms5840_read_temperature_pressure_int32__virtual(void *self, int32_t *t, int32_t *p,  void *caller_context) {
	return ms5840_read_temperature_pressure_int32(self, t, p, caller_context);
}

/// \brief    Reads the temperature and pressure values.
///
/// \param[in] ms5840_sensor* sensor : The sensor to use for measuring
/// \param[out] float* : degC temperature value
/// \param[out] float* : mbar pressure value
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
tepht_error_info  ms5840_read_temperature_pressure_float32(ms5840_sensor *sensor, float *t, float *p, void *caller_context)
{
	tepht_error_info  einfo;
	int32_t t_;
	int32_t p_;

	// NULL status of the `sensor` parameter will be checked by `ms5840_read_temperature_pressure_i32`.

	if ( t == NULL || p == NULL )
		return ms5840_error(sensor, tepht_status_null_argument);

	einfo = ms5840_read_temperature_pressure_int32(sensor, &t_, &p_, caller_context);
	*t = ((float)t_) / 1000;
	*p = ((float)p_) / 1000;

	return einfo;
}

/******************** Functions from Pressure sensor ********************/

///
/// \brief Writes the Pressure Sensor 8-bits command with the value passed
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to send the command to
/// \param[in] uint8_t : Command value to be written.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
static tepht_status psensor_write_command(ms5840_sensor *sensor, uint8_t cmd, void *caller_context)
{
	assert( sensor != NULL );
	uint8_t data[1];

	data[0] = cmd;

	tepht_i2c_controller_packet  transfer = {
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
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///         (As of this writing, this function does not do any I2C I/O and
///         does not call any host functions, so `caller_context` is unused here,
///         but nonetheless provided for sake of future-proofing.)
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///
///
tepht_error_info  ms5840_set_pressure_resolution(ms5840_sensor *sensor, enum ms5840_pressure_resolution res, void *caller_context)
{
	(void)caller_context;
	if ( sensor == NULL )
		return ms5840_error(sensor, tepht_status_null_sensor);

	sensor->psensor_resolution_osr = res;
	return ms5840_success(sensor);
}

/// \brief Reads the psensor EEPROM coefficient stored at address provided.
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to retrieve EEPROM coefficients from
/// \param[in] uint8_t : Address of coefficient in EEPROM
/// \param[out] uint16_t* : Value read in EEPROM
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok : All operations completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///
static tepht_error_info  psensor_read_eeprom_coeff(ms5840_sensor *sensor, uint8_t command, uint16_t *coeff, void *caller_context)
{
	assert( sensor != NULL );
	assert( coeff != NULL );

	tepht_status status;
	uint8_t buffer[2];

	buffer[0] = 0;
	buffer[1] = 0;

	/* Read data */
	tepht_i2c_controller_packet  read_transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 2,
		.data        = buffer,
	};

	// Send the conversion command
	status = psensor_write_command(sensor, command, caller_context);
	if(status != tepht_status_ok)
		return ms5840_error(sensor, status);

	status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if(status != tepht_status_ok)
		return ms5840_error(sensor, status);

	*coeff = (buffer[0] << 8) | buffer[1];

	if (*coeff == 0)
		return ms5840_error(sensor, tepht_status_eeprom_is_zero);

	return ms5840_success(sensor);
}

/// \brief Reads the ms5840 EEPROM coefficients to store them for computation.
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to retrieve EEPROM coefficients from
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok : All operations completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///
static tepht_error_info  psensor_read_eeprom(ms5840_sensor *sensor, void *caller_context)
{
	assert( sensor != NULL );
	tepht_error_info  einfo;
	uint8_t i;

	for( i=0 ; i< MS5840_COEFFICIENT_COUNT ; i++)
	{
		einfo = psensor_read_eeprom_coeff(
			sensor, PROM_ADDRESS_READ_ADDRESS_0 + i*2, sensor->eeprom_coeff+i, caller_context);
		if( tepht_is_error(einfo) )
			return einfo;
	}

	if( !psensor_crc_check( sensor->eeprom_coeff, (sensor->eeprom_coeff[CRC_INDEX] & 0xF000)>>12 ) )
		return ms5840_error(sensor, tepht_status_eeprom_crc_error);

	sensor->psensor_coeff_read = true;

	return ms5840_success(sensor);
}

/// \brief Triggers conversion and reading of ADC value
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to read the adc from
/// \param[in] uint8_t : Command used for conversion (will determine Temperature vs Pressure and osr)
/// \param[out] uint32_t* : ADC value.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok : All operations completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_measurement_invalid : I2C transfer(s) completed, but data received was invalid
///
static tepht_error_info  psensor_conversion_and_read_adc(ms5840_sensor *sensor, uint8_t cmd, uint32_t *adc, void *caller_context)
{
	assert( sensor != NULL );
	assert( adc != NULL );

	tepht_status status;
	uint8_t buffer[3];

	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    tepht_i2c_controller_packet  read_transfer = {
		.address     = PSENSOR_ADDR,
		.data_length = 3,
		.data        = buffer,
	};

	// TODO: implement?
#if 0
	status = psensor_write_command(sensor, cmd, caller_context);
	if( status != tepht_status_ok)
		return status;

	while ( microsecs() < psensor_conversion_time[ (cmd & PSENSOR_CONVERSION_OSR_MASK)/2 ] )
		spin();

	// Send the read command
	status = psensor_write_command(sensor, PSENSOR_READ_ADC, caller_context);
	if( status != tepht_status_ok)
		return status;

    status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if( status != tepht_status_ok )
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
	if( status != tepht_status_ok)
		return ms5840_error(sensor, status);

	// Send the read command
	status = psensor_write_command(sensor, PSENSOR_READ_ADC, caller_context);
	if( status != tepht_status_ok)
		return ms5840_error(sensor, status);

    status = sensor->host_funcs->i2c_controller_read(caller_context, &read_transfer);
	if( status != tepht_status_ok )
		return ms5840_error(sensor, status);

	*adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

	return ms5840_success(sensor);
}

/// \brief Compute temperature and pressure
///
/// \param[in] ms5840_sensor *sensor : Object representing the sensor to read temperature and pressure from
/// \param[out] float* : Celsius Degree temperature value
/// \param[out] float* : mbar pressure value
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS5840
///       - tepht_status_ok : All operations completed successfully
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
static tepht_error_info  psensor_read_pressure_and_temperature(ms5840_sensor *sensor, int32_t *temperature, int32_t *pressure, void *caller_context)
{
	assert( sensor      != NULL );
	assert( temperature != NULL );
	assert( pressure    != NULL );

	tepht_error_info  einfo;
	uint32_t adc_temperature, adc_pressure;
	int64_t dT;
	int32_t TEMP;
	int64_t OFF, SENS, P, T2, OFF2, SENS2;
	uint8_t cmd;

	// If first time adc is requested, get EEPROM coefficients
	einfo = ms5840_success(sensor);
	if( sensor->psensor_coeff_read == false )
		einfo = psensor_read_eeprom(sensor, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	// First read temperature
	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_TEMPERATURE_ADC_CONVERSION;
	einfo = psensor_conversion_and_read_adc(sensor, cmd, &adc_temperature, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	// Now read pressure
	cmd = sensor->psensor_resolution_osr*2;
	cmd |= PSENSOR_START_PRESSURE_ADC_CONVERSION;
	einfo = psensor_conversion_and_read_adc(sensor, cmd, &adc_pressure, caller_context);
	if( tepht_is_error(einfo) )
		return einfo;

	if (adc_temperature == 0 || adc_pressure == 0)
		return ms5840_error(sensor, tepht_status_measurement_invalid);

	// Difference between actual and reference temperature = D2 - Tref
	dT = (int32_t)adc_temperature - ( (int32_t)sensor->eeprom_coeff[REFERENCE_TEMPERATURE_INDEX] <<8 );

	// Actual temperature = 2000 + dT * TEMPSENS
	TEMP = 2000 + ((int64_t)dT * (int64_t)sensor->eeprom_coeff[TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;

	// Second order temperature compensation
	if( TEMP < 2000 )
	{
		int64_t TEMP_MINUS_2000 = TEMP-2000;
		if ( TEMP < 1000 )
		{
			T2 = (((int64_t)12) * (dT*dT)) >> 35;
			OFF2 = (((int64_t)30) * (TEMP_MINUS_2000*TEMP_MINUS_2000)) >> 8;
			SENS2 = 0;
		}
		else
		{
			T2 = (((int64_t)14) * (dT*dT)) >> 35;
			OFF2 = (((int64_t)35) * (TEMP_MINUS_2000*TEMP_MINUS_2000)) >> 3;
			SENS2 = (((int64_t)63) * (TEMP_MINUS_2000*TEMP_MINUS_2000)) >> 5;
		}
	}
	else
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
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

	return ms5840_success(sensor);
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


static tepht_pt_sensor_vtable  ms5840_pt_sensor_vtable =
	{
		.is_connected                    = ms5840_is_connected__virtual,
		.reset                           = ms5840_reset__virtual,
		.read_temperature_pressure_int32 = ms5840_read_temperature_pressure_int32__virtual
	};

tepht_pt_sensor   ms5840_to_pt_sensor_interface(ms5840_sensor *sensor)
{
	tepht_pt_sensor  generic;
	assert(sensor != NULL);
	generic.self   = sensor;
	generic.vtable = &ms5840_pt_sensor_vtable;
	return generic;
}

#ifdef __cplusplus
}
#endif
