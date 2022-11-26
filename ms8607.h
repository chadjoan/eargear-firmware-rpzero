///
/// \file ms8607.h
///
/// \brief    ms8607 Temperature, pressure and humidity sensor driver header file
///
/// \details  This module uses no global mutable variables, whether process-local,
///           thread-local, or otherwise. This means that all functions are
///           reentrant and thread-safe to some degree, with specifics depending
///           on the parameters involved and the properties of callbacks passed
///           into these functions.
///
///           All functions in this file that manipulate `ms8607_sensor`
///           objects are thread-safe as long as any given `ms8607_sensor`
///           object is never accessed by more than one thread at a time.
///           This simply correlates with the non-sharable nature of indivisible
///           physical resources, such as a sensor on an I2C bus.
///
///           In other words:
///
///           One-sensor-per-thread = OK.
///           Multiple-sensors-per-thread = OK.
///           Multiple-threads-per-sensor = Not OK. Bad. Don't do it.
///
///
/// Copyright (c) 2016 Measurement Specialties. All rights reserved.

#ifndef MS8607_H_INCLUDED
#define MS8607_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "tepht_common.h"

// Macros

#define MS8607_COEFFICIENT_COUNT  (7)

// Enums

enum ms8607_humidity_i2c_controller_mode {
	ms8607_i2c_hold,
	ms8607_i2c_no_hold
};

enum ms8607_humidity_resolution {
	ms8607_humidity_resolution_12b = 0,
	ms8607_humidity_resolution_8b,
	ms8607_humidity_resolution_10b,
	ms8607_humidity_resolution_11b
};

enum ms8607_battery_status {
	ms8607_battery_ok,
	ms8607_battery_low
};

enum ms8607_heater_status {
	ms8607_heater_off,
	ms8607_heater_on
};

enum ms8607_pressure_resolution {
	ms8607_pressure_resolution_osr_256 = 0,
	ms8607_pressure_resolution_osr_512,
	ms8607_pressure_resolution_osr_1024,
	ms8607_pressure_resolution_osr_2048,
	ms8607_pressure_resolution_osr_4096,
	ms8607_pressure_resolution_osr_8192
};

// TODO: probable in-order:
// - Change how tepht_host_functions initializes ##done
// - Replace comment style with triple-slash ##done
// - Add i2c_nack error type. (Required for polling interface. Otherwise optional.) ##done?
// - Test i2c_nack and polling logic by using it instead of sleeping.
// - Remove global state ##wip - missed a spot or two, see TODOs
// - Add typedefs to all enums.
// - Struct-ize the API
// - Undo commenting-out of "static" attribute on some functions

// Basic types

/// \brief   Boolean type with predictable storage size.
///
/// \details This type is used instead of <stdbool.h>'s `bool` type, as the `bool`
///          type is not required to occupy exactly one byte (it can be larger!).
///          This type is an alias of `uint8_t`, and therefore occupies exactly
///          one byte, always. This is done to ensure that bindings for non-C
///          languages will have a way to know the width of the ms8607's booleans
///          whenever they appear in a struct or as a function parameter.
///
typedef uint8_t  ms8607_bool;

// Structs

/// \brief   Structure that tracks the configuration and status of an MS8607 sensor.
///
/// \details The members of this structure are all considered "internal" or "private"
///          and should not be accessed or modified. The only reason these
///          members have public visibility (in the header file) is to
///          allow calling code to know the size of the struct (ex: `sizeof(ms8607_sensor)`).
///          This is what allows variables to be declared with this struct
///          as their type, which then allows the caller to place the struct
///          wherever they consider appropriate, including on the stack
///          (by declaring an `ms8607_sensor` variable in function scope)
///          or in global storage (by declaring a `ms8607_sensor` variable
///          outside of functions, or as a "static" variable inside functions).
///
typedef struct ms8607_sensor {
	// Internal state. Avoid accessing directly.
	const tepht_host_functions                 *host_funcs;
	const tepht_driver_context_accessor        context_accessor;
	uint32_t                                   hsensor_conversion_time;
	enum ms8607_humidity_i2c_controller_mode   hsensor_i2c_controller_mode;
	enum ms8607_pressure_resolution            psensor_resolution_osr;
	ms8607_bool                                hsensor_heater_on;
	ms8607_bool                                psensor_coeff_read;
	uint16_t                                   eeprom_coeff[MS8607_COEFFICIENT_COUNT+1];
} ms8607_sensor;


// Functions

// TODO: Ensure that `ms8607_reset` always gets called, even if the caller skips it.
// (e.g., by calling it lazily from whatever might need it).

/// \brief    Initializes a new `ms8607_sensor` object.
///
/// \details  This function's purpose is to place the given `new_sensor`
///           argument into a predictable state and provide it with some
///           default configuration. This function does not actually communicate
///           with the sensor in any way.
///
///           This must be called after `tepht_init_and_assign_host_functions`
///           has been called at least once (to obtain the argument for the
///           `depends_to_use` parameter), and it must be called before any
///           other function that requires a `ms8607_sensor*` type parameter
///           (usually `ms8607_reset`).
///
///           This function is reentrant, idempotent, non-blocking,
///           and does not perform any I/O. This function is thread-safe
///           as long as, during this function's execution, no other threads
///           read from or write to that call's `new_sensor` instance, and
///           no other threads write to that call's `depends_to_use` instance.
///
/// \param[out] ms8607_sensor *new_sensor : The new sensor object.
/// \param[in]  const tepht_host_functions *depends_to_use : Specifies the
///           functions that this sensor can call to do things such as I2C I/O
///           and timing.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : Sensor object was initailized successfully
///       - tepht_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - tepht_status_null_argument : The pointer provided for the `depends_to_use` parameter was NULL.
///       - tepht_status_i2c_read_unimplemented : Returned if the `i2c_controller_read` function was not assigned.
///       - tepht_status_i2c_write_unimplemented : Returned if the `i2c_controller_write` function was not assigned.
///       - tepht_status_sleep_ms_unimplemented : Returned if the `sleep_ms` function was not assigned.
///
tepht_error_info  ms8607_init_sensor(ms8607_sensor *new_sensor,  tepht_host_functions *depends_to_use);

/// \brief Check whether MS8607 device is connected
///
/// \param[in] ms8607_sensor *sensor : Sensor object to test for connectivity.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return bool : status of MS8607
///       - true : Device is present
///       - false : Device is not acknowledging I2C address
///
tepht_bool  ms8607_is_connected(ms8607_sensor *sensor,  void *caller_context);

/// \brief Reset the MS8607 device
///
/// \details
///           This function is reentrant. It blocks and performs I2C I/O.
///           It is thread-safe as long as, during this function's execution,
///           no other threads read from or write to the given `sensor` instance.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to be reset.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_reset(ms8607_sensor *sensor,  void *caller_context);

/// \brief Set humidity ADC resolution.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to set humidity resolution on.
/// \param[in] ms8607_humidity_resolution : Resolution requested
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_set_humidity_resolution(ms8607_sensor *sensor, enum ms8607_humidity_resolution, void *caller_context);

/// \brief Set pressure ADC resolution.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to configure
/// \param[in] ms8607_pressure_resolution : Resolution requested
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///         (As of this writing, this function does not do any I2C I/O and
///         does not call any host functions, so `caller_context` is unused here,
///         but nonetheless provided for sake of future-proofing.)
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///
///
tepht_error_info  ms8607_set_pressure_resolution(ms8607_sensor *sensor, enum ms8607_pressure_resolution, void *caller_context);

/// \brief Set Humidity sensor ADC resolution.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to set controller mode on.
/// \param[in] ms8607_i2c_controller_mode : I2C mode
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///         (As of this writing, this function does not do any I2C I/O and
///         does not call any host functions, so `caller_context` is unused here,
///         but nonetheless provided for sake of future-proofing.)
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///
tepht_error_info  ms8607_set_humidity_i2c_controller_mode(ms8607_sensor *sensor, enum ms8607_humidity_i2c_controller_mode, void *caller_context);

/// \brief    Reads the temperature, pressure and relative humidity values.
///
/// \details  The results are returned in thousanths as a way to preserve
///           the sensor's resolution while using integer types.
///
/// \param[in] ms8607_sensor* sensor : The sensor to use for measuring
/// \param[out] int32_t* : Thousanths of degC temperature value
/// \param[out] int32_t* : Microbar pressure value (thousanths of millibar)
/// \param[out] int32_t* : Thousanths of %RH Relative Humidity value
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
tepht_error_info  ms8607_read_temperature_pressure_humidity_int32(ms8607_sensor *sensor, int32_t *, int32_t *, int32_t *, void *caller_context);


/// \brief    Reads the temperature and pressure values.
///
/// \details  The results are returned in thousanths as a way to preserve
///           the sensor's resolution while using integer types.
///
/// \param[in] ms8607_sensor* sensor : The sensor to use for measuring
/// \param[out] int32_t* : Thousanths of degC temperature value
/// \param[out] int32_t* : Microbar pressure value (thousanths of millibar)
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_null_argument : One or more of the `t` or `p` pointers were NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
tepht_error_info  ms8607_read_temperature_pressure_int32(ms8607_sensor *sensor, int32_t *t, int32_t *p, void *caller_context);

/// \brief Reads the temperature, pressure and relative humidity values.
///
/// \param[in] ms8607_sensor* sensor : The sensor to use for measuring
/// \param[out] float* : degC temperature value
/// \param[out] float* : mbar pressure value
/// \param[out] float* : %RH Relative Humidity value
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///       - tepht_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - tepht_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - tepht_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
tepht_error_info  ms8607_read_temperature_pressure_humidity_float32(ms8607_sensor *sensor, float *, float *, float *, void *caller_context);

/// \brief Provide battery status
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to get battery status from
/// \param[out] ms8607_battery_status* : Battery status
///                      - ms8607_battery_ok,
///                      - ms8607_battery_low
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_get_battery_status(ms8607_sensor *sensor, enum ms8607_battery_status*, void *caller_context);

/// \brief Enable heater
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor whose heater shall be enabled
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_enable_heater(ms8607_sensor *sensor, void *caller_context);

/// \brief Disable heater
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor whose heater shall be disabled
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_disable_heater(ms8607_sensor *sensor, void *caller_context);

/// \brief Get heater status
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor  to get heater status from
/// \param[in] ms8607_heater_status* : Return heater status (above or below 2.5V)
///                      - ms8607_heater_off,
///                      - ms8607_heater_on
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_get_heater_status(ms8607_sensor *sensor, enum ms8607_heater_status*, void *caller_context);

/// \brief Returns result of compensated humidity
///        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor that earlier obtained the temperature and uncompensated %RH
/// \param[in] float - Actual temperature measured (degC)
/// \param[in] float - Actual relative humidity measured (%RH)
/// \param[out] float *- Compensated humidity (%RH).
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_heater_on_error : Cannot compute compensated humidity because heater is on
///
tepht_error_info  ms8607_get_compensated_humidity(ms8607_sensor *sensor, float, float, float*);

/// \brief Returns the computed dew point
///        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor that earlier obtained the temperature and uncompensated %RH
/// \param[in] float - Actual temperature measured (degC)
/// \param[in] float - Actual relative humidity measured (%RH)
/// \param[out] float *- Dew point temperature (DegC).
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_heater_on_error : Cannot compute compensated humidity because heater is on
///
tepht_error_info  ms8607_get_dew_point(ms8607_sensor *sensor, float, float, float*);

#if 0
// TODO: remove?
tepht_status hsensor_poll_relative_humidity(ms8607_sensor *sensor, float *humidity, void *caller_context);
tepht_status hsensor_write_command(ms8607_sensor *sensor, uint8_t, void *caller_context);
tepht_status psensor_request_temperature(ms8607_sensor *sensor, uint64_t microsecs, void *caller_context);
tepht_status psensor_poll_raw_temperature(ms8607_sensor *sensor, uint64_t microsecs, uint32_t *temperature, void *caller_context);
tepht_status psensor_request_pressure(ms8607_sensor *sensor, uint64_t microsecs, void *caller_context);
tepht_status psensor_poll_raw_pressure(ms8607_sensor *sensor, uint64_t microsecs, uint32_t *pressure, void *caller_context);
#endif

/// \brief   Reads the MS8607 humidity user register.
///
/// \details The user register is described in the datasheet.
///          It is unnecessary to call this function to use the ms8607 sensor,
///          but it could be useful as a diagnostic tool or for better
///          understanding the behavior of the ms8607.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to read the user register from
/// \param[out] uint8_t* : Storage of user register value
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `tepht_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return tepht_status : status of MS8607
///       - tepht_status_ok : I2C transfer completed successfully
///       - tepht_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - tepht_status_callback_error : Error occurred within a tepht_host_functions function
///
tepht_error_info  ms8607_hsensor_read_user_register(ms8607_sensor *sensor, uint8_t *value, void *caller_context);

#endif /* MS8607_H_INCLUDED */
