///
/// \file ms5840.h
///
/// \brief    ms5840 temperature and pressure sensor driver header file
///
/// \details  This module uses no global mutable variables, whether process-local,
///           thread-local, or otherwise. This means that all functions are
///           reentrant and thread-safe to some degree, with specifics depending
///           on the parameters involved and the properties of callbacks passed
///           into these functions.
///
///           All functions in this file that manipulate `ms5840_sensor`
///           objects are thread-safe as long as any given `ms5840_sensor`
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

#ifndef MS5840_H_INCLUDED
#define MS5840_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "tepht_common.h"

// Macros

#define MS5840_COEFFICIENT_COUNT  (7)

// Enums

enum ms5840_pressure_resolution {
	ms5840_pressure_resolution_osr_256 = 0,
	ms5840_pressure_resolution_osr_512,
	ms5840_pressure_resolution_osr_1024,
	ms5840_pressure_resolution_osr_2048,
	ms5840_pressure_resolution_osr_4096,
	ms5840_pressure_resolution_osr_8192
};

// TODO: probable in-order:
// - Change how ms5840_host_functions initializes ##done
// - Replace comment style with triple-slash ##done
// - Add i2c_nack error type. (Required for polling interface. Otherwise optional.) ##done?
// - Test i2c_nack and polling logic by using it instead of sleeping.
// - Remove global state ##wip - missed a spot or two, see TODOs
// - Add typedefs to all enums.
// - Struct-ize the API
// - Undo commenting-out of "static" attribute on some functions

// Structs

/// \brief   Structure that tracks the configuration and status of an MS5840 sensor.
///
/// \details The members of this structure are all considered "internal" or "private"
///          and should not be accessed or modified. The only reason these
///          members have public visibility (in the header file) is to
///          allow calling code to know the size of the struct (ex: `sizeof(ms5840_sensor)`).
///          This is what allows variables to be declared with this struct
///          as their type, which then allows the caller to place the struct
///          wherever they consider appropriate, including on the stack
///          (by declaring an `ms5840_sensor` variable in function scope)
///          or in global storage (by declaring a `ms5840_sensor` variable
///          outside of functions, or as a "static" variable inside functions).
///
typedef struct ms5840_sensor {
	// Internal state. Avoid accessing directly.
	const tepht_host_functions                 *host_funcs;
	const tepht_driver_context_accessor        context_accessor;
	enum ms5840_pressure_resolution            psensor_resolution_osr;
	tepht_bool                                 psensor_coeff_read;
	uint16_t                                   eeprom_coeff[MS5840_COEFFICIENT_COUNT+1];
} ms5840_sensor;


// Functions

// TODO: Ensure that `ms5840_reset` always gets called, even if the caller skips it.
// (e.g., by calling it lazily from whatever might need it).

/// \brief    Initializes a new `ms5840_sensor` object.
///
/// \details  This function's purpose is to place the given `new_sensor`
///           argument into a predictable state and provide it with some
///           default configuration. This function does not actually communicate
///           with the sensor in any way.
///
///           This must be called after `tepht_init_and_assign_host_functions`
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
tepht_error_info  ms5840_init_sensor(ms5840_sensor *new_sensor,  tepht_host_functions *depends_to_use);

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
tepht_bool  ms5840_is_connected(ms5840_sensor *sensor,  void *caller_context);

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
tepht_error_info  ms5840_reset(ms5840_sensor *sensor,  void *caller_context);

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
tepht_error_info  ms5840_set_pressure_resolution(ms5840_sensor *sensor, enum ms5840_pressure_resolution, void *caller_context);

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
tepht_error_info  ms5840_read_temperature_pressure_int32(ms5840_sensor *sensor, int32_t *, int32_t *, void *caller_context);

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
tepht_error_info  ms5840_read_temperature_pressure_float32(ms5840_sensor *sensor, float *, float *, void *caller_context);

/// Acquires a generic PT sensor (pressure and temperature sensor) reference
/// that allows the sensor to be used in a way that is interchangable with
/// other sensors that offer the same <tepht_pt_sensor> interface.
///
tepht_pt_sensor   ms5840_to_pt_sensor_interface(ms5840_sensor *sensor);

#endif /* MS5840_H_INCLUDED */
