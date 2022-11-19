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

// Macros

#define MS8607_COEFFICIENT_COUNT  (7)

// Enums

enum ms8607_humidity_i2c_controller_mode {
	ms8607_i2c_hold,
	ms8607_i2c_no_hold
};

enum ms8607_status {
	ms8607_status_ok = 0,
	ms8607_status_waiting, // TODO: document, error handle, etc.
	ms8607_status_null_argument,
	ms8607_status_null_sensor,
	ms8607_status_null_host_function,
	ms8607_status_callback_error,
	ms8607_status_callback_i2c_nack,
	ms8607_status_eeprom_is_zero, // Formerly ms8607_status_crc_error
	ms8607_status_eeprom_crc_error, // Formerly ms8607_status_crc_error
	ms8607_status_measurement_invalid, // Formerly ms8607_status_i2c_transfer_error
	// TODO: ms8607_status_response_timeout, // I2C requests result in NACK even though the sensor should have responded by now. Possible driver state-machine desync.
	ms8607_status_heater_on_error,
	ms8607_status_i2c_read_unimplemented,
	ms8607_status_i2c_write_unimplemented,
	ms8607_status_i2c_write_no_stop_unimplemented,
	ms8607_status_sleep_ms_unimplemented
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
// - Change how ms8607_host_functions initializes ##done
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

///
/// \brief  This structure is used by the MS8607 driver to specify I2C transfers
///         for the callbacks that implement I2C functionality for the driver.
///
typedef struct ms8607_i2c_controller_packet {
	/// \brief Address to peripheral device
	uint16_t address;

	/// \brief Length of data array
	uint16_t data_length;

	/// \brief Data array containing all data to be transferred
	uint8_t *data;
} ms8607_i2c_controller_packet;

///
/// \brief    This structure allows the caller to provide implementations for the
///           MS8607 driver's dependencies, which are mostly I2C functionality.
///
/// \details  Some patterns are employed to keep the driver and callbacks
///           flexible and thread-safe:
///    - A `caller_context` pointer is passed from the caller into the MS8607
///        driver, and then from the MS8607 driver into these callbacks. This is
///        what allows the caller and their callbacks to communicate and persist
///        data accross calls into the driver (and to do so without relying on
///        global variables or thread-local-storage).
///    - The return value is usually the `ms8607_status` enum, but only one of
///        two possible enum values shall be returned from the callback:
///        `ms8607_status_ok` and `ms8607_status_callback_error`.
///    - Returning `ms8607_status_ok` indicates that the callback completed its
///        operation (usually an I2C transaction) successfully. This tells the
///        driver that it can continue working.
///    - Returning `ms8607_status_callback_error` indicates that something
///        went wrong. The driver will typically return from its own function
///        immediately after receiving this error code from a callback. This
///        is intended primarily as a way to provide the driver with "go / no-go"
///        information, and nothing more specific than that. If the caller needs
///        to return more specific error details from a failed callback,
///        the `caller_context` argument should be used for this purpose.
///
typedef struct ms8607_host_functions {
	/// \brief  Internal state; do not modify.
	uint8_t  validated_;

	/// \brief   Callback that shall implement I2C packet reading (receive|rx) functionality.
	///
	/// \details This callback (caller-supplied function) shall read an I2C packet
	///          from the I2C bus that the MS8607 sensor is connected to.
	///          This allows the MS8607 driver to complete I2C transfers without
	///          directly depending on any one specific I2C implementation.
	///
	///          This callback is required: the MS8607 driver will be unable to
	///          retrieve readings from the MS8607 sensor without it.
	///
	///          If the polling interface is used (TODO: call out the function name)
	///          then this function must distinguish between I2C NACK responses
	///          (by returning `ms8607_status_callback_i2c_nack`) and all other possibly
	///          negative results from I2C transfers (by returning `ms8607_status_callback_error`).
	///          If this behavior is not implemented while the polling interface
	///          is in use, then the polling interface will always report failure.
	///          This is necessary for polling because the MS8607 uses a NACK
	///          response to indicate that the controller must wait a litte bit
	///          longer before a measurement result becomes available.
	///
	/// \return  ms8607_status : Lets the driver know if the I2C transmit was successful.
	///        - ms8607_status_ok : I2C transfer completed successfully
	///        - ms8607_status_callback_error : Problem with i2c transfer
	///        - ms8607_status_callback_i2c_nack : I2C peripheral responded with NACK
	///
	enum ms8607_status  (*i2c_controller_read)(void *caller_context, ms8607_i2c_controller_packet *const);

	/// \brief   Callback that shall implement I2C packet writing (trasmit|tx) functionality.
	///
	/// \details This callback (caller-supplied function) shall write an I2C packet
	///          to the I2C bus that the MS8607 sensor is connected to.
	///          This allows the MS8607 driver to complete I2C transfers without
	///          directly depending on any one specific I2C implementation.
	///
	///          This callback is required: the MS8607 driver will be unable to
	///          retrieve readings from the MS8607 sensor without it.
	///
	/// \return  ms8607_status : Lets the driver know if the I2C transmit was successful.
	///        - ms8607_status_ok : I2C transfer completed successfully
	///        - ms8607_status_callback_error : Problem with i2c transfer
	///
	enum ms8607_status  (*i2c_controller_write)(void *caller_context, ms8607_i2c_controller_packet *const);

	/// \brief   Callback that shall implement I2C packet writing (trasmit|tx) functionality,
	///          but does not transmit a "stop" bit at the end of the I2C packet.
	///
	/// \details This callback (caller-supplied function) shall write an I2C packet
	///          to the I2C bus that the MS8607 sensor is connected to.
	///          Unlike the `i2c_controller_write_packet`  function, this version
	///          shall NOT write a "stop" bit at the end of the packet.
	///
	///          If the caller's I2C implementation is not capable of this,
	///          then it is recommended that the caller provide a stub function
	///          that returns `ms8607_status_callback_error`. The caller
	///          should then prevent any use of the MS8607's "hold" mode.
	///
	///          This callback is optional: the MS8607 driver can retrieve
	///          readings from the MS8607 sensor without it, but it becomes
	///          necessary if the sensor is used in "hold" mode
	///          (`ms8607_humidity_i2c_controller_mode : ms8607_i2c_hold`).
	///
	/// \return  ms8607_status : Lets the driver know if the I2C transmit was successful.
	///        - ms8607_status_ok : I2C transfer completed successfully
	///        - ms8607_status_callback_error : Problem with i2c transfer
	///
	enum ms8607_status  (*i2c_controller_write_no_stop)(void *caller_context, ms8607_i2c_controller_packet *const);

	/// \brief   Callback that shall wait for the given number of milliseconds when called.
	///
	/// \details If the caller is operating in a multi-threaded environment
	///          (including software-based schedulers running on single-threaded
	///          processors), then it is perfectly acceptable to yield this time
	///          to other threads or fibers.
	///
	enum ms8607_status  (*sleep_ms)(void *caller_context, uint32_t milliseconds);

	/// \brief  Optional callback that is used to print, report, or log errors or diagnostic messages.
	void  (*print_string)(void *caller_context, const char *text);

	/// \brief  Optional callback that is used to print, report, or log errors or diagnostic messages.
	void  (*print_int64)(void *caller_context,  int64_t  number, uint8_t pad_width,  ms8607_bool  pad_with_zeroes);

} ms8607_host_functions;


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
	const ms8607_host_functions                *host_funcs;
	uint32_t                                   hsensor_conversion_time;
	enum ms8607_humidity_i2c_controller_mode   hsensor_i2c_controller_mode;
	enum ms8607_pressure_resolution            psensor_resolution_osr;
	ms8607_bool                                hsensor_heater_on;
	ms8607_bool                                psensor_coeff_read;
	uint16_t                                   eeprom_coeff[MS8607_COEFFICIENT_COUNT+1];
} ms8607_sensor;


// Functions

/// \brief    Creates a `ms8607_host_functions` object (the `dependencies` parameter)
///           to store function pointers that implement the driver's dependencies.
///
/// \details  The purpose of this function is to create a `ms8607_host_functions`
///           object, which can then be used by the MS8607 driver to satisfy
///           its dependencies.
///
///           This must be called before calling the `ms8607_init_sensor`
///           function, as the `ms8607_init_sensor` function requires the
///           `ms8607_host_functions` object that is populated by this function.
///
///           This function works in 3 steps:
///
///           (1) It initializes the `ms8607_host_functions` object
///             given by the `dependencies` parameter. This places the object
///             into a known state so that the 3rd phase of this function can
///             know which functions were implemented by the caller/host.
///
///           (2) It calls the given `assign_functions` callback on `dependencies`.
///             The callback should create function pointers from host functions
///             that implement the various requirements of the MS8607 driver,
///             such as I2C I/O and timing mechanisms. Those function pointers
///             should be assigned to the various members of the `dependencies`
///             structure. See the `ms8607_host_functions` for details on the
///             necessary functions.
///
///           (3) After the callback returns, this function then validates
///             the resulting `ms8607_host_functions` object to ensure that
///             minimal requirements are met. Appropriate error codes are
///             returned if the driver's dependencies were not satisfied.
///
///           The `assign_functions` callback shall NOT assign NULL to any
///           members of the `ms8607_host_functions *dependencies` structure.
///           When a function pointer is optional and no implementation
///           is available, `assign_functions` should leave that member
///           unmodified.
///
///           The `ms8607_init_and_assign_host_functions` function will
///           have already assigned stubs (and missing requirement detectors)
///           to the members of the `ms8607_host_functions` structure.
///
///           This function is reentrant, idempotent, non-blocking,
///           and does not perform any I/O. These properties assume that
///           the `assign_functions` callback also possesses the same
///           corresponding properties. This function is thread-safe
///           as long as, during this function's execution, no other threads
///           read from or write to the objects pointed to by this function's
///           arguments.
///
/// \param[out] ms8607_host_functions* : Struct with callbacks that implement I2C controller functions.
/// \param[in] void* caller_context : This is passed to the `assign_functions`
///           callback's `caller_context` parameter.
/// \param[in] void (*assign_functions)(ms8607_host_functions *dependencies, void *caller_context):
///           Pointer to a caller-implemented function that shall assign pointers
///           to implementation functions that are required by the driver.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_null_argument : Returned if the `dependencies` or `assign_functions` parameters were NULL.
///       - ms8607_status_null_host_function : Returned if NULL was assigned to any of the members of `dependencies`.
///       - ms8607_status_i2c_read_unimplemented : Returned if the `i2c_controller_read` function was not assigned.
///       - ms8607_status_i2c_write_unimplemented : Returned if the `i2c_controller_write` function was not assigned.
///       - ms8607_status_sleep_ms_unimplemented : Returned if the `sleep_ms` function was not assigned.
///
enum ms8607_status  ms8607_init_and_assign_host_functions(
	ms8607_host_functions *deps,
	void *caller_context,
	void (*assign_functions)(ms8607_host_functions *deps, void *caller_context)
	);

// TODO: Ensure that `ms8607_reset` always gets called, even if the caller skips it.
// (e.g., by calling it lazily from whatever might need it).

/// \brief    Initializes a new `ms8607_sensor` object.
///
/// \details  This function's purpose is to place the given `new_sensor`
///           argument into a predictable state and provide it with some
///           default configuration. This function does not actually communicate
///           with the sensor in any way.
///
///           This must be called after `ms8607_init_and_assign_host_functions`
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
/// \param[in]  const ms8607_host_functions *depends_to_use : Specifies the
///           functions that this sensor can call to do things such as I2C I/O
///           and timing.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : Sensor object was initailized successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_null_argument : The pointer provided for the `depends_to_use` parameter was NULL.
///       - ms8607_status_i2c_read_unimplemented : Returned if the `i2c_controller_read` function was not assigned.
///       - ms8607_status_i2c_write_unimplemented : Returned if the `i2c_controller_write` function was not assigned.
///       - ms8607_status_sleep_ms_unimplemented : Returned if the `sleep_ms` function was not assigned.
///
enum ms8607_status  ms8607_init_sensor(ms8607_sensor *new_sensor,  ms8607_host_functions *depends_to_use);

/// \brief Check whether MS8607 device is connected
///
/// \param[in] ms8607_sensor *sensor : Sensor object to test for connectivity.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return bool : status of MS8607
///       - true : Device is present
///       - false : Device is not acknowledging I2C address
///
bool ms8607_is_connected(ms8607_sensor *sensor,  void *caller_context);

/// \brief Reset the MS8607 device
///
/// \details
///           This function is reentrant. It blocks and performs I2C I/O.
///           It is thread-safe as long as, during this function's execution,
///           no other threads read from or write to the given `sensor` instance.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to be reset.
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - ms8607_status_callback_error : Error occurred within a ms8607_host_functions function
///
enum ms8607_status ms8607_reset(ms8607_sensor *sensor,  void *caller_context);

/// \brief Set humidity ADC resolution.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to set humidity resolution on.
/// \param[in] ms8607_humidity_resolution : Resolution requested
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///       - ms8607_status_callback_error : Error occurred within a ms8607_host_functions function
///
enum ms8607_status ms8607_set_humidity_resolution(ms8607_sensor *sensor, enum ms8607_humidity_resolution, void *caller_context);

/// \brief Set pressure ADC resolution.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to configure
/// \param[in] ms8607_pressure_resolution : Resolution requested
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///         (As of this writing, this function does not do any I2C I/O and
///         does not call any host functions, so `caller_context` is unused here,
///         but nonetheless provided for sake of future-proofing.)
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok
///       - ms8607_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///
///
enum ms8607_status ms8607_set_pressure_resolution(ms8607_sensor *sensor, enum ms8607_pressure_resolution, void *caller_context);

/// \brief Set Humidity sensor ADC resolution.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to set controller mode on.
/// \param[in] ms8607_i2c_controller_mode : I2C mode
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///         (As of this writing, this function does not do any I2C I/O and
///         does not call any host functions, so `caller_context` is unused here,
///         but nonetheless provided for sake of future-proofing.)
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok
///       - ms8607_status_null_sensor : The pointer provided for the `sensor` parameter was NULL.
///
enum ms8607_status ms8607_set_humidity_i2c_controller_mode(ms8607_sensor *sensor, enum ms8607_humidity_i2c_controller_mode, void *caller_context);

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
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - ms8607_status_callback_error : Error occurred within a ms8607_host_functions function
///       - ms8607_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - ms8607_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - ms8607_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
enum ms8607_status ms8607_read_temperature_pressure_humidity_int32(ms8607_sensor *sensor, int32_t *, int32_t *, int32_t *, void *caller_context);

/// \brief Reads the temperature, pressure and relative humidity values.
///
/// \param[in] ms8607_sensor* sensor : The sensor to use for measuring
/// \param[out] float* : degC temperature value
/// \param[out] float* : mbar pressure value
/// \param[out] float* : %RH Relative Humidity value
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_null_argument : One or more of the `t`, `p`, or `h` pointers were NULL.
///       - ms8607_status_callback_error : Error occurred within a ms8607_host_functions function
///       - ms8607_status_eeprom_is_zero : One or more EEPROM coefficients were received as 0, preventing measurement.
///       - ms8607_status_eeprom_crc_error : CRC check error on the sensor's EEPROM coefficients
///       - ms8607_status_measurement_invalid : EEPROM is OK and I2C transfer completed, but data received was invalid
///
enum ms8607_status ms8607_read_temperature_pressure_humidity_float32(ms8607_sensor *sensor, float *, float *, float *, void *caller_context);

/// \brief Provide battery status
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor to get battery status from
/// \param[out] ms8607_battery_status* : Battery status
///                      - ms8607_battery_ok,
///                      - ms8607_battery_low
/// \param[in]  void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_callback_error : Error occurred within a ms8607_host_functions function
///
enum ms8607_status ms8607_get_battery_status(ms8607_sensor *sensor, enum ms8607_battery_status*, void *caller_context);

/// \brief Enable heater
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor whose heater shall be enabled
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_callback_error : Error occurred within a ms8607_host_functions function
///
enum ms8607_status ms8607_enable_heater(ms8607_sensor *sensor, void *caller_context);

/// \brief Disable heater
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor whose heater shall be disabled
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_callback_error : Error occurred within a ms8607_host_functions function
///
enum ms8607_status ms8607_disable_heater(ms8607_sensor *sensor, void *caller_context);

/// \brief Get heater status
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor  to get heater status from
/// \param[in] ms8607_heater_status* : Return heater status (above or below 2.5V)
///                      - ms8607_heater_off,
///                      - ms8607_heater_on
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_callback_error : Error occurred within a ms8607_host_functions function
///
enum ms8607_status ms8607_get_heater_status(ms8607_sensor *sensor, enum ms8607_heater_status*, void *caller_context);

/// \brief Returns result of compensated humidity
///        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor that earlier obtained the temperature and uncompensated %RH
/// \param[in] float - Actual temperature measured (degC)
/// \param[in] float - Actual relative humidity measured (%RH)
/// \param[out] float *- Compensated humidity (%RH).
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_heater_on_error : Cannot compute compensated humidity because heater is on
///
enum ms8607_status ms8607_get_compensated_humidity(ms8607_sensor *sensor, float, float, float*);

/// \brief Returns the computed dew point
///        Note : This function shall only be used when the heater is OFF. It will return an error otherwise.
///
/// \param[in] ms8607_sensor *sensor : Object representing the sensor that earlier obtained the temperature and uncompensated %RH
/// \param[in] float - Actual temperature measured (degC)
/// \param[in] float - Actual relative humidity measured (%RH)
/// \param[out] float *- Dew point temperature (DegC).
/// \param[in] void* caller_context : When this function calls any callbacks
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_heater_on_error : Cannot compute compensated humidity because heater is on
///
enum ms8607_status ms8607_get_dew_point(ms8607_sensor *sensor, float, float, float*);

/// \brief   Returns a string describing the given error code.
///
/// \details The caller should filter out any cases where `error_code` equals
///          `ms8607_status_callback_error`. The "error within callback"
///          return value is not returned from the ms8607 driver itself, so the
///          ms8607 driver has no way to know what caused the error.
///          This function will still return a valid string constant in those
///          cases, but it will a very generic message and thus won't be
///          specific enough for conclusive troubleshooting.
///
///          The `ms8607_status_callback_error` code is intended to be returned
///          from function pointers (callbacks) provided to the `ms8607_host_functions`
///          structure. Since the caller provides these callbacks, it is
///          the caller's responsibility to either report such errors from within
///          the callbacks, or to track that error information separately
///          (ex: by passing a custom struct pointer as the `caller_context` parameter,
///          then using such a struct to persist more specific error information
///          past the ms8607 driver function's return).
///
///          This function is reentrant, thread-safe, non-blocking, and pure
///          (it does not perform any I/O or have any side effects).
///
/// \return  A non-NULL string describing the given error code.
///          If the error code is not valid for any reason, the returned string
///          will simply indicate that the error code was not valid.
///
const char *ms8607_stringize_error(enum ms8607_status error_code);

#if 0
// TODO: remove?
enum ms8607_status hsensor_poll_relative_humidity(ms8607_sensor *sensor, float *humidity, void *caller_context);
enum ms8607_status hsensor_write_command(ms8607_sensor *sensor, uint8_t, void *caller_context);
enum ms8607_status psensor_request_temperature(ms8607_sensor *sensor, uint64_t microsecs, void *caller_context);
enum ms8607_status psensor_poll_raw_temperature(ms8607_sensor *sensor, uint64_t microsecs, uint32_t *temperature, void *caller_context);
enum ms8607_status psensor_request_pressure(ms8607_sensor *sensor, uint64_t microsecs, void *caller_context);
enum ms8607_status psensor_poll_raw_pressure(ms8607_sensor *sensor, uint64_t microsecs, uint32_t *pressure, void *caller_context);
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
///         from the `ms8607_host_functions` structure, this will be passed
///         directly to those callbacks' `caller_context` parameter.
///
/// \return ms8607_status : status of MS8607
///       - ms8607_status_ok : I2C transfer completed successfully
///       - ms8607_status_null_sensor : The pointer provided for the `new_sensor` parameter was NULL.
///       - ms8607_status_callback_error : Error occurred within a ms8607_host_functions function
///
enum ms8607_status ms8607_hsensor_read_user_register(ms8607_sensor *sensor, uint8_t *value, void *caller_context);

#endif /* MS8607_H_INCLUDED */
