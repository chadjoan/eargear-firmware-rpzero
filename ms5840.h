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

// Macros

#define MS5840_COEFFICIENT_COUNT  (7)

// Enums

enum ms5840_status {
	ms5840_status_ok = 0,
	ms5840_status_waiting, // TODO: document, error handle, etc.
	ms5840_status_null_argument,
	ms5840_status_null_sensor,
	ms5840_status_null_host_function,
	ms5840_status_callback_error,
	ms5840_status_callback_i2c_nack,
	ms5840_status_eeprom_is_zero, // Formerly ms5840_status_crc_error
	ms5840_status_eeprom_crc_error, // Formerly ms5840_status_crc_error
	ms5840_status_measurement_invalid, // Formerly ms5840_status_i2c_transfer_error
	// TODO: ms5840_status_response_timeout, // I2C requests result in NACK even though the sensor should have responded by now. Possible driver state-machine desync.
	ms5840_status_i2c_read_unimplemented,
	ms5840_status_i2c_write_unimplemented,
	ms5840_status_sleep_ms_unimplemented
};

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

// Basic types

/// \brief   Boolean type with predictable storage size.
///
/// \details This type is used instead of <stdbool.h>'s `bool` type, as the `bool`
///          type is not required to occupy exactly one byte (it can be larger!).
///          This type is an alias of `uint8_t`, and therefore occupies exactly
///          one byte, always. This is done to ensure that bindings for non-C
///          languages will have a way to know the width of the ms5840's booleans
///          whenever they appear in a struct or as a function parameter.
///
typedef uint8_t  ms5840_bool;

// Structs

///
/// \brief  This structure is used by the MS5840 driver to specify I2C transfers
///         for the callbacks that implement I2C functionality for the driver.
///
typedef struct ms5840_i2c_controller_packet {
	/// \brief Address to peripheral device
	uint16_t address;

	/// \brief Length of data array
	uint16_t data_length;

	/// \brief Data array containing all data to be transferred
	uint8_t *data;
} ms5840_i2c_controller_packet;

///
/// \brief    This structure allows the caller to provide implementations for the
///           MS5840 driver's dependencies, which are mostly I2C functionality.
///
/// \details  Some patterns are employed to keep the driver and callbacks
///           flexible and thread-safe:
///    - A `caller_context` pointer is passed from the caller into the MS5840
///        driver, and then from the MS5840 driver into these callbacks. This is
///        what allows the caller and their callbacks to communicate and persist
///        data accross calls into the driver (and to do so without relying on
///        global variables or thread-local-storage).
///    - The return value is usually the `ms5840_status` enum, but only one of
///        two possible enum values shall be returned from the callback:
///        `ms5840_status_ok` and `ms5840_status_callback_error`.
///    - Returning `ms5840_status_ok` indicates that the callback completed its
///        operation (usually an I2C transaction) successfully. This tells the
///        driver that it can continue working.
///    - Returning `ms5840_status_callback_error` indicates that something
///        went wrong. The driver will typically return from its own function
///        immediately after receiving this error code from a callback. This
///        is intended primarily as a way to provide the driver with "go / no-go"
///        information, and nothing more specific than that. If the caller needs
///        to return more specific error details from a failed callback,
///        the `caller_context` argument should be used for this purpose.
///
typedef struct ms5840_host_functions {
	/// \brief  Internal state; do not modify.
	uint8_t  validated_;

	/// \brief   Callback that shall implement I2C packet reading (receive|rx) functionality.
	///
	/// \details This callback (caller-supplied function) shall read an I2C packet
	///          from the I2C bus that the MS5840 sensor is connected to.
	///          This allows the MS5840 driver to complete I2C transfers without
	///          directly depending on any one specific I2C implementation.
	///
	///          This callback is required: the MS5840 driver will be unable to
	///          retrieve readings from the MS5840 sensor without it.
	///
	///          If the polling interface is used (TODO: call out the function name)
	///          then this function must distinguish between I2C NACK responses
	///          (by returning `ms5840_status_callback_i2c_nack`) and all other possibly
	///          negative results from I2C transfers (by returning `ms5840_status_callback_error`).
	///          If this behavior is not implemented while the polling interface
	///          is in use, then the polling interface will always report failure.
	///          This is necessary for polling because the MS5840 uses a NACK
	///          response to indicate that the controller must wait a litte bit
	///          longer before a measurement result becomes available.
	///
	/// \return  ms5840_status : Lets the driver know if the I2C transmit was successful.
	///        - ms5840_status_ok : I2C transfer completed successfully
	///        - ms5840_status_callback_error : Problem with i2c transfer
	///        - ms5840_status_callback_i2c_nack : I2C peripheral responded with NACK
	///
	enum ms5840_status  (*i2c_controller_read)(void *caller_context, ms5840_i2c_controller_packet *const);

	/// \brief   Callback that shall implement I2C packet writing (trasmit|tx) functionality.
	///
	/// \details This callback (caller-supplied function) shall write an I2C packet
	///          to the I2C bus that the MS5840 sensor is connected to.
	///          This allows the MS5840 driver to complete I2C transfers without
	///          directly depending on any one specific I2C implementation.
	///
	///          This callback is required: the MS5840 driver will be unable to
	///          retrieve readings from the MS5840 sensor without it.
	///
	/// \return  ms5840_status : Lets the driver know if the I2C transmit was successful.
	///        - ms5840_status_ok : I2C transfer completed successfully
	///        - ms5840_status_callback_error : Problem with i2c transfer
	///
	enum ms5840_status  (*i2c_controller_write)(void *caller_context, ms5840_i2c_controller_packet *const);

	/// \brief   Callback that shall wait for the given number of milliseconds when called.
	///
	/// \details If the caller is operating in a multi-threaded environment
	///          (including software-based schedulers running on single-threaded
	///          processors), then it is perfectly acceptable to yield this time
	///          to other threads or fibers.
	///
	enum ms5840_status  (*sleep_ms)(void *caller_context, uint32_t milliseconds);

	/// \brief  Optional callback that is used to print, report, or log errors or diagnostic messages.
	void  (*print_string)(void *caller_context, const char *text);

	/// \brief  Optional callback that is used to print, report, or log errors or diagnostic messages.
	void  (*print_int64)(void *caller_context,  int64_t  number, uint8_t pad_width,  ms5840_bool  pad_with_zeroes);

} ms5840_host_functions;


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
	const ms5840_host_functions                *host_funcs;
	enum ms5840_pressure_resolution            psensor_resolution_osr;
	ms5840_bool                                psensor_coeff_read;
	uint16_t                                   eeprom_coeff[MS5840_COEFFICIENT_COUNT+1];
} ms5840_sensor;


// Functions

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
enum ms5840_status  ms5840_init_and_assign_host_functions(
	ms5840_host_functions *deps,
	void *caller_context,
	void (*assign_functions)(ms5840_host_functions *deps, void *caller_context)
	);

// TODO: Ensure that `ms5840_reset` always gets called, even if the caller skips it.
// (e.g., by calling it lazily from whatever might need it).

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
enum ms5840_status  ms5840_init_sensor(ms5840_sensor *new_sensor,  ms5840_host_functions *depends_to_use);

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
bool ms5840_is_connected(ms5840_sensor *sensor,  void *caller_context);

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
enum ms5840_status ms5840_reset(ms5840_sensor *sensor,  void *caller_context);

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
enum ms5840_status ms5840_set_pressure_resolution(ms5840_sensor *sensor, enum ms5840_pressure_resolution, void *caller_context);

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
enum ms5840_status ms5840_read_temperature_pressure_int32(ms5840_sensor *sensor, int32_t *, int32_t *, void *caller_context);

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
enum ms5840_status ms5840_read_temperature_pressure_float32(ms5840_sensor *sensor, float *, float *, void *caller_context);

/// \brief   Returns a string describing the given error code.
///
/// \details The caller should filter out any cases where `error_code` equals
///          `ms5840_status_callback_error`. The "error within callback"
///          return value is not returned from the ms5840 driver itself, so the
///          ms5840 driver has no way to know what caused the error.
///          This function will still return a valid string constant in those
///          cases, but it will a very generic message and thus won't be
///          specific enough for conclusive troubleshooting.
///
///          The `ms5840_status_callback_error` code is intended to be returned
///          from function pointers (callbacks) provided to the `ms5840_host_functions`
///          structure. Since the caller provides these callbacks, it is
///          the caller's responsibility to either report such errors from within
///          the callbacks, or to track that error information separately
///          (ex: by passing a custom struct pointer as the `caller_context` parameter,
///          then using such a struct to persist more specific error information
///          past the ms5840 driver function's return).
///
///          This function is reentrant, thread-safe, non-blocking, and pure
///          (it does not perform any I/O or have any side effects).
///
/// \return  A non-NULL string describing the given error code.
///          If the error code is not valid for any reason, the returned string
///          will simply indicate that the error code was not valid.
///
const char *ms5840_stringize_error(enum ms5840_status error_code);

#endif /* MS5840_H_INCLUDED */
