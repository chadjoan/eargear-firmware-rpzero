
#ifndef TEPHT_COMMON_H_INCLUDED
#define TEPHT_COMMON_H_INCLUDED

#include <stddef.h>
#include <inttypes.h>

#if 0
// Below is part of an abandoned-but-tempting way to store debug information
// in a compiler-and-platform-agnostic manner. However, getting it to work
// in a way that's both efficient and safe (ex: for critical software) might
// require static analysis tools, and using those static analysis tools as
// part of the project's build process (e.g. to learn how many functions
// there are and how many times we might need to store {line,file,func} rows).

// Current proces for sizing this parameter is to run this command from this
// project's directory:
//   grep -P '\b([a-zA-Z0-9]+)_(error|success)\b' ./* | wc -l
// This will count all instances of things like `ms8607_error(...)`, since
// every appearance of that macro can potentially add one row to this database.
// The pathological scenario is that some program manages to somehow get
// every single error in the library to be emitted, which would add a row
// for every one of those instances. If we can still have space left over
// even after that happens, then we will never run out.
#ifndef TEPHT_DEBUG_FRAME_DB_ROW_CAPACITY
#define TEPHT_DEBUG_FRAME_DB_ROW_CAPACITY ((size_t)128)
#endif

typedef uint16_t  tepht_dframe_handle;
#endif

/// \brief   Boolean type with predictable storage size.
///
/// \details This type is used instead of <stdbool.h>'s `bool` type, as the `bool`
///          type is not required to occupy exactly one byte (it can be larger!).
///          This type is an alias of `uint8_t`, and therefore occupies exactly
///          one byte, always. This is done to ensure that bindings for non-C
///          languages will have a way to know the width of the ms5840's booleans
///          whenever they appear in a struct or as a function parameter.
///
typedef uint8_t  tepht_bool;

typedef void  (*tepht_string_printing_fn)(void *printer_context, const char* str, size_t len);

typedef struct tepht_string_printer {
	tepht_string_printing_fn  print_string;
	void                      *printer_context;
} tepht_string_printer;

void tepht_string_printer_init(tepht_string_printer *printer,  tepht_string_printing_fn  print_string,  void *printer_context);

// Print strings whose lengths are given in a separate integer of type `size_t`.
size_t tepht_print_string(tepht_string_printer  *printer, const char* str, size_t len);

// Prints strings that C arrays, e.g. string literals, whose lengths are found using `sizeof(msg)`.
#define tepht_print_array_string(printer, msg)  (tepht_print_string((printer), (msg), sizeof(msg)-1))

// Prints strings that are null-terminated.
size_t tepht_print_nt_string(tepht_string_printer  *printer, const char *msg);

// Prints an integer using the given string printer.
size_t tepht_print_int64(tepht_string_printer  *printer, int64_t number);

typedef enum tepht_status {

	/// The status returned by functions when no errors were detected
	/// and the function call proceeded until its operation completed.
	tepht_status_ok = 0,
	
	// TODO: implement, document, error handle, etc.
	// (also, maybe it would be better to return this status from a function?)
	// This "waiting" status was part of an attempt to write a non-blocking API.
	// That API proved too difficult to write under difficult health and
	// time constraints, so this is currently unused.
	//tepht_status_waiting,

	tepht_status_null_argument,
	tepht_status_null_sensor,
	tepht_status_null_host_function,
	tepht_status_callback_error,
	tepht_status_callback_i2c_nack,
	tepht_status_eeprom_is_zero, // Formerly ms8607_status_crc_error
	tepht_status_eeprom_crc_error, // Formerly ms8607_status_crc_error
	tepht_status_measurement_invalid, // Formerly ms8607_status_i2c_transfer_error
	// TODO: tepht_status_response_timeout, // I2C requests result in NACK even though the sensor should have responded by now. Possible driver state-machine desync.
	tepht_status_heater_on_error,
	tepht_status_i2c_read_unimplemented,
	tepht_status_i2c_write_unimplemented,
	tepht_status_i2c_write_no_stop_unimplemented,
	tepht_status_sleep_ms_unimplemented
} tepht_status;

// We use the u16 status (ex: in this API) when possible, because it is the
// most future-proof. But if something benefits from compact memory
// representation, we'll use the u8 version.
typedef uint8_t  tepht_status_u8;
typedef uint16_t tepht_status_u16;

#if 0
typedef struct tepht_vtable {
	// what goes here?
} tepht_vtable;

typedef struct tepht_classinfo {
	tepht_classinfo  *parent;
	const char       *name;
} tepht_classinfo;

extern tepht_classinfo  tepht_object_classinfo;

typedef struct tepht_object {
	tepht_vtable     *vtable;
	tepht_classinfo  *classinfo;
} tepht_object;
#endif

typedef struct tepht_driver_context_accessor_vtable {
	const char *(*get_device_model_name)(void *self);
	const char *(*get_driver_prefix)(void *self);
} tepht_driver_context_accessor_vtable;

typedef struct tepht_driver_context_accessor {
	void                                        *self;
	const tepht_driver_context_accessor_vtable  *vtable;
} tepht_driver_context_accessor;

static inline
	const char *tepht_get_device_model_name(tepht_driver_context_accessor  ctx_accessor)
{
	return ctx_accessor.vtable->get_device_model_name(ctx_accessor.self);
}

static inline
	const char *tepht_get_driver_prefix(tepht_driver_context_accessor  ctx_accessor)
{
	return ctx_accessor.vtable->get_driver_prefix(ctx_accessor.self);
}

typedef struct tepht_error_info
{
	// Internal state. Avoid accessing directly.
	const tepht_driver_context_accessor  *context_accessor;
	uint16_t                             line_number;
	tepht_status_u8                      status;
	uint8_t                              filename_index;
} tepht_error_info;

static inline
	tepht_error_info  tepht_success(const tepht_driver_context_accessor *ctx_accessor)
{
	tepht_error_info  einfo;
	einfo.context_accessor = ctx_accessor;
	einfo.line_number = 0;
	einfo.status = tepht_status_ok;
	einfo.filename_index = 0;
	return einfo;
}

#define           tepht_error(ctx_accessor, status)  (tepht_error_impl((ctx_accessor), __LINE__, (status), __FILE__))

// Please don't call `tepht_error_impl` directly; instead call the `tepht_error` macro-function.
tepht_error_info  tepht_error_impl(const tepht_driver_context_accessor *ctx_accessor, uint16_t line_number, tepht_status_u8  status, const char *filename);

static inline
	tepht_status_u16  tepht_error_get_status(tepht_error_info  einfo)
{
	return einfo.status;
}

static inline
	tepht_bool   tepht_is_success(tepht_error_info  einfo)
{
	return (einfo.status == tepht_status_ok);
}

static inline
	tepht_bool   tepht_is_error(tepht_error_info  einfo)
{
	return (einfo.status != tepht_status_ok);
}

#define  tepht_error_get_line_number(einfo)  (einfo.line_number)

const char *tepht_error_get_filename(tepht_error_info  einfo);

//TODO: fix documentation

/// \brief   Returns a string describing the given error code.
///
/// \details The caller should filter out any cases where `tepht_get_status(einfo)` equals
///          `tepht_status_callback_error`. The "error within callback"
///          return value is not returned from the ms5840 driver itself, so the
///          ms5840 driver has no way to know what caused the error.
///          This function will still return a valid string constant in those
///          cases, but it will a very generic message and thus won't be
///          specific enough for conclusive troubleshooting.
///
///          The `tepht_status_callback_error` code is intended to be returned
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
size_t tepht_print_error(tepht_error_info  einfo,  tepht_string_printing_fn  print_string,  void *printer_context);

/// This type provides an extremely simple implementation of a printer that
/// prints text to a caller-provided buffer.
///
/// This can be used with the `tepht_print_error` function to print error text
/// into plain strings (of type `char*`).
///
/// This data structure is being kept simplistic intentionally, and will be
/// unlikely to obtain any more features.  So if additional features are
/// required (ex: buffer resizing, stronger ownership semantics,
/// synchronization guarantees, optimizations, read cursors, etc), then it
/// is recommended to look elsewhere or implement a custom solution.
///
typedef struct tepht_buffer_printer
{
	char   *buffer;
	size_t buflen;
	size_t cursor;

} tepht_buffer_printer;

/// Initializes a new `tepht_buffer_printer`, providing it the given `buffer`
/// of size `buffer_size` (in bytes).
///
/// The printer will start with its cursor at position 0, and will place
/// a single null-terminator character ('\0') at the beginning of the buffer.
///
void tepht_buffer_printer_init(tepht_buffer_printer *printer, char *buffer, size_t buffer_size);

/// This function prints `len` bytes from the given string `str` into the
/// buffer provided in the given `tepht_buffer_printer`.
///
/// This function will ensure that the buffer always has a '\0' at the end of
/// the printer's buffer. This is true even if the text in `str` is longer
/// than the remaining buffer space and ends up truncated. In such a case of
/// truncation, the last character of the buffer will always be the null
/// terminator character, '\0'.
///
/// This function's signature is intentionally made to match the function
/// pointer type `tepht_string_printing_fn` which is used as a parameter
/// in the `tepht_print_error` function.
///
/// If `str` is NULL, no printing will be performed, and the call to this
/// function will effectively do nothing.
///
void tepht_buffer_printer_print_string(void *printer, const char *str, size_t len);

/// Returns the contents of the `tepht_buffer_printer`'s buffer as a null-terminated
/// string. The value pointed to by the `result_size` pointer will be populated
/// with the number of bytes (not including the null terminating character) that
/// were written into the buffer.
///
/// It is allowed to pass NULL into the `result_size` parameter, in which case
/// the `result_size` argument will be ignored and the length of the returned
/// string will simply not be provided.
///
/// If the caller persisted a pointer to the buffer inside the `tepht_buffer_printer`
/// object, then this function becomes largely redundant with the
/// `tepht_buffer_printer_get_byte_count`
char *tepht_buffer_printer_get_data(tepht_buffer_printer *printer, size_t *result_size);

/// The value of this function will never be any larger than the original
/// buffer's size, minus one (for the guaranteed null-terminator character).
size_t tepht_buffer_printer_get_byte_count(tepht_buffer_printer *printer);

/// Resets the given `tepht_buffer_printer`'s print cursor to the 0 position
/// and places a single '\0' character into that position in the buffer.
void tepht_buffer_printer_clear(tepht_buffer_printer *printer);

///
/// \brief  This structure is used by the MS5840 driver to specify I2C transfers
///         for the callbacks that implement I2C functionality for the driver.
///
typedef struct tepht_i2c_controller_packet {
	/// \brief Address to peripheral device
	uint16_t address;

	/// \brief Length of data array
	uint16_t data_length;

	/// \brief Data array containing all data to be transferred
	uint8_t *data;
} tepht_i2c_controller_packet;

///
/// \brief    This structure allows the caller to provide implementations for the
///           dependencies of the various TE Connectivity PHT sensors.
///           Most of what is required is I2C functionality.
///
/// \details  Some patterns are employed to keep the driver and callbacks
///           flexible and thread-safe:
///    - A `caller_context` pointer is passed from the caller into the driver,
///        and then from the driver into these callbacks. This is what allows
///        the caller and their callbacks to communicate and persist data
///        across calls into the driver (and to do so without relying on
///        global variables or thread-local-storage).
///    - The return value is usually the `tepht_status` enum, but only one of
///        two possible enum values shall be returned from the callback:
///        `tepht_status_ok` and `tepht_status_callback_error`.
///    - Returning `tepht_status_ok` indicates that the callback completed its
///        operation (usually an I2C transaction) successfully. This tells the
///        driver that it can continue working.
///    - Returning `tepht_status_callback_error` indicates that something
///        went wrong. The driver will typically return from its own function
///        immediately after receiving this error code from a callback. This
///        is intended primarily as a way to provide the driver with "go / no-go"
///        information, and nothing more specific than that. If the caller needs
///        to return more specific error details from a failed callback,
///        the `caller_context` argument should be used for this purpose.
///
typedef struct tepht_host_functions {
	/// \brief  Internal state; do not modify.
	uint8_t  validated_;

	/// \brief   Callback that shall implement I2C packet reading (receive|rx) functionality.
	///
	/// \details This callback (caller-supplied function) shall read an I2C packet
	///          from the I2C bus that the sensor is connected to.
	///          This allows the driver to complete I2C transfers without
	///          directly depending on any one specific I2C implementation.
	///
	///          This callback is required: no driver will be able to
	///          retrieve readings from the sensor without it.
	///
	///          If the polling interface is used (TODO: call out the function name)
	///          then this function must distinguish between I2C NACK responses
	///          (by returning `tepht_status_callback_i2c_nack`) and all other possibly
	///          negative results from I2C transfers (by returning `tepht_status_callback_error`).
	///          If this behavior is not implemented while the polling interface
	///          is in use, then the polling interface will always report failure.
	///          This is necessary for polling because the MS5840 and MS8607
	///          (for example) use a NACK response to indicate that the
	///          controller must wait a litte bit longer before a measurement
	///          result becomes available.
	///
	/// \return  tepht_status : Lets the driver know if the I2C transmit was successful.
	///        - tepht_status_ok : I2C transfer completed successfully
	///        - tepht_status_callback_error : Problem with i2c transfer
	///        - tepht_status_callback_i2c_nack : I2C peripheral responded with NACK
	///
	tepht_status_u16  (*i2c_controller_read)(void *caller_context, tepht_i2c_controller_packet *const);

	/// \brief   Callback that shall implement I2C packet writing (trasmit|tx) functionality.
	///
	/// \details This callback (caller-supplied function) shall write an I2C packet
	///          to the I2C bus that the sensor is connected to.
	///          This allows the driver to complete I2C transfers without
	///          directly depending on any one specific I2C implementation.
	///
	///          This callback is required: no driver will be able to
	///          retrieve readings from the sensor without it.
	///
	/// \return  tepht_status : Lets the driver know if the I2C transmit was successful.
	///        - tepht_status_ok : I2C transfer completed successfully
	///        - tepht_status_callback_error : Problem with i2c transfer
	///
	tepht_status_u16  (*i2c_controller_write)(void *caller_context, tepht_i2c_controller_packet *const);

	/// \brief   Callback that shall implement I2C packet writing (trasmit|tx) functionality,
	///          but does not transmit a "stop" bit at the end of the I2C packet.
	///
	/// \details This callback is currently MS8607-specific and will not be used
	///          in other drivers (which is only the MS5840, as of this writing).
	///
	///          This callback (caller-supplied function) shall write an I2C packet
	///          to the I2C bus that the MS8607 sensor is connected to.
	///          Unlike the `i2c_controller_write_packet`  function, this version
	///          shall NOT write a "stop" bit at the end of the packet.
	///
	///          If the caller's I2C implementation is not capable of this,
	///          then it is recommended that the caller provide a stub function
	///          that returns `tepht_status_callback_error`. The caller
	///          should then prevent any use of the MS8607's "hold" mode.
	///
	///          This callback is optional: the MS8607 driver can retrieve
	///          readings from the MS8607 sensor without it, but it becomes
	///          necessary if the sensor is used in "hold" mode
	///          (`ms8607_humidity_i2c_controller_mode : ms8607_i2c_hold`).
	///
	/// \return  tepht_status : Lets the driver know if the I2C transmit was successful.
	///        - tepht_status_ok : I2C transfer completed successfully
	///        - tepht_status_callback_error : Problem with i2c transfer
	///
	tepht_status_u16  (*i2c_controller_write_no_stop)(void *caller_context, tepht_i2c_controller_packet *const);

	/// \brief   Callback that shall wait for the given number of milliseconds when called.
	///
	/// \details If the caller is operating in a multi-threaded environment
	///          (including software-based schedulers running on single-threaded
	///          processors), then it is perfectly acceptable to yield this time
	///          to other threads or fibers.
	///
	tepht_status_u16  (*sleep_ms)(void *caller_context, uint32_t milliseconds);

	/// \brief  Optional callback that is used to print, report, or log errors or diagnostic messages.
	void  (*print_string)(void *caller_context, const char *text);

	/// \brief  Optional callback that is used to print, report, or log errors or diagnostic messages.
	void  (*print_int64)(void *caller_context,  int64_t  number, uint8_t pad_width,  tepht_bool  pad_with_zeroes);

} tepht_host_functions;

// For internal use.
//
// The functions that need host functions validated will either call this on
// their own or do their own (more specific) validation.
//
// This does need to be a public symbol, though, as it is used by multiple
// drivers.
tepht_status  tepht_validate_mandatory_depends(tepht_host_functions *deps);

/// \brief    Creates a `tepht_host_functions` object (the `dependencies` parameter)
///           to store function pointers that implement the driver's dependencies.
///
/// \details  The purpose of this function is to create a `tepht_host_functions`
///           object, which can then be used by the drivers to satisfy
///           their dependencies.
///
///           This must be called before calling any `*_init_sensor`
///           function, as the `*_init_sensor` functions require the
///           `tepht_host_functions` object that is populated by this function.
///
///           This function works in 3 steps:
///
///           (1) It initializes the `tepht_host_functions` object
///             given by the `dependencies` parameter. This places the object
///             into a known state so that the 3rd phase of this function can
///             know which functions were implemented by the caller/host.
///
///           (2) It calls the given `assign_functions` callback on `dependencies`.
///             The callback should create function pointers from host functions
///             that implement the various requirements of the driver, such as
///             I2C I/O and timing mechanisms. Those function pointers should be
///             assigned to the various members of the `dependencies` structure.
///             See the `tepht_host_functions` for details on the necessary functions.
///
///           (3) After the callback returns, this function then validates
///             the resulting `tepht_host_functions` object to ensure that
///             minimal requirements are met. Appropriate error codes are
///             returned if the driver's dependencies were not satisfied.
///
///           The `assign_functions` callback shall NOT assign NULL to any
///           members of the `tepht_host_functions *dependencies` structure.
///           When a function pointer is optional and no implementation
///           is available, `assign_functions` should leave that member
///           unmodified.
///
///           The `tepht_init_and_assign_host_functions` function will
///           have already assigned stubs (and missing requirement detectors)
///           to the members of the `tepht_host_functions` structure.
///
///           This function is reentrant, idempotent, non-blocking,
///           and does not perform any I/O. These properties assume that
///           the `assign_functions` callback also possesses the same
///           corresponding properties. This function is thread-safe
///           as long as, during this function's execution, no other threads
///           read from or write to the objects pointed to by this function's
///           arguments.
///
/// \param[out] tepht_host_functions* : Struct with callbacks that implement I2C controller functions.
/// \param[in] void* caller_context : This is passed to the `assign_functions`
///           callback's `caller_context` parameter.
/// \param[in] void (*assign_functions)(tepht_host_functions *dependencies, void *caller_context):
///           Pointer to a caller-implemented function that shall assign pointers
///           to implementation functions that are required by the driver.
///
/// \return tepht_error_info : `tepht_error_get_status(error_info)` would have these possibilities:
///       - tepht_status_ok : Returned if everything completed without error.
///       - tepht_status_null_argument : Returned if the `dependencies` or `assign_functions` parameters were NULL.
///       - tepht_status_null_host_function : Returned if NULL was assigned to any of the members of `dependencies`.
///       - tepht_status_i2c_read_unimplemented : Returned if the `i2c_controller_read` function was not assigned.
///       - tepht_status_i2c_write_unimplemented : Returned if the `i2c_controller_write` function was not assigned.
///       - tepht_status_sleep_ms_unimplemented : Returned if the `sleep_ms` function was not assigned.
///
tepht_error_info  tepht_init_and_assign_host_functions(
	tepht_host_functions *deps,
	void *caller_context,
	void (*assign_functions)(tepht_host_functions *deps, void *caller_context)
	);


typedef struct tepht_pt_sensor_vtable {
	// Internal, module-level, state. Avoid accessing directly.
	tepht_bool        (*is_connected)(void *self,  void *caller_context);
	tepht_error_info  (*reset)(void *self,  void *caller_context);
	tepht_error_info  (*read_temperature_pressure_int32)(void *self, int32_t *t, int32_t *p, void *caller_context);
} tepht_pt_sensor_vtable;

typedef struct tepht_pt_sensor {
	// Internal state. Avoid accessing directly.
	void                          *self;
	const tepht_pt_sensor_vtable  *vtable;
} tepht_pt_sensor;

tepht_bool  tepht_pt_sensor_is_connected(tepht_pt_sensor  sensor,  void *caller_context);
tepht_error_info  tepht_pt_sensor_reset(tepht_pt_sensor  sensor,  void *caller_context);
tepht_error_info  tepht_pt_sensor_read_temperature_pressure_int32(
	tepht_pt_sensor  sensor, int32_t *t, int32_t *p, void *caller_context);

/// Runs all unittests in the `tepht_common` module.
void tepht_run_unittests(tepht_string_printer  *printer);

#endif
