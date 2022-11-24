
#ifndef TEPHT_COMMON_H_INCLUDED
#define TEPHT_COMMON_H_INCLUDED

#include <stddef.h>
#include <inttypes.h>

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


// TODO: Copy the "host_functions" structure into here and update docs.

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
