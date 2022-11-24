
#include "tepht_common.h"

#include <assert.h>
#include <string.h>
#include <stddef.h>
#include <inttypes.h>

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


#if 0
static void tepht_buffer_printer_dump(tepht_buffer_printer *printer)
{
	size_t i;
	for ( i = 0; i <= printer->cursor; i++ )
	{
		if ( 0 == i % 16 )
			printf("\n");
		printf("%02X, ", printer->buffer[i]);
	}
}
#endif

/// Implementation of digit-encoder for `*_to_string` functions.
///
/// This version does no assertions and assumes that `output_len` is never NULL.
static char *tepht_uint64_to_string_impl(uint64_t num, uint8_t base, char *buffer, size_t buflen, size_t *output_len)
{
	size_t i = 0;

	// Put single zero digit if `num` is zero.
	if ( num == 0 )
	{
		buffer[i++] = '0';

		buffer[i] = '\0';
		*output_len = i;
		return buffer;
	}

	// Write the number out.
	// This will put it out in reverse order, so we'll have to reverse it later.
	size_t digitStart = i;
	while(num > 0)
	{
		uint8_t digit = (uint8_t)(num % base);
		num /= base;

		if (base <= 10)
			buffer[i] = (char)('0' + digit);
		else
		{
			if ( digit < 10 )
				buffer[i] = (char)('0' + digit);
			else
				buffer[i] = (char)('A' + (digit-10));
		}

		i++;
	}

	size_t digitEnd = i;

	// Reverse/swizzle
	size_t halfway = (digitEnd - digitStart) / 2;
	size_t lastDigit = digitEnd-1;
	for ( i = 0; i < halfway; i++ )
	{
		size_t lo = digitStart + i;
		size_t hi = lastDigit - i;

		char tmp = buffer[lo];
		buffer[lo] = buffer[hi];
		buffer[hi] = tmp;
	}

	buffer[digitEnd] = '\0';
	*output_len = digitEnd;
	return buffer;
}

/// Function that converts an int64_t, the `num` parameter, into a textual
/// representation in the given `base`.
///
/// This assumes that the given 'buffer' contains enough space to render
/// the largest possible representation of the given integer. If it isn't,
/// undefined behavior can result.
///
/// For decimal outputs (base == 10), the string "9223372036854775807" is the
/// highest possible output, and "-9223372036854775808" is the lowest.
/// The longest of those is "-9223372036854775808", which is 20 characters long.
/// An additional character would be needed to store the null terminator, '\0'.
/// Thus, the minimum buffer size needed to represent all possible int64_t
/// numbers is 21 characters (bytes).
///
/// The value pointed to by the `output_len` pointer will be assigned the
/// length of the string that was produced by this function. So if the output
/// is "5", then `*output_len` will become 1; if the output is "57", then
/// `*output_len` will become 2; if the output is "123", then `*output_len`
/// will become 3, and so on. NULL may be passed into this parameter, in
/// which case the length of the resulting string will not be returned to
/// the caller, and the only way to determine the length will be to
/// scan the resulting string for the null-terminating character,
/// e.g. to use `strlen` on the return value of this function.
///
/// For decimal outputs (base == 10), the number 18446744073709551616 is the
/// largest possible 
///
/// Only bases between 2 and 36 (inclusive) are supported.
///
char *tepht_int64_to_string(int64_t num, uint8_t base, char *buffer, size_t buflen, size_t *output_len)
{
	assert( 2 <= base && base <= 36 );

	assert(buffer != NULL);
	assert(buflen >= 21 || base != 10); // 1 sign + 19 digits + 1 null terminator
	assert(buflen >= 18 || base != 16); // 1 sign + 16 hexadigits + 1 null terminator

	size_t impl_output_len;

	if ( num >= 0 )
	{
		tepht_uint64_to_string_impl(num, base, buffer, buflen, &impl_output_len);
	}
	else
	{
		num = -num;
		buffer[0] = '-';

		char *digits = tepht_uint64_to_string_impl(num, base, buffer+1, buflen-1, &impl_output_len);
		assert((buffer + 1) == digits);
		impl_output_len++;
	}

	if ( output_len != NULL ) {
		*output_len = impl_output_len;
	}

	return buffer;
}


/// Function that converts an uint64_t, the `num` parameter, into a textual
/// representation in the given `base`.
///
/// This assumes that the given 'buffer' contains enough space to render
/// the largest possible representation of the given integer. If it isn't,
/// undefined behavior can result.
///
/// For decimal outputs (base == 10), the string "18446744073709551615" is the
/// highest possible output, and "0" is the lowest. The longest of those is
/// "18446744073709551615", which is 20 characters long.
/// An additional character would be needed to store the null terminator, '\0'.
/// Thus, the minimum buffer size needed to represent all possible uint64_t
/// numbers is 21 characters (bytes).
///
/// The value pointed to by the `output_len` pointer will be assigned the
/// length of the string that was produced by this function. So if the output
/// is "5", then `*output_len` will become 1; if the output is "57", then
/// `*output_len` will become 2; if the output is "123", then `*output_len`
/// will become 3, and so on. NULL may be passed into this parameter, in
/// which case the length of the resulting string will not be returned to
/// the caller, and the only way to determine the length will be to
/// scan the resulting string for the null-terminating character,
/// e.g. to use `strlen` on the return value of this function.
///
///
/// Only bases between 2 and 36 (inclusive) are supported.
///
char *tepht_uint64_to_string(uint64_t num, uint8_t base, char *buffer, size_t buflen, size_t *output_len)
{
	size_t dummy_output;
	if ( output_len == NULL )
		output_len = &dummy_output;

	assert( 2 <= base && base <= 36 );

	assert(buffer != NULL);
	assert(buflen >= 21 || base != 10); // 20 digits + 1 null terminator
	assert(buflen >= 17 || base != 16); // 16 hexadigits + 1 null terminator

	return tepht_uint64_to_string_impl(num, base, buffer, buflen, output_len);
}

///
char *tepht_int64_to_binary(int64_t num, char *buffer, size_t buflen, size_t *outlen)
{
	return tepht_int64_to_string(num, 2, buffer, buflen, outlen);
}

///
char *tepht_uint64_to_binary(uint64_t num, char *buffer, size_t buflen, size_t *outlen)
{
	return tepht_uint64_to_string(num, 2, buffer, buflen, outlen);
}

///
char *tepht_int64_to_octal(int64_t num, char *buffer, size_t buflen, size_t *outlen)
{
	return tepht_int64_to_string(num, 8, buffer, buflen, outlen);
}

///
char *tepht_uint64_to_octal(uint64_t num, char *buffer, size_t buflen, size_t *outlen)
{
	return tepht_uint64_to_string(num, 8, buffer, buflen, outlen);
}

///
char *tepht_int64_to_dec(int64_t num, char *buffer, size_t buflen, size_t *outlen)
{
	return tepht_int64_to_string(num, 10, buffer, buflen, outlen);
}

///
char *tepht_uint64_to_dec(uint64_t num, char *buffer, size_t buflen, size_t *outlen)
{
	return tepht_uint64_to_string(num, 10, buffer, buflen, outlen);
}

///
char *tepht_int64_to_hex(int64_t num, char *buffer, size_t buflen, size_t *outlen)
{
	return tepht_int64_to_string(num, 16, buffer, buflen, outlen);
}

///
char *tepht_uint64_to_hex(uint64_t num, char *buffer, size_t buflen, size_t *outlen)
{
	return tepht_uint64_to_string(num, 16, buffer, buflen, outlen);
}

static void unittest_int_to_str(tepht_string_printer  *printer)
{
	char buffer[32];
	size_t buflen = sizeof(buffer);

	tepht_print_array_string(printer, "  unittest_int_to_str, for the tepht_int64_to_string(...) function:");

	assert(0 == strcmp(tepht_int64_to_dec(0  , buffer, buflen, NULL), "0"  ));
	assert(0 == strcmp(tepht_int64_to_dec(-0 , buffer, buflen, NULL), "0"  ));
	assert(0 == strcmp(tepht_int64_to_dec(32 , buffer, buflen, NULL), "32" ));
	assert(0 == strcmp(tepht_int64_to_dec(31 , buffer, buflen, NULL), "31" ));
	assert(0 == strcmp(tepht_int64_to_dec(-8 , buffer, buflen, NULL), "-8" ));
	assert(0 == strcmp(tepht_int64_to_dec(-32, buffer, buflen, NULL), "-32"));

	assert(0 == strcmp(tepht_int64_to_hex(0  , buffer, buflen, NULL), "0"  ));
	assert(0 == strcmp(tepht_int64_to_hex(-0 , buffer, buflen, NULL), "0"  ));
	assert(0 == strcmp(tepht_int64_to_hex(32 , buffer, buflen, NULL), "20" ));
	assert(0 == strcmp(tepht_int64_to_hex(31 , buffer, buflen, NULL), "1F" ));
	assert(0 == strcmp(tepht_int64_to_hex(-8 , buffer, buflen, NULL), "-8" ));
	assert(0 == strcmp(tepht_int64_to_hex(-32, buffer, buflen, NULL), "-20"));

	assert(0 == strcmp(tepht_int64_to_hex(10, buffer, buflen, NULL), "A"));
	assert(0 == strcmp(tepht_int64_to_hex(11, buffer, buflen, NULL), "B"));
	assert(0 == strcmp(tepht_int64_to_hex(12, buffer, buflen, NULL), "C"));
	assert(0 == strcmp(tepht_int64_to_hex(13, buffer, buflen, NULL), "D"));
	assert(0 == strcmp(tepht_int64_to_hex(14, buffer, buflen, NULL), "E"));
	assert(0 == strcmp(tepht_int64_to_hex(15, buffer, buflen, NULL), "F"));

	assert(0 == strcmp(tepht_int64_to_binary(0  , buffer, buflen, NULL), "0"      ));
	assert(0 == strcmp(tepht_int64_to_binary(-0 , buffer, buflen, NULL), "0"      ));
	assert(0 == strcmp(tepht_int64_to_binary(32 , buffer, buflen, NULL), "100000" ));
	assert(0 == strcmp(tepht_int64_to_binary(31 , buffer, buflen, NULL), "11111"  ));
	assert(0 == strcmp(tepht_int64_to_binary(-8 , buffer, buflen, NULL), "-1000"  ));
	assert(0 == strcmp(tepht_int64_to_binary(-32, buffer, buflen, NULL), "-100000"));

	tepht_print_array_string(printer, "  Passed.\n");
}

void tepht_string_printer_init(tepht_string_printer *printer,  tepht_string_printing_fn  print_string,  void *printer_context)
{
	printer->print_string    = print_string;
	printer->printer_context = printer_context;
}

// Print strings whose lengths are given in a separate integer of type `size_t`.
size_t tepht_print_string(tepht_string_printer  *printer, const char* str, size_t len)
{
	printer->print_string(printer->printer_context, str, len);
	return len;
}

// Prints strings that are null-terminated.
size_t tepht_print_nt_string(tepht_string_printer  *printer, const char *msg)
{
	size_t len = strlen(msg);
	printer->print_string(printer->printer_context, msg, len);
	return len;
}

// Prints an integer using the given string printer.
size_t tepht_print_int64_as_dec(tepht_string_printer  *printer, int64_t number)
{
	char buffer[24];
	size_t buflen = sizeof(buffer);
	size_t len = 0;
	char *str = tepht_int64_to_dec(number, buffer, buflen, &len);
	printer->print_string(printer->printer_context, str, len);
	return len;
}

// Prints an integer using the given string printer.
size_t tepht_print_uint64_as_dec(tepht_string_printer  *printer, uint64_t number)
{
	char buffer[24];
	size_t buflen = sizeof(buffer);
	size_t len = 0;
	char *str = tepht_uint64_to_dec(number, buffer, buflen, &len);
	printer->print_string(printer->printer_context, str, len);
	return len;
}

// This table exists as an optimization.
// It allows us to return filename information as an 8-bit index into this
// array, as opposed to a 32-bit or 64-bit pointer. In most cases this wouldn't
// matter much, but it allows us to keep the `tepht_error_info` struct small,
// which is important because that struct is returned-by-value from everything.
// And it's important to be able to return a struct for error info, not just
// an integer, because this allows us to print better error messages
// (e.g. we can parameterize the error messages).
static const char *tepht_filenames[] = {
	"filename_not_provided",
	"ms5840.c",
	"ms8607.c",
	"tepht_common.c"
};

static uint8_t filename_to_index(const char *filename)
{
	uint8_t len = (sizeof(tepht_filenames) / sizeof(tepht_filenames[0]));
	uint8_t i;
	for ( i = 0; i < len; i++ )
	{
		const char *candidate = tepht_filenames[i];
		if ( 0 == strcmp(filename, candidate) )
			break;
	}

	if ( i >= len )
		return 0; // filename_not_provided
	else
		return i;
}

static const char *index_to_filename(uint8_t filename_index)
{
	uint8_t len = (sizeof(tepht_filenames) / sizeof(tepht_filenames[0]));
	if ( filename_index < len )
		return tepht_filenames[filename_index];
	else
		return "invalid_filename_index";
}

static void unittest_filename_indexes(tepht_string_printer  *printer)
{
	tepht_print_array_string(printer, "  unittest_filename_indexes (for debug/error handling):");

	assert(0 == filename_to_index("foo"));
	assert(1 == filename_to_index("ms5840.c"));

	assert(0 == strcmp(index_to_filename(255), "invalid_filename_index"));
	assert(0 == strcmp(index_to_filename(0), "filename_not_provided"));
	assert(0 == strcmp(index_to_filename(1), "ms5840.c"));

	uint8_t len = (sizeof(tepht_filenames) / sizeof(tepht_filenames[0]));
	size_t i;
	for ( i = 1; i < len; i++ ) {
		assert(i == filename_to_index(index_to_filename(i)));
	}

	tepht_print_array_string(printer, "  Passed.\n");
}

#if 0
tepht_vtable     tepht_object_vtable =
	{
		
	};

tepht_classinfo  tepht_object_classinfo =
	{
		.parent = NULL,
		.name = "tepht_object"
	};
#endif

#if 0
// Example setup for `tepht_driver_context_accessor` instance.


const char *ms8607_get_device_model_name(void *ms8607_driver)
{
	return "MS8607";
}

const char *ms8607_get_driver_prefix(void *ms8607_driver)
{
	return "ms8607";
}

tepht_driver_context_accessor_vtable  ms8607_dca_vtable =
	{
		.get_device_model_name = ms8607_get_device_model_name,
		.get_driver_prefix     = ms8607_get_driver_prefix
	};

// TODO: This might not align with the most practical way to do it.
tepht_driver_context_accessor  ms8607_get_driver_context_accessor(ms8607_sensor *sensor)
{
	tepht_driver_context_accessor  ctx_accessor;
	ctx_accessor.self   = sensor;
	ctx_accessor.vtable = &ms8607_dca_vtable;
	return ctx_accessor;
}
#endif

tepht_error_info  tepht_error_impl(const tepht_driver_context_accessor *ctx_accessor, uint16_t line_number, tepht_status_u8  status, const char *filename)
{
	tepht_error_info  einfo;
	einfo.context_accessor = ctx_accessor;
	einfo.line_number = line_number;
	einfo.status = status;
	einfo.filename_index = filename_to_index(filename);
	return einfo;
}

const char *tepht_error_get_filename(tepht_error_info  einfo)
{
	if ( einfo.status == tepht_status_ok )
		return "N/A";
	else
		return index_to_filename(einfo.filename_index);
}


// TODO: I'm starting to think that this should return the number of characters
//   that WOULD be printed, not necessarily the number that were.
//   (After all, we can't know unless the caller tells us, so they know how
//   much was printed anyways, and we don't even need that information.)
//   This makes the callback simpler too: just return `void`. That's easier
//   for the caller to implement, since they don't have to worry about any
//   hidden meaning in the value that they return (or being "correct" about
//   return the right number or whatever).
//   This strategy would also allow callers to use this function to right-size
//   buffers in the event that truncation occurs. For example, if the caller
//   were to be printing to an in-memory string buffer, then they could compare
//   the return value against the string buffer's length. If the string buffer
//   was too small, then the return value will be greater than its length,
//   and the caller will know that they need to resize the buffer AND what
//   they will need to resize it to (without having to guess or anything).
//   For this to be useful, the error printer has to be pure, or at least
//   idempotent.

//const char *tepht_stringize_error(enum tepht_status error_code)
size_t tepht_print_error(tepht_error_info  einfo,  tepht_string_printing_fn  print_string,  void *printer_context)
{
	tepht_string_printer  printer_;
	tepht_string_printer  *printer = &printer_;
	tepht_string_printer_init(printer, print_string, printer_context);

	// Handle this case separately; it is special because it doesn't involve
	// file or line number printing.
	if ( einfo.status == tepht_status_ok )
		return tepht_print_array_string(printer, "Status OK, success. (tepht_status_ok)");

	size_t char_count = 0;
	char_count += tepht_print_array_string (printer, "Error reported from file ");
	char_count += tepht_print_nt_string    (printer, tepht_error_get_filename(einfo));
	char_count += tepht_print_array_string (printer, ", line ");
	char_count += tepht_print_uint64_as_dec(printer, einfo.line_number);
	char_count += tepht_print_array_string (printer, ": ");

	switch(einfo.status)
	{
		case tepht_status_ok:
			// This case was already handled by the initial check for OK/success status.
			// So this code shouldn't be reachable.
			assert(0);

#if 0
		// Unimplemented
		case tepht_status_waiting:
			char_count += tepht_print_array_string(printer, "Status OK, but waiting for sensor to respond. (tepht_status_waiting)");
			break;
#endif

		case tepht_status_null_argument:
			char_count += tepht_print_array_string(printer, "A function in the ");
			char_count += tepht_print_nt_string   (printer, tepht_get_device_model_name(*einfo.context_accessor)); // ex: MS8607
			char_count += tepht_print_array_string(printer,
				" driver was called with a NULL argument when a non-NULL argument was required. (tepht_status_null_argument)");
			break;

		case tepht_status_null_sensor:
			char_count += tepht_print_array_string(printer, "A function in the ");
			char_count += tepht_print_nt_string   (printer, tepht_get_device_model_name(*einfo.context_accessor)); // ex: MS8607
			char_count += tepht_print_array_string(printer, " driver was given a NULL pointer to a `");
			char_count += tepht_print_nt_string   (printer, tepht_get_driver_prefix(*einfo.context_accessor)); // ex: ms8607_sensor
			char_count += tepht_print_array_string(printer, "_sensor` object. These are required to be non-NULL. (tepht_status_null_sensor)");
			break;

		case tepht_status_null_host_function:
			// TODO: This might not need a prefix parameterization if *_host_functions gets unified.
			char_count += tepht_print_array_string(printer, "A member of the `");
			char_count += tepht_print_nt_string   (printer, tepht_get_driver_prefix(*einfo.context_accessor)); // ex: ms8607_host_functions
			char_count += tepht_print_array_string(printer,
				"_host_functions` structure was NULL. This could lead to crashing and "
				"unpredictable behavior later on. (tepht_status_null_host_function)");
			break;

		case tepht_status_callback_error:
			// TODO: This might not need a prefix parameterization if *_host_functions gets unified.
			char_count += tepht_print_array_string(printer,
				"Error occurred within a function dispatched from the `");
			char_count += tepht_print_nt_string   (printer, tepht_get_driver_prefix(*einfo.context_accessor)); // ex: ms8607_host_functions
			char_count += tepht_print_array_string(printer,
				"_host_functions` structure. (tepht_status_callback_error)");
			break;

		case tepht_status_callback_i2c_nack:
			char_count += tepht_print_array_string(printer,
				"I2C transfer did not complete. Peripheral responded with NACK. (tepht_status_callback_i2c_nack)");
			break;

		case tepht_status_eeprom_is_zero: // Formerly ms8607_status_crc_error
			char_count += tepht_print_array_string(printer,
				"One or more EEPROM coefficients were received as 0, preventing measurement. (tepht_status_eeprom_is_zero)");
			break;

		case tepht_status_eeprom_crc_error: // Formerly ms8607_status_crc_error
			char_count += tepht_print_array_string(printer,
				"EEPROM coefficients retrieved from the sensor did not pass CRC check. (tepht_status_eeprom_crc_error)");
			break;

		case tepht_status_measurement_invalid: // Formerly ms8607_status_i2c_transfer_error
			char_count += tepht_print_array_string(printer,
				"EEPROM is OK and I2C transfer completed, but data received was invalid. (tepht_status_measurement_invalid)");
			break;

		case tepht_status_heater_on_error:
			char_count += tepht_print_array_string(printer,
				"Cannot compute compensated humidity because heater is on. (tepht_status_heater_on_error)");
			break;

		// TODO: Replace the `ms8607` prefix on the below symbols after *_host_functions gets unified.
		case tepht_status_i2c_read_unimplemented:
			char_count += tepht_print_array_string(printer,
				"An implementation for the `ms8607_host_functions.i2c_controller_read`"
				" function was not provided, but was needed to complete an operation.");
			break;

		case tepht_status_i2c_write_unimplemented:
			char_count += tepht_print_array_string(printer,
				"An implementation for the `ms8607_host_functions.i2c_controller_write`"
				" function was not provided, but was needed to complete an operation.");
			break;

		case tepht_status_i2c_write_no_stop_unimplemented:
			char_count += tepht_print_array_string(printer,
				"An implementation for the `ms8607_host_functions.i2c_controller_write_no_stop`"
				" function was not provided, but was needed to complete an operation.");
			break;

		case tepht_status_sleep_ms_unimplemented:
			char_count += tepht_print_array_string(printer,
				"An implementation for the `ms8607_host_functions.sleep_ms`"
				" function was not provided, but was needed to complete an operation.");
			break;

		default:
			char_count += tepht_print_array_string(printer, "Error code ");
			char_count += tepht_print_int64_as_dec(printer, einfo.status);
			char_count += tepht_print_array_string(printer, " is not a valid tepht_status.");
			break;
	}
	return char_count;
}

void tepht_buffer_printer_init(tepht_buffer_printer *printer, char *buffer, size_t buffer_size)
{
	assert(printer != NULL);
	assert(buffer != NULL);
	printer->buffer = buffer;
	printer->buflen = buffer_size;
	printer->cursor = 0;

	if ( buffer_size >= 1 )
		buffer[0] = '\0';
}

void tepht_buffer_printer_print_string(void *printer_vptr, const char *str, size_t len)
{
	assert(printer_vptr != NULL);
	if ( str == NULL || len == 0 )
		return;

	tepht_buffer_printer  *printer = printer_vptr;

	char   *cursor_ptr      = printer->buffer + printer->cursor;
	size_t buffer_remaining = (printer->buflen - printer->cursor) - 1;

	size_t copy_size = len;
	if ( copy_size > buffer_remaining )
		copy_size = buffer_remaining;

	// Do this FIRST.
	// The order of operations could potentially be important if other threads
	// can access the buffer during the printing operation. That situation
	// would still end up with some UB(?) due to the poorly-specified state
	// of the next buffer region during the `memcpy` operation, but at least
	// we've prevented even nastier failure modes (buffer overrun) by ensuring
	// that there is ALWAYS a null-terminator in the buffer, regardless of
	// what point in execution we're at.
	cursor_ptr[copy_size] = '\0';

	// Now that we've guaranteed a null-terminator at the end of our next
	// growth, we can proceed with the actual copying.
	memcpy(cursor_ptr, str, copy_size);

	// ... and update our own state.
	printer->cursor += copy_size;
}

char *tepht_buffer_printer_get_data(tepht_buffer_printer *printer, size_t *result_size)
{
	assert(printer != NULL);
	if ( result_size != NULL )
		*result_size = printer->cursor;

	return printer->buffer;
}

size_t tepht_buffer_printer_get_byte_count(tepht_buffer_printer *printer)
{
	assert(printer != NULL);
	return printer->cursor;
}

void tepht_buffer_printer_clear(tepht_buffer_printer *printer)
{
	assert(printer != NULL);
	char   *buffer = printer->buffer;
	size_t buflen  = printer->buflen;
	tepht_buffer_printer_init(printer, buffer, buflen);
}

static void unittest_buffer_printer(tepht_string_printer  *printer)
{
	tepht_buffer_printer  bufprinter_;
	tepht_buffer_printer  *bufprinter = &bufprinter_;
	char buffer[8];
	size_t buflen = sizeof(buffer);
	size_t len = 0;

	tepht_print_array_string(printer, "  unittest_buffer_printer:");

	tepht_buffer_printer_init(bufprinter, buffer, buflen);
	assert(buffer[0] == '\0');
	assert(0 == tepht_buffer_printer_get_byte_count(bufprinter));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,NULL), ""));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,&len), ""));
	assert(0 == len);

	tepht_buffer_printer_print_string(bufprinter, "Hello", 5);
	assert(buffer[0] == 'H');
	assert(buffer[4] == 'o');
	assert(buffer[5] == '\0');
	assert(5 == tepht_buffer_printer_get_byte_count(bufprinter));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,NULL), "Hello"));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,&len), "Hello"));
	assert(5 == len);

	tepht_buffer_printer_print_string(bufprinter, " world!", 7);
	assert(buffer[0] == 'H');
	assert(buffer[4] == 'o');
	assert(buffer[5] == ' ');
	assert(buffer[6] == 'w');
	assert(buffer[7] == '\0');
	assert(7 == tepht_buffer_printer_get_byte_count(bufprinter));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,NULL), "Hello w"));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,&len), "Hello w"));
	assert(7 == len);

	tepht_buffer_printer_clear(bufprinter);
	assert(buffer[0] == '\0');
	assert(0 == tepht_buffer_printer_get_byte_count(bufprinter));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,NULL), ""));

	tepht_buffer_printer_print_string(bufprinter, "Have no doubt", 13);
	assert(buffer[0] == 'H');
	assert(buffer[4] == ' ');
	assert(buffer[5] == 'n');
	assert(buffer[6] == 'o');
	assert(buffer[7] == '\0');
	assert(7 == tepht_buffer_printer_get_byte_count(bufprinter));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,NULL), "Have no"));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,&len), "Have no"));
	assert(7 == len);

	tepht_buffer_printer_clear(bufprinter);
	assert(buffer[0] == '\0');
	assert(0 == tepht_buffer_printer_get_byte_count(bufprinter));
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,NULL), ""));

	
	tepht_print_array_string(printer, "  Passed.\n");
}

static void unittest_print_error(tepht_string_printer  *printer)
{
	tepht_buffer_printer  bufprinter_;
	tepht_buffer_printer  *bufprinter = &bufprinter_;

	tepht_driver_context_accessor  ctx;

	// Just zeroing out the accessor is simpler than actually creating
	// a mock driver and accessor just to test error printing.
	// (And using the actual drivers/accessors would create a circular depends; yuck.)
	// This means we can't use any of the error messages that call the methods
	// of the accessor. So be careful about that when writing tests:
	// always look at what `tepht_print_error` is going to do when it
	// receives the given error code, to make sure we don't pass it a code
	// that will make it call into the accessor!
	memset(&ctx, 0, sizeof(tepht_driver_context_accessor));

	char buffer[1024];
	size_t buflen = sizeof(buffer);
	tepht_buffer_printer_init(bufprinter, buffer, buflen);

	tepht_error_info  einfo;

	tepht_print_array_string(printer, "  unittest_print_error:");

	einfo = tepht_success(&ctx);
	tepht_print_error(einfo, &tepht_buffer_printer_print_string, bufprinter);
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,NULL), "Status OK, success. (tepht_status_ok)"));
	tepht_buffer_printer_clear(bufprinter);

	size_t line_capture = 0;
	einfo = tepht_error(&ctx, tepht_status_callback_i2c_nack); line_capture = __LINE__;
	if ( einfo.line_number == line_capture ) {
		// Pick some value that can't be an actual line number, but is
		// predictable so that we can hardcode it into the string to test against.
		einfo.line_number = 31337;
	}
	tepht_print_error(einfo, &tepht_buffer_printer_print_string, bufprinter);
	assert(0 == strcmp(tepht_buffer_printer_get_data(bufprinter,NULL),
		"Error reported from file " __FILE__ ", line 31337: "
		"I2C transfer did not complete. "
		"Peripheral responded with NACK. "
		"(tepht_status_callback_i2c_nack)"));
	tepht_buffer_printer_clear(bufprinter);

	tepht_print_array_string(printer, "  Passed.\n");
}


tepht_bool  tepht_pt_sensor_is_connected(tepht_pt_sensor  sensor,  void *caller_context)
{
	assert(sensor.self != NULL);
	assert(sensor.vtable->is_connected != NULL);
	return sensor.vtable->is_connected(sensor.self, caller_context);
}

tepht_error_info  tepht_pt_sensor_reset(tepht_pt_sensor  sensor,  void *caller_context)
{
	assert(sensor.self != NULL);
	assert(sensor.vtable->reset != NULL);
	return sensor.vtable->reset(sensor.self, caller_context);
}

tepht_error_info  tepht_pt_sensor_read_temperature_pressure_int32(
	tepht_pt_sensor  sensor, int32_t *t, int32_t *p, void *caller_context)
{
	assert(sensor.self != NULL);
	assert(sensor.vtable->read_temperature_pressure_int32 != NULL);
	return sensor.vtable->read_temperature_pressure_int32(sensor.self, t, p, caller_context);
}

void tepht_run_unittests(tepht_string_printer  *printer)
{
	tepht_print_array_string(printer, "Running unittests for " __FILE__ ":\n");
	unittest_int_to_str(printer);
	unittest_filename_indexes(printer);
	unittest_buffer_printer(printer);
	unittest_print_error(printer);
	tepht_print_array_string(printer, "\n");
}

#ifdef __cplusplus
}
#endif
