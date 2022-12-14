
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
static char *tepht_uint64_to_string_impl(uint64_t num, uint8_t *output_len, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with, uint8_t base)
{
	uint_fast8_t  i = 0;

	// Write the number out.
	// This will put it out in reverse order, so we'll have to reverse it later.
	uint_fast8_t  digitStart = i;
	while(num > 0 && i+1 < buflen)
	{
		uint_fast8_t  digit = (uint_fast8_t)(num % base);
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

	// This loop also handles the case where `num` is 0, and we (usually)
	// just need to output a single '0' character. But this will also handle
	// padding functionality, as a generalization of that case.
	while(i < min_width) {
		buffer[i++] = pad_with;
	}

	// Reverse/swizzle
	uint_fast8_t  digitEnd = i;
	uint_fast8_t  halfway  = (digitEnd - digitStart) / 2;
	uint_fast8_t  lastDigit = digitEnd-1;
	for ( i = 0; i < halfway; i++ )
	{
		uint_fast8_t  lo = digitStart + i;
		uint_fast8_t  hi = lastDigit - i;

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
char *tepht_int64_to_string(int64_t num, uint8_t *output_len, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with, uint8_t base)
{
	assert( 2 <= base && base <= 36 );

	assert(buffer != NULL);
	assert(buflen >= 65 || base !=  2); // 1 sign + 63 bits + 1 null terminator
	assert(buflen >= 21 || base != 10); // 1 sign + 19 digits + 1 null terminator
	assert(buflen >= 18 || base != 16); // 1 sign + 16 hexadigits + 1 null terminator

	// -1 for the null terminator at the end of the output, then another -1 for the possible minus sign.
	assert(min_width <= buflen-2);

	uint8_t impl_output_len;

	if ( num >= 0 )
	{
		tepht_uint64_to_string_impl(num, &impl_output_len, buffer, buflen, min_width, pad_with, base);
	}
	else
	{
		num = -num;
		buffer[0] = '-';

		char *digits = tepht_uint64_to_string_impl(num, &impl_output_len, buffer+1, buflen-1, min_width, pad_with, base);
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
char *tepht_uint64_to_string(uint64_t num, uint8_t *output_len, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with, uint8_t base)
{
	uint8_t dummy_output;
	if ( output_len == NULL )
		output_len = &dummy_output;

	assert( 2 <= base && base <= 36 );

	// -1 for the null terminator at the end of the output.
	assert(min_width <= buflen-1);

	assert(buffer != NULL);
	assert(buflen >= 65 || base !=  2); // 64 bits + 1 null terminator
	assert(buflen >= 21 || base != 10); // 20 digits + 1 null terminator
	assert(buflen >= 17 || base != 16); // 16 hexadigits + 1 null terminator

	return tepht_uint64_to_string_impl(num, output_len, buffer, buflen, min_width, pad_with, base);
}

///
char *tepht_int64_to_binary(int64_t num, uint8_t *outlen, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with)
{
	return tepht_int64_to_string(num, outlen, buffer, buflen, min_width, pad_with, 2);
}

///
char *tepht_int64_to_binary_default(int64_t num, uint8_t *outlen, char *buffer, uint8_t buflen)
{
	return tepht_int64_to_string(num, outlen, buffer, buflen, 1, '0', 2);
}

///
char *tepht_uint64_to_binary(uint64_t num, uint8_t *outlen, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with)
{
	return tepht_uint64_to_string(num, outlen, buffer, buflen, min_width, pad_with, 2);
}

///
char *tepht_uint64_to_binary_default(uint64_t num, uint8_t *outlen, char *buffer, uint8_t buflen)
{
	return tepht_uint64_to_string(num, outlen, buffer, buflen, 1, '0', 2);
}

///
char *tepht_int64_to_octal(int64_t num, uint8_t *outlen, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with)
{
	return tepht_int64_to_string(num, outlen, buffer, buflen, min_width, pad_with, 8);
}

///
char *tepht_int64_to_octal_default(int64_t num, uint8_t *outlen, char *buffer, uint8_t buflen)
{
	return tepht_int64_to_string(num, outlen, buffer, buflen, 1, '0', 8);
}

///
char *tepht_uint64_to_octal(uint64_t num, uint8_t *outlen, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with)
{
	return tepht_uint64_to_string(num, outlen, buffer, buflen, min_width, pad_with, 8);
}

///
char *tepht_uint64_to_octal_default(uint64_t num, uint8_t *outlen, char *buffer, uint8_t buflen)
{
	return tepht_uint64_to_string(num, outlen, buffer, buflen, 1, '0', 8);
}

///
char *tepht_int64_to_dec(int64_t num, uint8_t *outlen, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with)
{
	return tepht_int64_to_string(num, outlen, buffer, buflen, min_width, pad_with, 10);
}

///
char *tepht_int64_to_dec_default(int64_t num, uint8_t *outlen, char *buffer, uint8_t buflen)
{
	return tepht_int64_to_string(num, outlen, buffer, buflen, 1, '0', 10);
}

///
char *tepht_uint64_to_dec(uint64_t num, uint8_t *outlen, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with)
{
	return tepht_uint64_to_string(num, outlen, buffer, buflen, min_width, pad_with, 10);
}

///
char *tepht_uint64_to_dec_default(uint64_t num, uint8_t *outlen, char *buffer, uint8_t buflen)
{
	return tepht_uint64_to_string(num, outlen, buffer, buflen, 1, '0', 10);
}

///
char *tepht_int64_to_hex(int64_t num, uint8_t *outlen, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with)
{
	return tepht_int64_to_string(num, outlen, buffer, buflen, min_width, pad_with, 16);
}

///
char *tepht_int64_to_hex_default(int64_t num, uint8_t *outlen, char *buffer, uint8_t buflen)
{
	return tepht_int64_to_string(num, outlen, buffer, buflen, 1, '0', 16);
}

///
char *tepht_uint64_to_hex(uint64_t num, uint8_t *outlen, char *buffer, uint8_t buflen, uint8_t min_width, char pad_with)
{
	return tepht_uint64_to_string(num, outlen, buffer, buflen, min_width, pad_with, 16);
}

///
char *tepht_uint64_to_hex_default(uint64_t num, uint8_t *outlen, char *buffer, uint8_t buflen)
{
	return tepht_uint64_to_string(num, outlen, buffer, buflen, 1, '0', 16);
}

static void unittest_int_to_str(tepht_string_printer  *printer)
{
	char buffer[72];
	uint8_t buflen = sizeof(buffer);
	uint8_t outlen = 0;

	tepht_print_array_string(printer, "  unittest_int_to_str, for the tepht_int64_to_string(...) function:");

	assert(0 == strcmp(tepht_int64_to_dec_default(0  , NULL, buffer, buflen), "0"  ));
	assert(0 == strcmp(tepht_int64_to_dec_default(-0 , NULL, buffer, buflen), "0"  ));
	assert(0 == strcmp(tepht_int64_to_dec_default(32 , NULL, buffer, buflen), "32" ));
	assert(0 == strcmp(tepht_int64_to_dec_default(31 , NULL, buffer, buflen), "31" ));
	assert(0 == strcmp(tepht_int64_to_dec_default(-8 , NULL, buffer, buflen), "-8" ));
	assert(0 == strcmp(tepht_int64_to_dec_default(-32, NULL, buffer, buflen), "-32"));

	assert(0 == strcmp(tepht_int64_to_dec_default(0  , &outlen, buffer, buflen), "0"  )); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_dec_default(-0 , &outlen, buffer, buflen), "0"  )); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_dec_default(32 , &outlen, buffer, buflen), "32" )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_dec_default(31 , &outlen, buffer, buflen), "31" )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_dec_default(-8 , &outlen, buffer, buflen), "-8" )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_dec_default(-32, &outlen, buffer, buflen), "-32")); assert(outlen == 3);

	assert(0 == strcmp(tepht_int64_to_hex_default(0  , &outlen, buffer, buflen), "0"  )); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_hex_default(-0 , &outlen, buffer, buflen), "0"  )); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_hex_default(32 , &outlen, buffer, buflen), "20" )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex_default(31 , &outlen, buffer, buflen), "1F" )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex_default(-8 , &outlen, buffer, buflen), "-8" )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex_default(-32, &outlen, buffer, buflen), "-20")); assert(outlen == 3);

	assert(0 == strcmp(tepht_int64_to_hex_default(10, &outlen, buffer, buflen), "A")); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_hex_default(11, &outlen, buffer, buflen), "B")); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_hex_default(12, &outlen, buffer, buflen), "C")); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_hex_default(13, &outlen, buffer, buflen), "D")); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_hex_default(14, &outlen, buffer, buflen), "E")); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_hex_default(15, &outlen, buffer, buflen), "F")); assert(outlen == 1);

	assert(0 == strcmp(tepht_int64_to_hex(0  , &outlen, buffer, buflen, 2, '0'), "00"  )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex(-0 , &outlen, buffer, buflen, 2, '0'), "00"  )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex(32 , &outlen, buffer, buflen, 2, '0'), "20" )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex(31 , &outlen, buffer, buflen, 2, '0'), "1F" )); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex(-8 , &outlen, buffer, buflen, 2, '0'), "-08" )); assert(outlen == 3);
	assert(0 == strcmp(tepht_int64_to_hex(-32, &outlen, buffer, buflen, 2, '0'), "-20")); assert(outlen == 3);

	assert(0 == strcmp(tepht_int64_to_hex(10, &outlen, buffer, buflen, 2, '0'), "0A")); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex(11, &outlen, buffer, buflen, 2, '0'), "0B")); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex(12, &outlen, buffer, buflen, 2, '0'), "0C")); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex(13, &outlen, buffer, buflen, 2, '0'), "0D")); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex(14, &outlen, buffer, buflen, 2, '0'), "0E")); assert(outlen == 2);
	assert(0 == strcmp(tepht_int64_to_hex(15, &outlen, buffer, buflen, 2, '0'), "0F")); assert(outlen == 2);

	assert(0 == strcmp(tepht_int64_to_binary_default(0  , &outlen, buffer, buflen), "0"      )); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_binary_default(-0 , &outlen, buffer, buflen), "0"      )); assert(outlen == 1);
	assert(0 == strcmp(tepht_int64_to_binary_default(32 , &outlen, buffer, buflen), "100000" )); assert(outlen == 6);
	assert(0 == strcmp(tepht_int64_to_binary_default(31 , &outlen, buffer, buflen), "11111"  )); assert(outlen == 5);
	assert(0 == strcmp(tepht_int64_to_binary_default(-8 , &outlen, buffer, buflen), "-1000"  )); assert(outlen == 5);
	assert(0 == strcmp(tepht_int64_to_binary_default(-32, &outlen, buffer, buflen), "-100000")); assert(outlen == 7);

	// Test what happens if we force an overflow/truncation.
	char buffer3[3];
	const char *outstr = NULL;
	outstr = tepht_uint64_to_string_impl(123, &outlen, buffer3, 3, 1, '0', 10);
	assert(outlen == 2);
	assert(outstr[0] == '2');
	assert(outstr[1] == '3');
	assert(outstr[2] == '\0');

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
	uint8_t buflen = sizeof(buffer);
	uint8_t len = 0;
	char *str = tepht_int64_to_dec_default(number, &len, buffer, buflen);
	printer->print_string(printer->printer_context, str, len);
	return len;
}

// Prints an integer using the given string printer.
size_t tepht_print_uint64_as_dec(tepht_string_printer  *printer, uint64_t number)
{
	char buffer[24];
	uint8_t buflen = sizeof(buffer);
	uint8_t len = 0;
	char *str = tepht_uint64_to_dec_default(number, &len, buffer, buflen);
	printer->print_string(printer->printer_context, str, len);
	return len;
}

#if 0
// This could be a powerful way to store file/line/function info in
// a compiler-and-platform-independent way, but right-sizing the arrays
// is too tricky for now. 
static const char *tepht_filenames[64];
static const char *tepht_function_names[196];

#if 0
typedef struct tepht_debug_frame {
	uint16_t  word1;
	uint16_t  word2;
	uint16_t  word3;
} tepht_debug_frame;

static tepht_debug_frame  tepht_debug_frame[TEPHT_DEBUG_FRAME_DB_ROW_CAPACITY];

#define tepht_dframe_to_u64(dframe) ( \
	((uint64_t)(dframe.word1) <<  0) | \
	((uint64_t)(dframe.word2) << 16) | \
	((uint64_t)(dframe.word3) << 32) )
#endif

static uint16_t  tepht_debug_frame_db[TEPHT_DEBUG_FRAME_DB_ROW_CAPACITY*3];

// Below we have a bitfield layout that supports:
//
// Files with up to (2^18 = 262144) lines.
// Function count (project-wide) up to (2^18 = 262144).
// File count up to (2^12 = 4096).
//
// And so, as long as the project stays within those limits, we can save some
// memory and store every line of debug information with about 48 bits each
// (not including the string pointers referenced by these handles --
// which is a separate problem).
//

#define tepht_dframe_to_u64(d0,d1,d2) ( \
	((uint64_t)(d0) <<  0) | \
	((uint64_t)(d1) << 16) | \
	((uint64_t)(d2) << 32) )

#define tepht_dframe_get_line_number(d0,d1,d2)    ((tepht_dframe_to_u64(d0,d1,d2) & 0x00000003FFFFu) >>  0)
#define tepht_dframe_get_function_handle(d0,d1,d2) ((tepht_dframe_to_u64(d0,d1,d2) & 0x000FFFFC0000u) >> 18)
#define tepht_dframe_get_filename_handle(d0,d1,d2) ((tepht_dframe_to_u64(d0,d1,d2) & 0xFFF000000000u) >> 36)

size_t  tepht_upsert_debug_filename(const char *filename)
{
	// Ugh datastructures.
	// Need a sublinear find-and-insert-as-needed algorithm.
}

tepht_dframe_handle  tepht_upsert_debug_location_to_dframe(const char *filename, const char *function_name, size_t  line_number)
{
	
}

#endif


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

static const char *get_basename(const char *filename)
{
	if ( filename == NULL )
		return NULL;

	size_t     i = 0;
	const char *basename_start = filename;

	while(filename[i] != '\0')
	{
		if ( filename[i] == '/' )
			basename_start = &filename[i+1];
		i++;
	}

	return basename_start;
}

static void unittest_get_basename(tepht_string_printer  *printer)
{
	tepht_print_array_string(printer, "  unittest_get_basename:");

	assert( NULL == get_basename(NULL) );
	assert( 0 == strcmp("", get_basename("")) );
	assert( 0 == strcmp("foo", get_basename("foo")) );
	assert( 0 == strcmp("bar", get_basename("foo/bar")) );
	assert( 0 == strcmp("baz", get_basename("foo/bar/baz")) );
	assert( 0 == strcmp("qux", get_basename("/qux")) );
	assert( 0 == strcmp("qux", get_basename("/foo/qux")) );

	assert( 0 == strcmp("ms8607.c", get_basename("ms8607.c")) );
	assert( 0 == strcmp("ms8607.c", get_basename("te-sensor-drivers-pht/ms8607.c")) );
	assert( 0 == strcmp("ms8607.c", get_basename("some/path/te-sensor-drivers-pht/ms8607.c")) );
	assert( 0 == strcmp("ms8607.c", get_basename("/abs/path/te-sensor-drivers-pht/ms8607.c")) );

	tepht_print_array_string(printer, "  Passed.\n");
}

static uint8_t filename_to_index(const char *filename)
{
	const char *basename = get_basename(filename);
	uint8_t len = (sizeof(tepht_filenames) / sizeof(tepht_filenames[0]));
	uint8_t i;
	for ( i = 0; i < len; i++ )
	{
		const char *candidate = tepht_filenames[i];
		if ( 0 == strcmp(basename, candidate) )
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
	assert(2 == filename_to_index("te-sensor-drivers-pht/ms8607.c"));
	assert(2 == filename_to_index("some/path/te-sensor-drivers-pht/ms8607.c"));
	assert(2 == filename_to_index("/abs/path/te-sensor-drivers-pht/ms8607.c"));

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


static tepht_status_u16
	i2c_controller_read_unimpl(void *caller_context, tepht_i2c_controller_packet *const packet)
{
	(void)caller_context;
	(void)packet;
	return tepht_status_i2c_read_unimplemented;
}

static tepht_status_u16
	i2c_controller_write_unimpl(void *caller_context, tepht_i2c_controller_packet *const packet)
{
	(void)caller_context;
	(void)packet;
	return tepht_status_i2c_write_unimplemented;
}

static tepht_status_u16
	i2c_controller_write_no_stop_unimpl(void *caller_context, tepht_i2c_controller_packet *const packet)
{
	(void)caller_context;
	(void)packet;
	return tepht_status_i2c_write_no_stop_unimplemented;
}

static tepht_status_u16
	sleep_ms_unimpl(void *caller_context, uint32_t milliseconds)
{
	(void)caller_context;
	(void)milliseconds;
	return tepht_status_sleep_ms_unimplemented;
}

static void print_string_stub(void *caller_context, const char *text)
{
	(void)caller_context;
	(void)text;
}

static void print_int64_stub(void *caller_context,  int64_t number, uint8_t pad_width,  tepht_bool  pad_with_zeroes)
{
	(void)caller_context;
	(void)number;
	(void)pad_width;
	(void)pad_with_zeroes;
}

/// \brief Initializes the `ms8607_host_functions` struct; this should be called
///        *before* assigning function pointers into the structure.
///
static tepht_error_info  tepht_init_host_functions(tepht_host_functions *deps)
{
	if ( deps == NULL )
		return tepht_error(NULL, tepht_status_null_argument);

	deps->validated_ = 0;

	deps->i2c_controller_read           = &i2c_controller_read_unimpl;
	deps->i2c_controller_write          = &i2c_controller_write_unimpl;
	deps->i2c_controller_write_no_stop  = &i2c_controller_write_no_stop_unimpl;
	deps->sleep_ms                      = &sleep_ms_unimpl;
	deps->print_string                  = &print_string_stub;
	deps->print_int64                   = &print_int64_stub;

	return tepht_success(NULL);
}

tepht_status  tepht_validate_mandatory_depends(tepht_host_functions *deps)
{
	assert(deps != NULL);

	if ( deps->validated_ )
		return tepht_status_ok;

	if ( deps->i2c_controller_read == NULL
	||   deps->i2c_controller_read == &i2c_controller_read_unimpl )
		return tepht_status_i2c_read_unimplemented;
	else
	if ( deps->i2c_controller_write == NULL
	||   deps->i2c_controller_write == &i2c_controller_write_unimpl )
		return tepht_status_i2c_write_unimplemented;
	/*
	// It's actually OK to be missing the `i2c_controller_write_no_stop` function.
	// The write-no-stop function is only needed for hold-mode ADC.
	// So we shouldn't validate this one until the caller actually tries to
	// use the hold-mode feature. In most cases, the caller won't need this,
	// and in some of those cases, the caller *can't* provide this (ex:
	// because their MCU's I2C peripheral doesn't allow sending a transmit
	// sequence without the trailing `stop` bit).
	// TODO: Maybe a -D macro could be provided to enable early checking of this?
	// TODO: Sort of like USE-flags on Gentoo systems... USE_HOLD implies RDEPENDS={$RDEPENDS, write-no-stop}
	else
	if ( deps->i2c_controller_write_no_stop == NULL
	||   deps->i2c_controller_write_no_stop == &i2c_controller_write_no_stop_unimpl )
		return tepht_status_i2c_write_no_stop_unimplemented;
	*/
	else
	if ( deps->sleep_ms == NULL
	||   deps->sleep_ms == &sleep_ms_unimpl )
		return tepht_status_sleep_ms_unimplemented;

	// Under no condition should any of the function pointers be NULL.
	// Even unimplemented things should be assigned error handlers or stubs
	// by the `ms8607_init_host_functions` function.
	if( deps->i2c_controller_read == NULL
	||  deps->i2c_controller_write == NULL
	||  deps->i2c_controller_write_no_stop == NULL
	||  deps->sleep_ms == NULL
	||  deps->print_string == NULL
	||  deps->print_int64 == NULL )
		return tepht_status_null_host_function;

	// If we've made it to the end of this function, then it is at least plausible
	// that the driver can use this host functions object to do *something*.
	// (More specific features of the driver might require functions that we
	// didn't check here. However, absent a more complicated configuration system,
	// we'll just have to check those later, at point-of-use.)
	deps->validated_ = 1;
	return tepht_status_ok;
}

//
// TODO: Finish rewriting docs; might need to move some (or all) validation
//       into the `*_init_sensor` functions. (Probably not all of it.
//       library-wide universal dependencies should be checked as early
//       as possible!)
tepht_error_info  tepht_init_and_assign_host_functions(
	tepht_host_functions *dependencies,
	void *caller_context,
	void (*assign_functions)(tepht_host_functions *dependencies, void *caller_context)
	)
{
	tepht_error_info  einfo;

	// The call to `tepht_init_host_functions` will enforce that `dependencies` is non-NULL.
	einfo = tepht_init_host_functions(dependencies);
	if ( tepht_is_error(einfo) )
		return einfo;

	// It's OK if `caller_context` is NULL.
	// Whether that's required to be non-NULL or not is up to the caller, so
	// they would have to enforce that from within `assign_functions`, if they
	// wanted such a thing.
	// (`caller_context` being NULL is actually pretty likely, in this case!
	// The host functions are likely to be known at compile-time, so it
	// would be unnecessary to use the dynamic (run-time) information
	// referenced by the `caller_context` object to compute their values.)

	assign_functions(dependencies, caller_context);

	tepht_status  status = tepht_validate_mandatory_depends(dependencies);
	if ( status != tepht_status_ok )
		return tepht_error(NULL, status);

	return tepht_success(NULL);
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
	unittest_get_basename(printer);
	unittest_filename_indexes(printer);
	unittest_buffer_printer(printer);
	unittest_print_error(printer);
	tepht_print_array_string(printer, "\n");
}

#ifdef __cplusplus
}
#endif
