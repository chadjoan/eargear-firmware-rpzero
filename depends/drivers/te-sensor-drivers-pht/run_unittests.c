#include <stdio.h>
#include "tepht_common.h"

static void print(void *printer_context, const char* str, size_t len)
{
	(void)printer_context; // Because `stdout` is globally defined.
	printf("%.*s", (int)len, str);
}

int main(int argc, char *argv[])
{
	(void)argc; (void)argv;
	tepht_string_printer  printer_;
	tepht_string_printer  *printer = &printer_;
	tepht_string_printer_init(printer, &print, NULL);

	tepht_run_unittests(printer);

	return 0;
}
