/*
 * tiny_printf.h
 *
 * Minimal integer-only snprintf replacement for the debug logging paths.
 *
 * The C library snprintf is only ever reached from the debug log macros
 * (BMS_PRINT / DSN_PRINT and serial_debug_send_*), but it drags in roughly
 * 2.6KB of newlib: _svfprintf_r, _printf_i, _printf_common, memmove, memchr
 * and - because nano-svfprintf.c references them - the whole heap
 * (_malloc_r / _free_r / _realloc_r / _sbrk_r).
 *
 * This replacement covers only the conversions the firmware actually uses:
 *
 *     %d %i %u %x %X %c %s %%
 *     optional '0' flag, optional decimal width, optional 'l' modifier
 *
 * Set TINY_PRINTF_ENABLE to 0 in config.h to go back to the C library.
 *
 * License: GNU GPL v3 or later
 */

#ifndef TINY_PRINTF_H_
#define TINY_PRINTF_H_

/*-----------------------------------------------------------------------------
  INCLUDE FILES
-----------------------------------------------------------------------------*/
#include <stdarg.h>
#include <stddef.h>

#include "config.h"

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
#if TINY_PRINTF_ENABLE

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
/*
 * Both carry the printf format attribute so -Wformat / -Wformat=2 keep
 * checking every debug format string exactly as they did for snprintf.
 */
extern int tiny_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap)
           __attribute__((format(printf, 3, 0)));

extern int tiny_snprintf(char *buf, size_t size, const char *fmt, ...)
           __attribute__((format(printf, 3, 4)));

#define DEBUG_SNPRINTF(buf_, size_, ...)   tiny_snprintf((buf_), (size_), __VA_ARGS__)

#else /* !TINY_PRINTF_ENABLE - use the C library */

#include <stdio.h>
#define DEBUG_SNPRINTF(buf_, size_, ...)   snprintf((buf_), (size_), __VA_ARGS__)

#endif /* TINY_PRINTF_ENABLE */

/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#endif /* TINY_PRINTF_H_ */
