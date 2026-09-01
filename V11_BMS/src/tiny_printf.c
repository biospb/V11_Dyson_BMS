/*
 * tiny_printf.c
 *
 * Minimal integer-only snprintf replacement - see tiny_printf.h.
 *
 * License: GNU GPL v3 or later
 */

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "tiny_printf.h"

#if TINY_PRINTF_ENABLE

#include <stdbool.h>
#include <stdint.h>

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
/* 4294967295 is the longest unsigned 32-bit rendering */
#define TP_MAX_DIGITS   10

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/
typedef struct
{
  char   *buf;
  size_t  size;   /* total capacity of buf, including the terminating NUL */
  size_t  len;    /* length the complete output would need, as snprintf reports */
} tp_out_t;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static void tp_putc(tp_out_t *o, char c);
static void tp_puts(tp_out_t *o, const char *s);
static void tp_putu(tp_out_t *o, uint32_t v, uint8_t base, uint8_t width,
                    bool zero_pad, bool upper, bool neg);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
 * @brief snprintf-compatible formatter for the supported conversion subset.
 *
 * @param buf   Destination buffer (may be NULL only when size is 0).
 * @param size  Capacity of buf including the terminating NUL.
 * @param fmt   Format string.
 * @param ap    Argument list.
 * @return      Length the complete output would have needed, excluding the NUL.
 */
int tiny_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap)
{
  tp_out_t o;

  o.buf  = buf;
  o.size = size;
  o.len  = 0u;

  while (*fmt != '\0')
  {
    bool     zero_pad;
    bool     is_long;
    uint8_t  width;
    uint32_t uval;
    int32_t  sval;

    if (*fmt != '%')
    {
      tp_putc(&o, *fmt);
      fmt++;
      continue;
    }
    fmt++;

    /* flags - only '0' is supported, which is all the firmware uses */
    zero_pad = false;
    while (*fmt == '0')
    {
      zero_pad = true;
      fmt++;
    }

    /* field width */
    width = 0u;
    while ((*fmt >= '0') && (*fmt <= '9'))
    {
      width = (uint8_t)((width * 10u) + (uint8_t)(*fmt - '0'));
      fmt++;
    }

    /* length modifier - 'l' and 'll' both mean 32 bit on this target */
    is_long = false;
    while (*fmt == 'l')
    {
      is_long = true;
      fmt++;
    }

    switch (*fmt)
    {
      case 'd':
      case 'i':
        sval = is_long ? (int32_t)va_arg(ap, long) : (int32_t)va_arg(ap, int);
        if (sval < 0)
        {
          /* negate in unsigned space so INT32_MIN is handled correctly */
          uval = (uint32_t)0u - (uint32_t)sval;
        }
        else
        {
          uval = (uint32_t)sval;
        }
        tp_putu(&o, uval, 10u, width, zero_pad, false, (sval < 0));
        break;

      case 'u':
        uval = is_long ? (uint32_t)va_arg(ap, unsigned long)
                       : (uint32_t)va_arg(ap, unsigned int);
        tp_putu(&o, uval, 10u, width, zero_pad, false, false);
        break;

      case 'x':
      case 'X':
        uval = is_long ? (uint32_t)va_arg(ap, unsigned long)
                       : (uint32_t)va_arg(ap, unsigned int);
        tp_putu(&o, uval, 16u, width, zero_pad, (*fmt == 'X'), false);
        break;

      case 'c':
        tp_putc(&o, (char)va_arg(ap, int));
        break;

      case 's':
      {
        const char *s = va_arg(ap, const char *);
        tp_puts(&o, (s != NULL) ? s : "(null)");
        break;
      }

      case '%':
        tp_putc(&o, '%');
        break;

      case '\0':
        /* trailing '%' with nothing after it - stop cleanly */
        continue;

      default:
        /* unsupported conversion - echo it so the mistake is visible in the log */
        tp_putc(&o, '%');
        tp_putc(&o, *fmt);
        break;
    }

    fmt++;
  }

  if (size != 0u)
  {
    buf[(o.len < size) ? o.len : (size - 1u)] = '\0';
  }

  return (int)o.len;
}

/**
 * @brief Variadic wrapper around tiny_vsnprintf().
 *
 * @param buf   Destination buffer.
 * @param size  Capacity of buf including the terminating NUL.
 * @param fmt   Format string.
 * @return      Length the complete output would have needed, excluding the NUL.
 */
int tiny_snprintf(char *buf, size_t size, const char *fmt, ...)
{
  va_list ap;
  int     n;

  va_start(ap, fmt);
  n = tiny_vsnprintf(buf, size, fmt, ap);
  va_end(ap);

  return n;
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
 * @brief Append one character, counting it even when it does not fit.
 */
static void tp_putc(tp_out_t *o, char c)
{
  if ((o->size != 0u) && (o->len < (o->size - 1u)))
  {
    o->buf[o->len] = c;
  }
  o->len++;
}

/**
 * @brief Append a NUL-terminated string.
 */
static void tp_puts(tp_out_t *o, const char *s)
{
  while (*s != '\0')
  {
    tp_putc(o, *s);
    s++;
  }
}

/**
 * @brief Append a value in the given base, with optional sign and padding.
 *
 * The minus sign counts toward the field width, as C requires: "%5d" of -426
 * is " -426", not "-  426". With the '0' flag the sign leads the zeros
 * ("%05d" of -42 is "-0042"); otherwise the spaces lead the sign ("  -42").
 */
static void tp_putu(tp_out_t *o, uint32_t v, uint8_t base, uint8_t width,
                    bool zero_pad, bool upper, bool neg)
{
  static const char lower_digits[] = "0123456789abcdef";
  static const char upper_digits[] = "0123456789ABCDEF";

  const char *digits = upper ? upper_digits : lower_digits;
  char        tmp[TP_MAX_DIGITS];
  uint8_t     n = 0u;
  uint8_t     used;
  uint8_t     pad;

  do
  {
    tmp[n] = digits[v % base];
    n++;
    v /= base;
  } while (v != 0u);

  used = neg ? (uint8_t)(n + 1u) : n;
  pad  = (width > used) ? (uint8_t)(width - used) : 0u;

  if (zero_pad)
  {
    if (neg)
    {
      tp_putc(o, '-');
    }
    while (pad != 0u)
    {
      tp_putc(o, '0');
      pad--;
    }
  }
  else
  {
    while (pad != 0u)
    {
      tp_putc(o, ' ');
      pad--;
    }
    if (neg)
    {
      tp_putc(o, '-');
    }
  }

  while (n != 0u)
  {
    n--;
    tp_putc(o, tmp[n]);
  }
}

#else /* !TINY_PRINTF_ENABLE */

/* Keep the translation unit non-empty when the formatter is compiled out. */
typedef int tiny_printf_disabled_t;

#endif /* TINY_PRINTF_ENABLE */

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
