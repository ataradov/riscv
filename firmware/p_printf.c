//-----------------------------------------------------------------------------
//
// Simple portable vsnprintf() implementation.
//
// Copyright (c) 2013, Alex Taradov <taradov@gmail.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   1. Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
//   2. Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Limitations:
//   1. Some rarely used specifiers ('#', 'n', 'p', etc) are not implemented
//   3. Floating point is not supported
//
// Prototypes of implemented functions:
//   int p_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap);
//   int p_snprintf(char *str, size_t size, const char *fmt, ...);
//
//-----------------------------------------------------------------------------
#include <stdarg.h>
#include <stdlib.h>

//-----------------------------------------------------------------------------
//#define ENABLE_TESTS
//#define ENABLE_FLOAT

#define PUT_CHR(c) \
  do { \
    char __c = c; \
    if (buf < end) \
      *buf++ = __c; \
    nch++; \
  } while (0)

//-----------------------------------------------------------------------------
enum
{
  FL_ZERO_PAD     = 1 << 0,
  FL_LEFT_JUST    = 1 << 1,
  FL_BLANK_SIGN   = 1 << 2,
  FL_ALWAYS_SIGN  = 1 << 3,
  FL_UPPERCASE    = 1 << 4,
  FL_UNSIGNED     = 1 << 5,
  FL_INT          = 1 << 6,
  FL_DOUBLE       = 1 << 7,
  FL_STRING       = 1 << 8,
  FL_CHAR         = 1 << 9,
  FL_SHORT        = 1 << 10,
  FL_LONG         = 1 << 11,
  FL_LONG_LONG    = 1 << 12,
  FL_OCT          = 1 << 13,
  FL_HEX          = 1 << 14,
  FL_EXP          = 1 << 15,
};

//-----------------------------------------------------------------------------
static int put_int(char *buf, char *end, int flags, int width, int prec, long val)
{
  unsigned long uval = val;
  int fwidth, len = 0, nch = 0;
  char tmp[64];
  char *hex = (flags & FL_UPPERCASE) ? "0123456789ABCDEF" : "0123456789abcdef";
  int base = (flags & FL_OCT) ? 8 : (flags & FL_HEX) ? 16 : 10;
  int zero = flags & FL_ZERO_PAD ? 1 : 0;
  int ljust = flags & FL_LEFT_JUST ? 1 : 0;
  char sign = 0;

  if (0 == (flags & FL_UNSIGNED))
  {
    if (val < 0)
    {
      uval = -val;
      sign = '-';
    }
    else if (flags & FL_ALWAYS_SIGN)
      sign = '+';
    else if (flags & FL_BLANK_SIGN)
      sign = ' ';
  }

  do
  {
    tmp[len++] = hex[uval % base];
    uval /= base;
  } while (uval);

  while (len < prec)
    tmp[len++] = '0';

  fwidth = len + (sign ? 1 : 0);
  if (fwidth < width)
    fwidth = width;

  while (!zero && !ljust && (fwidth > (len + (sign ? 1 : 0))))
  {
    PUT_CHR(' ');
    fwidth--;
  }

  if (sign)
  {
    PUT_CHR(sign);
    fwidth--;
  }

  while (zero & !ljust && fwidth > len)
  {
    PUT_CHR('0');
    fwidth--;
  }

  while (len)
  {
    PUT_CHR(tmp[--len]);
    fwidth--;
  }

  while (ljust && fwidth)
  {
    PUT_CHR(' ');
    fwidth--;
  }

  return nch;
}

//-----------------------------------------------------------------------------
#ifdef ENABLE_FLOAT
static int put_double(char *buf, char *end, int flags, int width, int prec, double val)
{
  // Not implemented
  return 0;
}
#endif

//-----------------------------------------------------------------------------
static int put_string(char *buf, char *end, int flags, int width, char *val)
{
  int i, fwidth, len = 0, nch = 0;
  int ljust = flags & FL_LEFT_JUST ? 1 : 0;

  if (!val)
    val = "(null)";

  while (val[len])
    len++;

  fwidth = len;
  if (fwidth < width)
    fwidth = width;

  while (!ljust && (fwidth > len))
  {
    PUT_CHR(' ');
    fwidth--;
  }

  for (i = 0; i < len; i++)
  {
    PUT_CHR(val[i]);
    fwidth--;
  }

  while (ljust && fwidth)
  {
    PUT_CHR(' ');
    fwidth--;
  }

  return nch;
}

//-----------------------------------------------------------------------------
int p_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap)
{
  int nr, flags, width, prec, nch = 0;
  char *end = buf + size;
  char c;

  while (*fmt)
  {
    c = *fmt++;

    if (c != '%')
    {
      PUT_CHR(c);
      continue;
    }

    c = *fmt++;
    flags = width = 0;
    prec = 0;

    while (1)
    {
      if (c == '0')
        flags |= FL_ZERO_PAD;
      else if (c == '-')
        flags |= FL_LEFT_JUST;
      else if (c == ' ')
        flags |= FL_BLANK_SIGN;
      else if (c == '+')
        flags |= FL_ALWAYS_SIGN;
      else if (c == '*')
      {
        width = va_arg(ap, int);
        if (width < 0)
        {
          width = -width;
          flags |= FL_LEFT_JUST;
        }
      }
      else
        break;

      c = *fmt++;
    }

    for (; '0' <= c && c <= '9'; c = *fmt++)
      width = width * 10 + (c - '0');

    if (c == '.')
    {
      c = *fmt++;

      if (c == '*')
      {
        prec = va_arg(ap, int);
        c = *fmt++;
      }
      else if ('0' <= c && c <= '9')
      {
        for (; '0' <= c && c <= '9'; c = *fmt++)
          prec = prec * 10 + (c - '0');
      }
      else
        prec = 6;
    }

    while (1)
    {
      if (c == 'h')
        flags |= (flags & FL_SHORT) ? FL_CHAR : FL_SHORT;
      else if (c == 'l')
        flags |= (flags & FL_LONG) ? FL_LONG_LONG : FL_LONG;
      else
        break;

      c = *fmt++;
    }

    if (c == 'd' || c == 'i')
      flags |= FL_INT;
    else if (c == 'o')
      flags |= FL_INT | FL_UNSIGNED | FL_OCT;
    else if (c == 'u')
      flags |= FL_INT | FL_UNSIGNED;
    else if (c == 'x')
      flags |= FL_INT | FL_UNSIGNED | FL_HEX;
    else if (c == 'X')
      flags |= FL_INT | FL_UNSIGNED | FL_HEX | FL_UPPERCASE;
    else if (c == 'f')
      flags |= FL_DOUBLE;
    else if (c == 'F')
      flags |= FL_DOUBLE | FL_UPPERCASE;
    else if (c == 'e')
      flags |= FL_DOUBLE | FL_EXP;
    else if (c == 'E')
      flags |= FL_DOUBLE | FL_EXP | FL_UPPERCASE;
    else if (c == 'c')
      flags |= FL_CHAR;
    else if (c == 's')
      flags |= FL_STRING;
    else
      PUT_CHR(c);

    if (flags & FL_INT)
    {
      long val;

      if (flags & FL_LONG)
        val = va_arg(ap, long);
      else if (flags & FL_LONG_LONG)
        val = va_arg(ap, long);
      else
        val = va_arg(ap, int);

      if (flags & FL_UNSIGNED)
      {
        if (flags & FL_LONG)
          val = (unsigned long)val;
        else if (flags & FL_LONG_LONG)
          val = (unsigned long)val;
        else if (flags & FL_CHAR)
          val = (unsigned char)val;
        else if (flags & FL_SHORT)
          val = (unsigned short)val;
        else
          val = (unsigned int)val;
      }

      nr = put_int(buf, end, flags, width, prec, val);
      buf += nr;
      nch += nr;
    }
    else if (flags & FL_CHAR)
    {
      char val[2];

      val[0] = (char)va_arg(ap, int);
      val[1] = 0;

      nr = put_string(buf, end, flags, width, val);
      buf += nr;
      nch += nr;
    }
    else if (flags & FL_DOUBLE)
    {
#ifdef ENABLE_FLOAT
      double val;

      val = va_arg(ap, double);

      nr = put_double(buf, end, flags, width, prec, val);
      buf += nr;
      nch += nr;
#endif
    }
    else if (flags & FL_STRING)
    {
      char *val = va_arg(ap, char *);

      nr = put_string(buf, end, flags, width, val);
      buf += nr;
      nch += nr;
    }
  }

  if (buf > end)
    end[-1] = 0;
  else
    buf[0] = 0;

  return nch;
}

//-----------------------------------------------------------------------------
int p_snprintf(char *str, size_t size, const char *fmt, ...)
{
  va_list arg;
  int res;

  va_start(arg, fmt);
  res = p_vsnprintf(str, size, fmt, arg);
  va_end(arg);

  return res;
}

