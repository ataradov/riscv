/*
 * Copyright (c) 2017-2018, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include "soc.h"

/*- Definitions -------------------------------------------------------------*/

/*- Constants ---------------------------------------------------------------*/
static const uint32_t bit_mask[32] =
{
  0x00000001, 0x00000002, 0x00000004, 0x00000008, 0x00000010, 0x00000020, 0x00000040, 0x00000080,
  0x00000100, 0x00000200, 0x00000400, 0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000,
  0x00010000, 0x00020000, 0x00040000, 0x00080000, 0x00100000, 0x00200000, 0x00400000, 0x00800000,
  0x01000000, 0x02000000, 0x04000000, 0x08000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000,
};

#define SORT_DATA_SIZE 16
static const int32_t sort_data[SORT_DATA_SIZE] =
{
  0x00fe8bef, 0x70e4a110, 0x00cd66cb, 0x000000f9, 0x00789b60, 0x00005f1b, 0x67ce08f0, 0x00000000,
  0x00644231, 0x00003df3, 0x0000008d, 0x0f38549a, 0x0000f7fe, 0x38cf211f, 0x0000b9a6, 0x0000001e,
};

#define TP_SIZE 8
static const uint32_t test_pattern[TP_SIZE] =
{
  0x00000000, 0x00000001, 0x000ab000, 0x7fffffff,
  0x80000000, 0x80000001, 0x800ab000, 0xffffffff,
};

#define SH_TP_SIZE 8
static const int sh_test_pattern[SH_TP_SIZE] = { 0, 1, 10, 15, 16, 20, 30, 31 };

/*- Types -------------------------------------------------------------------*/
typedef void (*callback_t)(int, int);

/*- Variables ---------------------------------------------------------------*/
int array_for_sort[SORT_DATA_SIZE];

/*- Prototypes --------------------------------------------------------------*/
int p_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap);
int p_snprintf(char *str, size_t size, const char *fmt, ...);

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void uart_init(int br)
{
  UART->BR = F_CPU / br;
}

//-----------------------------------------------------------------------------
void iputc(int c)
{
  while (0 == (UART->CSR & UART_CSR_TX_READY));
  UART->DATA = c;
}

//-----------------------------------------------------------------------------
int igetc(void)
{
  if (UART->CSR & UART_CSR_RX_READY)
    return UART->DATA;

  return -1;
}

//-----------------------------------------------------------------------------
void iputs(char *s)
{
  while (*s)
    iputc(*s++);
}

//-----------------------------------------------------------------------------
void iprintf(const char *fmt, ...)
{
  char str[200];
  va_list ap;

  va_start(ap, fmt);
  p_vsnprintf(str, sizeof(str), fmt, ap);
  va_end(ap);

  iputs(str);
}

//-----------------------------------------------------------------------------
void halt(void)
{
  iputs("*** halting ***\r\n");
  *(volatile uint32_t *)0x80000000 = 1; // Simulation only
}

//-----------------------------------------------------------------------------
void assert(bool cond, const char *fmt, ...)
{
  if (!cond)
  {
    char str[200];
    va_list ap;

    iputs("Assertion failed: ");

    va_start(ap, fmt);
    p_vsnprintf(str, sizeof(str), fmt, ap);
    va_end(ap);

    iputs(str);
    iputs("\r\n");
    halt();
  }
}

//-----------------------------------------------------------------------------
void puthex(uint32_t v, int size)
{
  static const char hex[] = "0123456789abcdef";

  for (int i = 0; i < size; i++)
  {
    int offs = ((size - 1) - i) * 4;
    iputc(hex[(v >> offs) & 0xf]);
  }
}

//-----------------------------------------------------------------------------
uint32_t divide(uint32_t dividend, uint32_t divisor, uint32_t *remainder)
{
  uint32_t quot = dividend;
  uint32_t rem = 0;
  uint32_t tmp;

  for (int i = 0; i < 32; i++)
  {
    tmp = quot;
    quot = quot + quot;
    rem = rem + rem + (quot < tmp);

    if (rem >= divisor)
    {
      rem = rem - divisor;
      quot = quot + 1;
    }
  }

  *remainder = rem;

  return quot;
}

//-----------------------------------------------------------------------------
int32_t divide_signed(int32_t dividend, int32_t divisor, int32_t *remainder)
{
  bool negative = ((dividend & 0x80000000) != (divisor & 0x80000000));
  uint32_t quot, rem, tmp;

  if (dividend & 0x80000000)
    dividend = -dividend;

  if (divisor & 0x80000000)
    divisor = -divisor;

  quot = dividend;
  rem = 0;

  for (int i = 0; i < 32; i++)
  {
    tmp = quot;
    quot = quot + quot;
    rem = rem + rem + (quot < tmp);

    if (rem >= (uint32_t)divisor)
    {
      rem = rem - divisor;
      quot = quot + 1;
    }
  }

  *remainder = negative ? -rem : rem;

  return negative ? -quot : quot;
}

//-----------------------------------------------------------------------------
uint64_t multiply(uint32_t a, uint32_t b)
{
  uint64_t res = 0;

  for (int i = 0; i < 32; i++)
    res += (a & (1 << i))  ? ((uint64_t)b << i) : 0;

  return res;
}

//-----------------------------------------------------------------------------
int64_t multiply_signed(int32_t a, int32_t b)
{
  bool negative = ((a & 0x80000000) != (b & 0x80000000));
  int64_t aa = a;
  int64_t bb = b;
  uint64_t res = 0;

  if (0 != (aa & 0x80000000))
    aa = -aa;

  if (0 != (b & 0x80000000))
    bb = -bb;

  for (int i = 0; i < 32; i++)
    res += (aa & (1 << i))  ? (bb << i) : 0;

  return negative ? -res : res;
}

//-----------------------------------------------------------------------------
uint32_t shift(uint32_t a, int b, bool dir, bool arith)
{
  uint32_t res = 0;

  for (int i = 0; i < 32; i++)
  {
    int idx = dir ? (i + b) : (i - b);

    if ((a & bit_mask[i]) && (0 <= idx && idx < 32))
      res |= bit_mask[idx];
  }

  if (arith && (a & bit_mask[31]))
  {
    for (int i = 0; i < b; i++)
      res |= bit_mask[31 - i];
  }

  return res;
}

//-----------------------------------------------------------------------------
void bubble(int *a, int size)
{
  int swap;

  do
  {
    swap = 0;

    for (int i = 1; i < size; i++)
    {
      if (a[i-1] > a[i])
      {
        int t = a[i-1];
        a[i-1] = a[i];
        a[i] = t;
        swap = 1;
      }
    }
  } while (swap);
}

//-----------------------------------------------------------------------------
void qsort_my(int *v, int left, int right)
{
  int i, j, m, mi;

  i = left;
  j = right;
  mi = (left + right) / 2;
  m = v[mi];

  while (i <= j)
  {
    while (v[i] < m)
      i++;

    while (m < v[j])
      j--;

    if (i <= j)
    {
      //swap(v, i, j);
      int t = v[i];
      v[i] = v[j];
      v[j] = t;

      i++;
      j--;
    }
  }

  if (left < j)
    qsort_my(v, left, j);

  if (i < right)
    qsort_my(v, i, right);
}

//-----------------------------------------------------------------------------
static void test_basic_prints(void)
{
  iputs("\r\nBasic prints:\r\n");

  puthex(0x1, 1);
  iputs("\r\n");
  puthex(0x12, 2);
  iputs("\r\n");
  puthex(0x123, 3);
  iputs("\r\n");
  puthex(0x1234, 4);
  iputs("\r\n");
  puthex(0x12345, 5);
  iputs("\r\n");
  puthex(0x123456, 6);
  iputs("\r\n");
  puthex(0x1234567, 7);
  iputs("\r\n");
  puthex(0x12345678, 8);
  iputs("\r\n");
}

//-----------------------------------------------------------------------------
static void test_printf(void)
{
  iputs("\r\nprintf():\r\n");

  iprintf("  printf() test\r\n");
  iprintf("  printf() test %d some other text\r\n", 123456);
  iprintf("  Test %s %c %d, 0x%08x\r\n", "str", '$', 1234, 0x1234);
}

//-----------------------------------------------------------------------------
static void test_callback_fn(int index, int value)
{
  iprintf("  test_callback_fn: %d %d\r\n", index, value);
  assert(sort_data[index] == value, "Callback value is incorrect");
}

//-----------------------------------------------------------------------------
static void test_callbacks(void)
{
  volatile callback_t callback = test_callback_fn;

  iputs("\r\nCallbacks:\r\n");

  iprintf("  before callback()\r\n");

  callback(0, sort_data[0]);
  callback(1, sort_data[1]);
  callback(2, sort_data[2]);

  for (int i = 0; i < SORT_DATA_SIZE; i++)
    callback(i, sort_data[i]);

  iprintf("  after callback()\r\n");
}

//-----------------------------------------------------------------------------
static void test_bubble_sort(void)
{
  iputs("\r\nBubble sort:\r\n");

  for (int i = 0; i < SORT_DATA_SIZE; i++)
    array_for_sort[i] = sort_data[i];

  iputs("  array init done\r\n");

  bubble(array_for_sort, SORT_DATA_SIZE);

  iputs("  sort done:\r\n");

  for (int i = 0; i < SORT_DATA_SIZE; i++)
    iprintf("  %2d - 0x%08x\r\n", i, array_for_sort[i]);

  for (int i = 1; i < SORT_DATA_SIZE; i++)
    assert(array_for_sort[i-1] <= array_for_sort[i], "Bubble sort error\r\n");
}

//-----------------------------------------------------------------------------
static void test_qsort(void)
{
  iputs("\r\nQsort sort:\r\n");

  for (int i = 0; i < SORT_DATA_SIZE; i++)
    array_for_sort[i] = sort_data[i];

  iputs("  array init done\r\n");

  qsort_my(array_for_sort, 0, SORT_DATA_SIZE-1);

  iputs("  sort done:\r\n");

  for (int i = 0; i < SORT_DATA_SIZE; i++)
    iprintf("  %2d - 0x%08x\r\n", i, array_for_sort[i]);

  for (int i = 1; i < SORT_DATA_SIZE; i++)
    assert(array_for_sort[i-1] <= array_for_sort[i], "Qsort error\r\n");
}

//-----------------------------------------------------------------------------
void test_hw_sw_div(void)
{
  iputs("\r\nHW/SW divider:\r\n");

  for (int i = 0; i < TP_SIZE; i++)
  {
    iprintf("  i = %d\r\n", i);

    for (int j = 0; j < TP_SIZE; j++)
    {
      uint32_t num = test_pattern[i];
      uint32_t den = test_pattern[j];

      if (0 == den)
        den = 1;

      { // Unsigned
        uint32_t qref, rref, qres, rres;

        qref = divide(num, den, &rref);
        qres = num / den;
        rres = num % den;

        assert(qref == qres, "Unsigned divide error (Q): %08x / %08x: sw = %08x, hw = %08x\r\n", num, den, qref, qres);
        assert(rref == rres, "Unsigned divide error (R): %08x %% %08x: sw = %08x, hw = %08x\r\n", num, den, rref, rres);
      }

      { // Signed
        int32_t qref, rref, qres, rres;

        qref = divide_signed(num, den, &rref);
        qres = (int32_t)num / (int32_t)den;
        rres = (int32_t)num % (int32_t)den;

        assert(qref == qres, "Signed divide error (Q): %08x / %08x: sw = %08x, hw = %08x\r\n", num, den, qref, qres);
        assert(rref == rres, "Signed divide error (R): %08x %% %08x: sw = %08x, hw = %08x\r\n", num, den, rref, rres);
      }
    }
  }
}

//-----------------------------------------------------------------------------
void test_hw_sw_mul(void)
{
  iputs("\r\nHW/SW multiplier:\r\n");

  for (int i = 0; i < TP_SIZE; i++)
  {
    iprintf("  i = %d\r\n", i);

    for (int j = 0; j < TP_SIZE; j++)
    {
      uint32_t a = test_pattern[i];
      uint32_t b = test_pattern[j];

      { // Unsigned
        uint64_t ref, res;

        ref = multiply(a, b);
        res = (uint64_t)a * (uint64_t)b;

        assert(ref == res, "Unsigned multiply error: %08x * %08x: sw = %08x%08x, hw = %08x%08x\r\n",
              a, b, (uint32_t)(ref >> 32), (uint32_t)(ref & 0xffffffff), (uint32_t)(res >> 32), (uint32_t)(res & 0xffffffff));
      }
      { // Signed
        int32_t aa = a;
        int32_t bb = b;
        int64_t ref, res;

        ref = multiply_signed(aa, bb);
        res = (int64_t)aa * (int64_t)bb;

        assert(ref == res, "Signed multiply error: %08x * %08x: sw = %08x%08x, hw = %08x%08x\r\n",
              aa, bb, (uint32_t)(ref >> 32), (uint32_t)(ref & 0xffffffff), (uint32_t)(res >> 32), (uint32_t)(res & 0xffffffff));
      }
    }
  }
}

//-----------------------------------------------------------------------------
void test_hw_sw_shift(void)
{
  // Note: this is a sligtly redundant test, since without working shifts
  // we are not likely to get here anyway.
  iputs("\r\nHW/SW shifter:\r\n");

  for (int i = 0; i < TP_SIZE; i++)
  {
    iprintf("  i = %d\r\n", i);

    for (int j = 0; j < SH_TP_SIZE; j++)
    {
      int amt = sh_test_pattern[j];

      iprintf("    sh = %d\r\n", amt);

      { // Left
        uint32_t a = test_pattern[i];
        uint32_t ref, res;

        ref = shift(a, amt, true, false);
        res = a << amt;

        assert(ref == res, "Shift error: %08x << %d: sw = %08x, hw = %08x\r\n", a, amt, ref, res);
      }

      { // Logical Right
        uint32_t a = test_pattern[i];
        uint32_t ref, res;

        ref = shift(a, amt, false, false);
        res = a >> amt;

        assert(ref == res, "Shift error: %08x >> %d: sw = %08x, hw = %08x\r\n", a, amt, ref, res);
      }

      { // Arithmetic Right
        int32_t a = test_pattern[i];
        int32_t ref, res;

        ref = shift(a, amt, false, true);
        res = a >> amt;

        assert(ref == res, "Shift error: %08x >>> %d: sw = %08x, hw = %08x\r\n", a, amt, ref, res);
      }
    }
  }
}

//-----------------------------------------------------------------------------
void test_auipc(void)
{
  uint32_t pc, res;

  iputs("\r\nAUIPC:\r\n");

  iprintf("  imm = 0x0\r\n");

  asm(
    "jal %0, 1f \n\t"
    "1: \n\t"
    "auipc %1, 0x0 \n\t"
    : "=r"(pc), "=r"(res) : : );
  assert(res == pc, "AUIPC error 0: pc = 0x%08x, res = 0x%08x\r\n", pc, res);

  iprintf("  imm = 0x12345\r\n");

  asm(
    "jal %0, 1f \n\t"
    "1: \n\t"
    "auipc %1, 0x12345 \n\t"
    : "=r"(pc), "=r"(res) : : );
  assert(res == pc + 0x12345000, "AUIPC error 1: pc = 0x%08x, res = 0x%08x\r\n", pc, res);

  iprintf("  imm = 0xfffff\r\n");

  asm(
    "jal %0, 1f \n\t"
    "1: \n\t"
    "auipc %1, 0xfffff \n\t"
    : "=r"(pc), "=r"(res) : : );
  assert(res == pc + 0xfffff000, "AUIPC error 2: pc = 0x%08x, res = 0x%08x\r\n", pc, res);
}

//-----------------------------------------------------------------------------
void timer_init(int interval_us)
{
  TIMER->COUNT = 0;
  TIMER->COMPARE = interval_us * (F_CPU / 1000000);
  TIMER->CSR = TIMER_CSR_ENABLE | TIMER_CSR_OVERFLOW;
}

//-----------------------------------------------------------------------------
bool timer_expired(void)
{
  if (TIMER->CSR & TIMER_CSR_OVERFLOW)
  {
    TIMER->CSR = TIMER_CSR_OVERFLOW;
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
void test_hardware(void)
{
  int debounce = 0;
  bool dir = true;
  int cnt = 0;

  GPIO->WRITE = 0xaa;

  timer_init(1000000);

  while (1)
  {
    if (timer_expired())
    {
      iputc('.');
      GPIO->WRITE = cnt;
      cnt += dir ? 1 : -1;
    }

    if (GPIO->READ & GPIO_BIT_0)
      debounce = 0;
    else if (debounce < 20001)
      debounce++;

    if (20000 == debounce)
    {
      dir = !dir;
      iputs(dir ? "up" : "down");
    }

    int rx = igetc();

    if (-1 != rx)
    {
      if ('a' <= rx && rx <= 'z')
        rx -= 32;
      else if ('A' <= rx && rx <= 'Z')
        rx += 32;

      iputc(rx);
    }
  }
}

//-----------------------------------------------------------------------------
int main(void)
{
  uart_init(115200);
  iputs("\r\n--- main ---\r\n");

  test_basic_prints();
  test_printf();
  test_callbacks();
  test_bubble_sort();
  test_qsort();
  test_hw_sw_div();
  test_hw_sw_mul();
  test_hw_sw_shift();
  test_auipc();

  iputs("\r\nAll tests PASSED.\r\n");

  halt();

  test_hardware();

  return 0;
}

//-----------------------------------------------------------------------------
void entry(void)
{
  extern unsigned int _bss;
  extern unsigned int _ebss;

  for (unsigned int *dst = &_bss; dst < &_ebss; dst++)
    *dst = 0;

  main();
}

