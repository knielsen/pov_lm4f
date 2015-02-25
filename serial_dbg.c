#include <inttypes.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"

#include "serial_dbg.h"

static void
serial_output_hexdig(uint32_t dig)
{
  ROM_UARTCharPut(UART0_BASE, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


void
serial_output_str(const char *str)
{
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
}


char *
uint32_tostring(char *buf, uint32_t val)
{
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p = '\0';
  return p;
}


void
println_uint32(uint32_t val)
{
  char buf[13];
  char *p = uint32_tostring(buf, val);
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


void
float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after)
{
  float a;
  uint32_t d;
  uint8_t leading_zero;

  if (f == 0.0f)
  {
    buf[0] = '0';
    buf[1] = '\0';
    return;
  }
  if (f < 0)
  {
    *buf++ = '-';
    f = -f;
  }
  a =  powf(10.0f, (float)dig_before);
  if (f >= a)
  {
    buf[0] = '#';
    buf[1] = '\0';
    return;
  }
  leading_zero = 1;
  while (dig_before)
  {
    a /= 10.0f;
    d = (uint32_t)(f / a);
    if (leading_zero && d == 0 && a >= 10.0f)
      *buf++ = ' ';
    else
    {
      leading_zero = 0;
      *buf++ = '0' + d;
      f -= d*a;
    }
    --dig_before;
  }
  if (!dig_after)
  {
    *buf++ = '\0';
    return;
  }
  *buf++ = '.';
  do
  {
    f *= 10.0f;
    d = (uint32_t)f;
    *buf++ = '0' + d;
    f -= (float)d;
    --dig_after;
  } while (dig_after);
  *buf++ = '\0';
}


void
println_float(float f, uint32_t dig_before, uint32_t dig_after)
{
  char buf[21];
  char *p = buf;

  float_to_str(p, f, dig_before, dig_after);
  while (*p)
    ++p;
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


void
serial_hexdump(uint8_t *buf, uint32_t len)
{
  uint32_t i, j;

  for (i = 0; i < len; i += 16)
  {
    for (j = 0; j < 16; ++j)
    {
      if (i + j < len)
        serial_output_hexbyte(buf[i+j]);
      else
        serial_output_str("  ");
      if (j % 2)
        serial_output_str(" ");
    }
    serial_output_str(" ");
    for (j = 0; j < 16; ++j)
    {
      char c;
      if (i + j >= len)
      {
        ROM_UARTCharPut(UART0_BASE, ' ');
        continue;
      }
      c = buf[i+j];
      if (c >= 32 && c < 127)
        ROM_UARTCharPut(UART0_BASE, c);
      else
        ROM_UARTCharPut(UART0_BASE, '.');
    }
    serial_output_str("\r\n");
  }
}
