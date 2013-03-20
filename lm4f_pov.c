#include <inttypes.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"

#define DC_VALUE 2
#define NUM_TLC 6
#define LEDS_PER_TLC 16
#define TLC_GS_BYTES (12 * LEDS_PER_TLC * NUM_TLC / 8)
#define NUM_RGB_LEDS (NUM_TLC*LEDS_PER_TLC/3)


/*
  Current pinouts:

  PB5  GSCLK (20MHz)
  PA2  SCLK
  PA4  SOUT
  PA5  SIN

  PA6  XLAT
  PA7  MODE
  PB4  BLANK
*/

#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3


static void
serial_output_hexdig(uint32_t dig)
{
  ROM_UARTCharPut(UART0_BASE, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


static void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


/* Configure Timer1B to output a PWM signal for GSCLK. */
static void
setup_pwm_GSCLK(void)
{
  /* Enable the timer. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  /* Enable the GPIO module for the timer pin. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  /* Configure PB5 to be I/O pin for timer 1 B. */
  ROM_GPIOPinConfigure(GPIO_PB5_T1CCP1);
  /*
    Configure PB5: direction controlled by hardware, 2mA drive strength,
    normal drive (no pullup).
  */
  ROM_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_5);
  /* Configure 2 * 16-bit timer in PWM mode. */
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_B_PWM);
  /*
    Set timer1B start and compare values.
    High at 3, low at 1 -> 80MHz/4 = 20MHz clock.
  */
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_B, 3);
  ROM_TimerMatchSet(TIMER1_BASE, TIMER_B, 1);
  /* Start the timer. */
  ROM_TimerEnable(TIMER1_BASE, TIMER_B);
}


static void
config_tlc_gpio(void)
{
  /* Setup GPIO pins for SSI0. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
  ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
  ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
  ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                     GPIO_PIN_2);

  /* Setup PA6 and PA7 for XLAT and MODE. */
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
  /* Set XLAT low. */
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
  /* Set MODE low, to select GS mode. */
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);

  /* Setup PB4 for BLANK, pull it high initially. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
  ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
}


static void
config_spi_tlc_read(uint32_t bits_per_word)
{
  /*
    Configure the SPI for correct mode to read from TLC5940.

    We need CLK inactive low, so SPO=0.
    We need to sample on the trailing, falling CLK edge, so SPH=1.

    (We could try to use SPH=0 also for reading. Then we will sample on rising
    edge, while TLC does setup on the same edge. The datasheet says setup time
    is MAX 30 ns, there is no min. So depending on the actual setup time,
    there may be time to read the old value before the new one is setup, or
    there may not.)
  */

  ROM_SSIDisable(SSI0_BASE);
  ROM_SSIConfigSetExpClk(SSI0_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_1,
                     SSI_MODE_MASTER, 20000000, bits_per_word);
  ROM_SSIEnable(SSI0_BASE);
}

static void
config_spi_tlc_write(uint32_t bits_per_word)
{
  /*
    Configure the SPI for correct mode to write to TLC5940, and send
    DC data.

    We need CLK inactive low, so SPO=0.
    We need to setup on the trailing, falling CLK edge, so SPH=0.
  */

  ROM_SSIDisable(SSI0_BASE);
  ROM_SSIConfigSetExpClk(SSI0_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 20000000, bits_per_word);
  ROM_SSIEnable(SSI0_BASE);
}

/*
  Set the DC register on TLC5940 through SSI0.
  Then read out the TLC5940 status register and check that DC is correct.
*/
static void
init_tlc_dc(uint8_t dc_value)
{
  uint32_t data;
  uint32_t i;
  uint32_t value;
  uint32_t bits_collected;

  config_tlc_gpio();

  /* Prepare to write 6-bit DC words to TLC. */
  config_spi_tlc_write(6);

  /* Set MODE high, to select DC mode. */
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
  ROM_SysCtlDelay(5);

  /* Write DC value (6 bits) to all 16 outputs and all 6 TLCs. */
  for (i = 0; i < LEDS_PER_TLC * NUM_TLC; ++i)
  {
    ROM_SSIDataPut(SSI0_BASE, dc_value);
    while (ROM_SSIBusy(SSI0_BASE))
      ;
    /* Drain the receive FIFO just so it does not get full. */
    ROM_SSIDataGet(SSI0_BASE, &data);
  }

  /* Pulse XLAT so we get the DC values stored in the DC registers. */
  ROM_SysCtlDelay(5);
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
  ROM_SysCtlDelay(5);
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
  ROM_SysCtlDelay(5);

  config_spi_tlc_read(8);

  /* Set MODE low, to select GS mode. */
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
  ROM_SysCtlDelay(5);

  /* Empty the receive FIFO. */
  while(ROM_SSIDataGetNonBlocking(SSI0_BASE, &data))
    ;

  /* Pulse XLAT so we get the status register loaded into the shift register. */
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
  ROM_SysCtlDelay(5);
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
  ROM_SysCtlDelay(5);

  bits_collected = 0;
  value = 0;
  for (i = 0; i < 12*LEDS_PER_TLC*NUM_TLC/8; ++i)
  {
    uint32_t j;

    ROM_SSIDataPut(SSI0_BASE, 0);
    while (ROM_SSIBusy(SSI0_BASE))
      ;
    ROM_SSIDataGet(SSI0_BASE, &data);
    serial_output_hexbyte(data);
    if ((i % (12*LEDS_PER_TLC/8)) == 2 || (i % (12*LEDS_PER_TLC/8)) == 14)
      ROM_UARTCharPut(UART0_BASE, ' ');
    else if ((i % (12*LEDS_PER_TLC/8)) == 23)
      ROM_UARTCharPut(UART0_BASE, '\n');

    if ((i % (12*LEDS_PER_TLC/8)) < 3 || (i % (12*LEDS_PER_TLC/8)) >= 15)
      continue;
    /* This is one of the DC values, pick out the 6-bit values one-by-one. */
    for (j = 0; j < 8; ++j)
    {
      value = (value << 1) | ((data >> (7-j)) & 1);
      ++bits_collected;
      if (bits_collected == 6)
      {
        /* Check if the received DC value is correct, halt if not. */
        if (value != DC_VALUE)
        {
          ROM_UARTCharPut(UART0_BASE, '#');
          serial_output_hexbyte(value);
          ROM_UARTCharPut(UART0_BASE, '#');
          ROM_UARTCharPut(UART0_BASE, '\n');
          /* Flash the red LED endlessly to complain. */
          for (;;)
          {
            ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_RED);
            ROM_SysCtlDelay(40000000/3);
            ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, 0);
            ROM_SysCtlDelay(40000000/3);
          }
        }
        value = 0;
        bits_collected = 0;
      }
    }
  }
  ROM_UARTCharPut(UART0_BASE, '!');
  ROM_UARTCharPut(UART0_BASE, '\n');

  /* Leave the SPI in GS write mode. */
  config_spi_tlc_write(8);
}


/*
  Shift out an array of 12-bit grayscale data to the TLCs.

  Assumes that GPIO and SSI has already been configured and that
  VPRG (MODE) is already low to select GS mode.
*/
static void
shift_out_gs_to_tlc(uint8_t *gs_data)
{
  uint32_t data;
  uint32_t i;

  /* Write 12 bits of GS data to all 16 outputs on all 6 TLCs. */
  for (i = 0; i < TLC_GS_BYTES; ++i)
  {
    ROM_SSIDataPut(SSI0_BASE, gs_data[i]);
    while (ROM_SSIBusy(SSI0_BASE))
      ;
    /* Drain the receive FIFO just so it does not get full. */
    ROM_SSIDataGet(SSI0_BASE, &data);
  }
}

static void
display_led_data(uint8_t *gs_data)
{
  shift_out_gs_to_tlc(gs_data);

  /* Pulse XLAT while holding BLANK high to latch new GS data. */
  ROM_SysCtlDelay(1);
  ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4);
  ROM_SysCtlDelay(1);
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
  ROM_SysCtlDelay(1);
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
  ROM_SysCtlDelay(1);
  ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0);
  ROM_SysCtlDelay(1);
}


static void
set_led(uint8_t *buf, uint32_t idx, uint32_t red, uint32_t green, uint32_t blue)
{
  uint8_t *entry;

  /*
    On the first TLC, OUT0 is green, OUT1 is red, OUT2 is blue.

    Since data is shifted out last-to-first, the first entry in the buffer is
    the last blue LED, and the last entry is the first green LED.
  */

  idx = ((NUM_RGB_LEDS-1) - idx);
  entry = buf + (idx*9)/2;
  if (idx & 1)
  {
    entry[4] = green & 0xff;
    entry[3] = (green >> 8) | ((red & 0xf) << 4);
    entry[2] = red >> 4;
    entry[1] = blue & 0xff;
    entry[0] = (entry[0] & 0xf0) | (blue >> 8);
  }
  else
  {
    entry[4] = (entry[4] & 0xf) | ((green & 0xf) << 4);
    entry[3] = green >> 4;
    entry[2] = red & 0xff;
    entry[1] = (red >> 8) | ((blue & 0xf) << 4);
    entry[0] = blue >> 4;
  }
}


static void
gs_clear(uint8_t *buf)
{
  //memset(buf, 0, TLC_GS_BYTES);
  for (uint32_t i = 0; i < TLC_GS_BYTES; ++i)
    buf[i] = 0;
}


 __attribute__ ((unused))
static void
anim1(uint8_t *buf, uint32_t count)
{
  gs_clear(buf);
  switch ((count/50/NUM_RGB_LEDS)%3)
  {
  case 0:
    set_led(buf, ((count/50) % NUM_RGB_LEDS), 4095, 0, 0);
    break;
  case 1:
    set_led(buf, ((count/50) % NUM_RGB_LEDS), 0, 4095, 0);
    break;
  case 2:
    set_led(buf, ((count/50) % NUM_RGB_LEDS), 0, 0, 4095);
    break;
  }
}

 __attribute__ ((unused))
static void
anim2(uint8_t *buf, uint32_t count)
{
  uint32_t i;
  gs_clear(buf);

  for (i = 0; i < 32; ++i)
  {
    uint32_t v = (i + (count/15)) % 48;
    if (v < 16)
      set_led(buf, i, 4095, 4095*(15-v)/15, 0);
    else if (v < 24)
      set_led(buf, i, 4095, 0, 4095*(v-16)/7);
    else if (v < 32)
      set_led(buf, i, 4095*(7-(v-24))/7, 0, 4095);
    else if (v < 40)
      set_led(buf, i, 0, 4095*(v-32)/7, 4095*(7-(v-32))/7);
    else
      set_led(buf, i, 4095*(v-40)/7, 4095, 0);
  }
}


int main()
{
  uint32_t count;

  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  setup_pwm_GSCLK();

  /* Configure LED GPIOs. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_BLUE|LED_GREEN);

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

  init_tlc_dc(DC_VALUE);

  count = 0;
  for (;;) {
    uint8_t buf[TLC_GS_BYTES];

    /* Try display a bit of animation. */
    //anim1(buf, count);
    anim2(buf, count);

    ++count;
    display_led_data(buf);

    /* Flash the LED a bit. */
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_RED|LED_GREEN);
    ROM_SysCtlDelay(50);
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, 0);
    ROM_SysCtlDelay(50000+50000-500);

    //ROM_UARTCharPut(UART0_BASE, '!');
  }
}
