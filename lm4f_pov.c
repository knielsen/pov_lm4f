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

/*
  Current pinouts:

  PB5  GSCLK (20MHz)
  PA2  SCLK
  PA4  SOUT
  PA5  SIN

  PA6  XLAT
  PA7  MODE
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
  for (i = 0; i < 96*6/6; ++i)
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
  for (i = 0; i < 192*6/8; ++i)
  {
    uint32_t j;

    ROM_SSIDataPut(SSI0_BASE, 0);
    while (ROM_SSIBusy(SSI0_BASE))
      ;
    ROM_SSIDataGet(SSI0_BASE, &data);
    serial_output_hexbyte(data);
    if ((i % (192/8)) == 2 || (i % (192/8)) == 14)
      ROM_UARTCharPut(UART0_BASE, ' ');
    else if ((i % (192/8)) == 23)
      ROM_UARTCharPut(UART0_BASE, '\n');

    if ((i % (192/8)) < 3 || (i % (192/8)) >= 15)
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


int main()
{
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

  for (;;) {
    // set the red LED pin high, others low
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_RED|LED_GREEN);
    ROM_SysCtlDelay(5000);
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, 0);
    ROM_SysCtlDelay(5000000+5000000-5000);

    //ROM_UARTCharPut(UART0_BASE, '!');
    init_tlc_dc(DC_VALUE);
  }
}
