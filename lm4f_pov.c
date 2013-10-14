#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"

#include "gfx.h"


#define DC_VALUE 1
#define NUM_TLC 6
#define LEDS_PER_TLC 16
#define TLC_GS_BYTES (12 * LEDS_PER_TLC * NUM_TLC / 8)
#define NUM_RGB_LEDS (NUM_TLC*LEDS_PER_TLC/3)

/*
  Useful for testing.
  When defined, the actual hall sensor is ignored, instead it fakes that
  the hall triggers with some reasonable frequency.
*/
#define FAKE_HALL_SENSOR 1
/* Define to use only a single blade, for testing. */
//#define SINGLE_BLADE 1

/*
  Current pinouts:

                  PCB#1  PCB#2  PCB#3
   SIN             PA5    PB7    PD3
   GSCLK (20MHz)   PB1    PB2    PB3
   MODE            PA7    PE0    PE3
   SCLK            PA2    PB4    PD0
   XLAT            PA6    PD6    PE2
   BLANK           PE1    PE4    PE5
   SOUT            PA4    PB6    PD2

   HALL            PC4   (PC5)  (PC6)

  Nordic wireless: MISO  PF0 (white)
                   MOSI  PF1 (green)
                   CLK   PF2 (brown, middle)
                   CS    PF3 (brown, right)

*/

#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3


#define GPIO_MODE1_PERIPH SYSCTL_PERIPH_GPIOA
#define GPIO_MODE1_BASE GPIO_PORTA_BASE
#define GPIO_MODE1_PIN GPIO_PIN_7
#define GPIO_MODE2_PERIPH SYSCTL_PERIPH_GPIOE
#define GPIO_MODE2_BASE GPIO_PORTE_BASE
#define GPIO_MODE2_PIN GPIO_PIN_0
#define GPIO_MODE3_PERIPH SYSCTL_PERIPH_GPIOE
#define GPIO_MODE3_BASE GPIO_PORTE_BASE
#define GPIO_MODE3_PIN GPIO_PIN_3

#define GPIO_XLAT1_PERIPH SYSCTL_PERIPH_GPIOA
#define GPIO_XLAT1_BASE GPIO_PORTA_BASE
#define GPIO_XLAT1_PIN GPIO_PIN_6
#define GPIO_XLAT2_PERIPH SYSCTL_PERIPH_GPIOD
#define GPIO_XLAT2_BASE GPIO_PORTD_BASE
#define GPIO_XLAT2_PIN GPIO_PIN_6
#define GPIO_XLAT3_PERIPH SYSCTL_PERIPH_GPIOE
#define GPIO_XLAT3_BASE GPIO_PORTE_BASE
#define GPIO_XLAT3_PIN GPIO_PIN_2

#define GPIO_BLANK1_PERIPH SYSCTL_PERIPH_GPIOE
#define GPIO_BLANK1_BASE GPIO_PORTE_BASE
#define GPIO_BLANK1_PIN GPIO_PIN_1
#define GPIO_BLANK2_PERIPH SYSCTL_PERIPH_GPIOE
#define GPIO_BLANK2_BASE GPIO_PORTE_BASE
#define GPIO_BLANK2_PIN GPIO_PIN_4
#define GPIO_BLANK3_PERIPH SYSCTL_PERIPH_GPIOE
#define GPIO_BLANK3_BASE GPIO_PORTE_BASE
#define GPIO_BLANK3_PIN GPIO_PIN_5

#define SSI_TLC1_PERIPH SYSCTL_PERIPH_SSI0
#define SSI_TLC1_BASE SSI0_BASE
#define SSI_TLC1_IO_PERIPH SYSCTL_PERIPH_GPIOA
#define SSI_TLC1_IO_BASE GPIO_PORTA_BASE
#define SSI_TLC1_INT INT_SSI0
#define SSI_TLC1_DMA UDMA_CHANNEL_SSI0TX
#define SSI_TLC1_DMA_CHAN_ASSIGN UDMA_CH11_SSI0TX
#define SSI_TLC1_CLK_CFG GPIO_PA2_SSI0CLK
#define SSI_TLC1_CLK_PIN GPIO_PIN_2
#define SSI_TLC1_RX_CFG GPIO_PA4_SSI0RX
#define SSI_TLC1_RX_PIN GPIO_PIN_4
#define SSI_TLC1_TX_CFG GPIO_PA5_SSI0TX
#define SSI_TLC1_TX_PIN GPIO_PIN_5

#define SSI_TLC2_PERIPH SYSCTL_PERIPH_SSI2
#define SSI_TLC2_BASE SSI2_BASE
#define SSI_TLC2_IO_PERIPH SYSCTL_PERIPH_GPIOB
#define SSI_TLC2_IO_BASE GPIO_PORTB_BASE
#define SSI_TLC2_INT INT_SSI2
#define SSI_TLC2_DMA_CHAN_ASSIGN UDMA_CH13_SSI2TX
#define SSI_TLC2_DMA (SSI_TLC2_DMA_CHAN_ASSIGN & 0xff)
#define SSI_TLC2_CLK_CFG GPIO_PB4_SSI2CLK
#define SSI_TLC2_CLK_PIN GPIO_PIN_4
#define SSI_TLC2_RX_CFG GPIO_PB6_SSI2RX
#define SSI_TLC2_RX_PIN GPIO_PIN_6
#define SSI_TLC2_TX_CFG GPIO_PB7_SSI2TX
#define SSI_TLC2_TX_PIN GPIO_PIN_7

#define SSI_TLC3_PERIPH SYSCTL_PERIPH_SSI3
#define SSI_TLC3_BASE SSI3_BASE
#define SSI_TLC3_IO_PERIPH SYSCTL_PERIPH_GPIOD
#define SSI_TLC3_IO_BASE GPIO_PORTD_BASE
#define SSI_TLC3_INT INT_SSI3
#define SSI_TLC3_DMA_CHAN_ASSIGN UDMA_CH15_SSI3TX
#define SSI_TLC3_DMA (SSI_TLC3_DMA_CHAN_ASSIGN & 0xff)
#define SSI_TLC3_CLK_CFG GPIO_PD0_SSI3CLK
#define SSI_TLC3_CLK_PIN GPIO_PIN_0
#define SSI_TLC3_RX_CFG GPIO_PD2_SSI3RX
#define SSI_TLC3_RX_PIN GPIO_PIN_2
#define SSI_TLC3_TX_CFG GPIO_PD3_SSI3TX
#define SSI_TLC3_TX_PIN GPIO_PIN_3

/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000



#ifndef M_PI
#define M_PI 3.141592654f
#endif

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


static void
serial_output_str(const char *str)
{
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
}


 __attribute__ ((unused))
static void
println_uint32(uint32_t val)
{
  char buf[12];
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

  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


static void
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


 __attribute__ ((unused))
static void
println_float(float f, uint32_t dig_before, uint32_t dig_after)
{
  char buf[20];
  char *p = buf;

  float_to_str(p, f, dig_before, dig_after);
  while (*p)
    ++p;
  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


static void
setup_hall_gpio_n_timer(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  ROM_GPIOPinConfigure(GPIO_PC4_WT0CCP0);
  ROM_GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4);

  /*
    I could not get the timer capture to work in count-up mode.
    Perhaps count-up for capture is just not supported on my chip, though
    I did not see this anywhere in the datasheet...
  */

  ROM_TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME);
  ROM_TimerLoadSet(WTIMER0_BASE, TIMER_A, 0xffffffffUL);
  ROM_TimerMatchSet(WTIMER0_BASE, TIMER_A, 0);
  ROM_TimerControlEvent(WTIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

  ROM_IntMasterEnable();
  ROM_TimerIntEnable(WTIMER0_BASE, TIMER_CAPA_EVENT);
  ROM_IntEnable(INT_WTIMER0A);

  ROM_TimerEnable(WTIMER0_BASE, TIMER_A);
}


static volatile uint32_t last_hall = 0xffffffffUL;
static volatile uint32_t last_hall_period = 0;
static volatile uint32_t hall_int_counts = 0;

static volatile uint32_t hall_capture_delay = 0;

static void
record_hall_sensor(uint32_t timer_val)
{
  last_hall_period = last_hall - timer_val;
  last_hall = timer_val;
  ++hall_int_counts;
}


void
IntHandlerWTimer0A(void)
{
  /*
    Wait a little while before taking the interrupt again.
    This to work-around a very unstable signal around the low->high
    transition.
  */
  ROM_IntDisable(INT_WTIMER0A);
  ROM_TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);
  hall_capture_delay = 50;

#ifndef FAKE_HALL_SENSOR
  record_hall_sensor(ROM_TimerValueGet(WTIMER0_BASE, TIMER_A));
#endif
}


/*
  Configure Timer2B to output a PWM signal for GSCLK1.
  Timer2A triggers an interrupt at the end of each TLC5940 PWM period.
*/
static void
setup_pwm_GSCLK1_n_timer(void)
{
  /* Enable the timer. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  /* Enable the GPIO module for the timer pin. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  /* Configure PB1 to be I/O pin for timer 2 B. */
  ROM_GPIOPinConfigure(GPIO_PB1_T2CCP1);
  /*
    Configure PB1: direction controlled by hardware, 2mA drive strength,
    normal drive (no pullup).
  */
  ROM_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_1);
  /* Configure 2 * 16-bit timer, B in PWM mode, A periodic. */
  ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR |
                     TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PWM);
  /*
    Set timer2B start and compare values.
    High at 3, low at 1 -> 80MHz/4 = 20MHz clock.
  */
  ROM_TimerLoadSet(TIMER2_BASE, TIMER_B, 3);
  ROM_TimerMatchSet(TIMER2_BASE, TIMER_B, 1);

  /* Set timer interrupt at GSCLK PWM period. */
  ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, 4*4096);

  /* Enable interrrupts. */
  ROM_IntMasterEnable();
  ROM_TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
  ROM_IntEnable(INT_TIMER2A);

  /* Start the timer. */
  ROM_TimerEnable(TIMER2_BASE, TIMER_BOTH);
}


/* Configure Timer3 to output a PWM signal for GSCLK2 and 3. */
static void
setup_pwm_GSCLK23(void)
{
  /* Enable the timer. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
  /* Enable the GPIO module for the timer pin. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  /* Configure PB2 and PB3 to be I/O pins for timer 3 A and B. */
  ROM_GPIOPinConfigure(GPIO_PB2_T3CCP0);
  ROM_GPIOPinConfigure(GPIO_PB3_T3CCP1);
  /*
    Configure PB2 and PB3: direction controlled by hardware, 2mA drive
    strength, normal drive (no pullup).
  */
  ROM_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2);
  ROM_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_3);
  /* Configure 2 * 16-bit timer in PWM mode. */
  ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR |
                     TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
  /*
    Set timer3A/B start and compare values.
    High at 3, low at 1 -> 80MHz/4 = 20MHz clock.
  */
  ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, 3);
  ROM_TimerMatchSet(TIMER3_BASE, TIMER_A, 1);
  ROM_TimerLoadSet(TIMER3_BASE, TIMER_B, 3);
  ROM_TimerMatchSet(TIMER3_BASE, TIMER_B, 1);

  /* Start the timer. */
  ROM_TimerEnable(TIMER3_BASE, TIMER_BOTH);
}


static void
config_tlc_gpio(void)
{
  /* Setup GPIO pins for SSI to TLCs. */
  ROM_SysCtlPeripheralEnable(SSI_TLC1_PERIPH);
  ROM_SysCtlPeripheralEnable(SSI_TLC1_IO_PERIPH);
  ROM_GPIOPinConfigure(SSI_TLC1_CLK_CFG);
  ROM_GPIOPinConfigure(SSI_TLC1_RX_CFG);
  ROM_GPIOPinConfigure(SSI_TLC1_TX_CFG);
  ROM_GPIOPinTypeSSI(SSI_TLC1_IO_BASE,
                     SSI_TLC1_CLK_PIN | SSI_TLC1_RX_PIN | SSI_TLC1_TX_PIN);

  ROM_SysCtlPeripheralEnable(SSI_TLC2_PERIPH);
  ROM_SysCtlPeripheralEnable(SSI_TLC2_IO_PERIPH);
  ROM_GPIOPinConfigure(SSI_TLC2_CLK_CFG);
  ROM_GPIOPinConfigure(SSI_TLC2_RX_CFG);
  ROM_GPIOPinConfigure(SSI_TLC2_TX_CFG);
  ROM_GPIOPinTypeSSI(SSI_TLC2_IO_BASE,
                     SSI_TLC2_CLK_PIN | SSI_TLC2_RX_PIN | SSI_TLC2_TX_PIN);

  ROM_SysCtlPeripheralEnable(SSI_TLC3_PERIPH);
  ROM_SysCtlPeripheralEnable(SSI_TLC3_IO_PERIPH);
  ROM_GPIOPinConfigure(SSI_TLC3_CLK_CFG);
  ROM_GPIOPinConfigure(SSI_TLC3_RX_CFG);
  ROM_GPIOPinConfigure(SSI_TLC3_TX_CFG);
  ROM_GPIOPinTypeSSI(SSI_TLC3_IO_BASE,
                     SSI_TLC3_CLK_PIN | SSI_TLC3_RX_PIN | SSI_TLC3_TX_PIN);

  /* GPIO for XLAT and MODE. */
  ROM_SysCtlPeripheralEnable(GPIO_XLAT1_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(GPIO_XLAT1_BASE, GPIO_XLAT1_PIN);
  ROM_SysCtlPeripheralEnable(GPIO_MODE1_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(GPIO_MODE1_BASE, GPIO_MODE1_PIN);
  ROM_SysCtlPeripheralEnable(GPIO_XLAT2_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(GPIO_XLAT2_BASE, GPIO_XLAT2_PIN);
  ROM_SysCtlPeripheralEnable(GPIO_MODE2_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(GPIO_MODE2_BASE, GPIO_MODE2_PIN);
  ROM_SysCtlPeripheralEnable(GPIO_XLAT3_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(GPIO_XLAT3_BASE, GPIO_XLAT3_PIN);
  ROM_SysCtlPeripheralEnable(GPIO_MODE3_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(GPIO_MODE3_BASE, GPIO_MODE3_PIN);

  /* Set XLAT low. */
  ROM_GPIOPinWrite(GPIO_XLAT1_BASE, GPIO_XLAT1_PIN, 0);
  ROM_GPIOPinWrite(GPIO_XLAT2_BASE, GPIO_XLAT2_PIN, 0);
  ROM_GPIOPinWrite(GPIO_XLAT3_BASE, GPIO_XLAT3_PIN, 0);
  /* Set MODE low, to select GS mode. */
  ROM_GPIOPinWrite(GPIO_MODE1_BASE, GPIO_MODE1_PIN, 0);
  ROM_GPIOPinWrite(GPIO_MODE2_BASE, GPIO_MODE2_PIN, 0);
  ROM_GPIOPinWrite(GPIO_MODE3_BASE, GPIO_MODE3_PIN, 0);

  /* Setup BLANK, pull it high initially. */
  ROM_SysCtlPeripheralEnable(GPIO_BLANK1_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(GPIO_BLANK1_BASE, GPIO_BLANK1_PIN);
  ROM_GPIOPinWrite(GPIO_BLANK1_BASE, GPIO_BLANK1_PIN, GPIO_BLANK1_PIN);
  ROM_SysCtlPeripheralEnable(GPIO_BLANK2_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(GPIO_BLANK2_BASE, GPIO_BLANK2_PIN);
  ROM_GPIOPinWrite(GPIO_BLANK2_BASE, GPIO_BLANK2_PIN, GPIO_BLANK2_PIN);
  ROM_SysCtlPeripheralEnable(GPIO_BLANK3_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(GPIO_BLANK3_BASE, GPIO_BLANK3_PIN);
  ROM_GPIOPinWrite(GPIO_BLANK3_BASE, GPIO_BLANK3_PIN, GPIO_BLANK3_PIN);
}


static void
config_spi_tlc_read(uint32_t base, uint32_t bits_per_word)
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

  ROM_SSIDisable(base);
  ROM_SSIConfigSetExpClk(base, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_1,
                     SSI_MODE_MASTER, 20000000, bits_per_word);
  ROM_SSIEnable(base);
}

static void
config_spi_tlc_write(uint32_t base, uint32_t bits_per_word)
{
  /*
    Configure the SPI for correct mode to write to TLC5940, and send
    DC data.

    We need CLK inactive low, so SPO=0.
    We need to setup on the trailing, falling CLK edge, so SPH=0.
  */

  ROM_SSIDisable(base);
  ROM_SSIConfigSetExpClk(base, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 20000000, bits_per_word);
  ROM_SSIEnable(base);
}

/*
  Set the DC register on TLC5940 through SSI.
  Then read out the TLC5940 status register and check that DC is correct.
*/
static void
init_tlc_dc(uint32_t ssi_base, uint8_t dc_value,
            uint32_t mode_base, uint32_t mode_pin,
            uint32_t xlat_base, uint32_t xlat_pin)
{
  uint32_t data;
  uint32_t i;
  uint32_t value;
  uint32_t bits_collected;
  uint32_t error_retry_count=3;
  uint32_t dc_errors;

try_again:
  /* Prepare to write 6-bit DC words to TLC. */
  config_spi_tlc_write(ssi_base, 6);

  /*
    Clock out an initial dummy byte, just to get the clock and data
    signals stable.
  */
  ROM_SSIDataPut(ssi_base, 0);
  while (ROM_SSIBusy(ssi_base))
    ;
  /* Drain the receive FIFO just so it does not get full. */
  ROM_SSIDataGet(ssi_base, &data);

  /* Set MODE high, to select DC mode. */
  ROM_GPIOPinWrite(mode_base, mode_pin, mode_pin);
  ROM_SysCtlDelay(5);

  /* Write DC value (6 bits) to all 16 outputs and all 6 TLCs. */
  for (i = 0; i < LEDS_PER_TLC * NUM_TLC; ++i)
  {
    ROM_SSIDataPut(ssi_base, dc_value);
    while (ROM_SSIBusy(ssi_base))
      ;
    /* Drain the receive FIFO just so it does not get full. */
    ROM_SSIDataGet(ssi_base, &data);
  }

  /* Pulse XLAT so we get the DC values stored in the DC registers. */
  ROM_SysCtlDelay(5);
  ROM_GPIOPinWrite(xlat_base, xlat_pin, xlat_pin);
  ROM_SysCtlDelay(5);
  ROM_GPIOPinWrite(xlat_base, xlat_pin, 0);
  ROM_SysCtlDelay(5);

  config_spi_tlc_read(ssi_base, 8);

  /* Set MODE low, to select GS mode. */
  ROM_GPIOPinWrite(mode_base, mode_pin, 0);
  ROM_SysCtlDelay(5);

  /* Empty the receive FIFO. */
  while(ROM_SSIDataGetNonBlocking(ssi_base, &data))
    ;

  /* Pulse XLAT so we get the status register loaded into the shift register. */
  ROM_GPIOPinWrite(xlat_base, xlat_pin, xlat_pin);
  ROM_SysCtlDelay(5);
  ROM_GPIOPinWrite(xlat_base, xlat_pin, 0);
  ROM_SysCtlDelay(5);

  bits_collected = 0;
  value = 0;
  dc_errors = 0;
  for (i = 0; i < 12*LEDS_PER_TLC*NUM_TLC/8; ++i)
  {
    uint32_t j;

    ROM_SSIDataPut(ssi_base, 0);
    while (ROM_SSIBusy(ssi_base))
      ;
    ROM_SSIDataGet(ssi_base, &data);
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
        if (value != dc_value)
        {
          ROM_UARTCharPut(UART0_BASE, '#');
          serial_output_hexbyte(value);
          ROM_UARTCharPut(UART0_BASE, '#');
          ROM_UARTCharPut(UART0_BASE, '\n');
          ++dc_errors;
        }
        value = 0;
        bits_collected = 0;
      }
    }
  }
  ROM_UARTCharPut(UART0_BASE, '!');
  ROM_UARTCharPut(UART0_BASE, '\n');

  if (dc_errors > 0)
  {
    if (error_retry_count > 0)
    {
      --error_retry_count;
      goto try_again;
    }
    /* Flash the red LED endlessly to complain. */
    for (;;)
    {
      ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_RED);
      ROM_SysCtlDelay(40000000/3);
      ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, 0);
      ROM_SysCtlDelay(40000000/3);
    }
  }

  /* Leave the SPI in GS write mode. */
  config_spi_tlc_write(ssi_base, 8);
}

static uint32_t udma_control_block[256] __attribute__ ((aligned(1024)));
static void
init_udma_for_tlc(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  ROM_uDMAEnable();
  ROM_uDMAControlBaseSet(udma_control_block);

  ROM_SSIDMAEnable(SSI_TLC1_BASE, SSI_DMA_TX);
  ROM_IntEnable(SSI_TLC1_INT);
  ROM_SSIDMAEnable(SSI_TLC2_BASE, SSI_DMA_TX);
  ROM_IntEnable(SSI_TLC2_INT);
  ROM_SSIDMAEnable(SSI_TLC3_BASE, SSI_DMA_TX);
  ROM_IntEnable(SSI_TLC3_INT);

  ROM_uDMAChannelAttributeDisable(SSI_TLC1_DMA, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(SSI_TLC1_DMA_CHAN_ASSIGN);
  ROM_uDMAChannelAttributeEnable(SSI_TLC1_DMA, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(SSI_TLC1_DMA | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  ROM_uDMAChannelAttributeDisable(SSI_TLC2_DMA, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(SSI_TLC2_DMA_CHAN_ASSIGN);
  ROM_uDMAChannelAttributeEnable(SSI_TLC2_DMA, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(SSI_TLC2_DMA | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);

  ROM_uDMAChannelAttributeDisable(SSI_TLC3_DMA, UDMA_ATTR_ALTSELECT |
                                  UDMA_ATTR_REQMASK | UDMA_ATTR_HIGH_PRIORITY);
  ROM_uDMAChannelAssign(SSI_TLC3_DMA_CHAN_ASSIGN);
  ROM_uDMAChannelAttributeEnable(SSI_TLC3_DMA, UDMA_ATTR_USEBURST);
  ROM_uDMAChannelControlSet(SSI_TLC3_DMA | UDMA_PRI_SELECT,
                            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                            UDMA_ARB_4);
}

static volatile uint8_t tlc1_udma_running = 0;
static volatile uint8_t tlc2_udma_running = 0;
static volatile uint8_t tlc3_udma_running = 0;

static void
wait_for_spi_to_tlcs(volatile uint8_t *running_flag, uint32_t ssi_base)
{
  while (tlc1_udma_running)
    ;
  while (ROM_SSIBusy(ssi_base))
    ;
}


 __attribute__ ((unused))
static void
start_shift_out_gs_to_tlc_udma(uint8_t *gs_data, uint32_t ssi_dma,
                               uint32_t ssi_base,
                               volatile uint8_t *running_flag)
{
  ROM_uDMAChannelTransferSet(ssi_dma | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                             gs_data, (void *)(ssi_base + SSI_O_DR),
                             TLC_GS_BYTES);
  *running_flag = 1;
  ROM_uDMAChannelEnable(ssi_dma);
}


static void
latch_data_to_tlcs(void)
{
  /* Pulse XLAT while holding BLANK high to latch new GS data. */
  ROM_GPIOPinWrite(GPIO_BLANK1_BASE, GPIO_BLANK1_PIN, GPIO_BLANK1_PIN);
  ROM_GPIOPinWrite(GPIO_BLANK2_BASE, GPIO_BLANK2_PIN, GPIO_BLANK2_PIN);
  ROM_GPIOPinWrite(GPIO_BLANK3_BASE, GPIO_BLANK3_PIN, GPIO_BLANK3_PIN);

  ROM_GPIOPinWrite(GPIO_XLAT1_BASE, GPIO_XLAT1_PIN, GPIO_XLAT1_PIN);
  ROM_GPIOPinWrite(GPIO_XLAT2_BASE, GPIO_XLAT2_PIN, GPIO_XLAT2_PIN);
  ROM_GPIOPinWrite(GPIO_XLAT3_BASE, GPIO_XLAT3_PIN, GPIO_XLAT3_PIN);
  ROM_GPIOPinWrite(GPIO_XLAT1_BASE, GPIO_XLAT1_PIN, 0);
  ROM_GPIOPinWrite(GPIO_XLAT2_BASE, GPIO_XLAT2_PIN, 0);
  ROM_GPIOPinWrite(GPIO_XLAT3_BASE, GPIO_XLAT3_PIN, 0);

  ROM_GPIOPinWrite(GPIO_BLANK1_BASE, GPIO_BLANK1_PIN, 0);
  ROM_GPIOPinWrite(GPIO_BLANK2_BASE, GPIO_BLANK2_PIN, 0);
  ROM_GPIOPinWrite(GPIO_BLANK3_BASE, GPIO_BLANK3_PIN, 0);
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
  memset(buf, 0, TLC_GS_BYTES);
}


 __attribute__ ((unused))
static void
anim1(uint8_t *buf, uint32_t count)
{
  static const uint32_t val = 4095;
  gs_clear(buf);
  switch ((count/450/NUM_RGB_LEDS)%3)
  {
  case 0:
    set_led(buf, ((count/450) % NUM_RGB_LEDS), val, 0, 0);
    break;
  case 1:
    set_led(buf, ((count/450) % NUM_RGB_LEDS), 0, val, 0);
    break;
  case 2:
    set_led(buf, ((count/450) % NUM_RGB_LEDS), 0, 0, val);
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
    static const uint32_t speed = 100;
    static const uint32_t W = 12*speed;
    static const uint32_t value = 4095;

    uint32_t v = (i*speed + count) % (W*6);
    float rampdown = (value/2.0f)*(1.0f+cosf((v % W)*(float)(M_PI/W)));
    float rampup = (value/2.0f)*(1.0f-cosf((v % W)*(float)(M_PI/W)));

    switch (v/W)
    {
    case 0:
      set_led(buf, i,     value,   rampup,        0);  break;
    case 1:
      set_led(buf, i, rampdown,     value,        0);  break;
    case 2:
      set_led(buf, i,        0,     value,   rampup);  break;
    case 3:
      set_led(buf, i,        0, rampdown,     value);  break;
    case 4:
      set_led(buf, i,   rampup,        0,     value);  break;
    case 5:
      set_led(buf, i,     value,        0, rampdown);  break;
    }
  }
}

 __attribute__ ((unused))
static void
anim3(uint8_t *buf, uint32_t count)
{
  gs_clear(buf);
  //set_led(buf, 0, 0, 0x0ff, 0);
  //set_led(buf, 31, 0, 0, 0x100);
  buf[0] = 0xff; buf[TLC_GS_BYTES-1] = 0xff;
}


uint32_t scanline_time;

 __attribute__ ((unused))
static void
anim4(uint8_t *buf, uint32_t count)
{
  float angle = 2.0f*(float)M_PI*(count % 30000)/30000.0f;
  uint32_t t_start, t_stop;

  t_start = HWREG(WTIMER0_BASE + TIMER_O_TAV);
  bm_scanline(angle, 32, buf);
  t_stop = HWREG(WTIMER0_BASE + TIMER_O_TAV);
  scanline_time = t_start - t_stop;
}


static void
led_stuff(void)
{
  static uint32_t counter = 0;

  /* Flash the LED a bit. */
  if (counter == 0 || counter == 300)
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE,
                     ROM_GPIOPinRead(GPIO_PORTF_BASE, LED_BLUE) ^ LED_BLUE);
  ++counter;
  if (counter == MCU_HZ/4/4096)
    counter = 0;
}


static volatile uint8_t do_next_frame = 0;
static volatile uint8_t current_tlc_frame_buf = 0;
static uint8_t tlc1_frame_buf[2][TLC_GS_BYTES];
static uint8_t tlc2_frame_buf[2][TLC_GS_BYTES];
static uint8_t tlc3_frame_buf[2][TLC_GS_BYTES];

void
IntHandlerTimer2A(void)
{
  uint8_t cur;
  static uint32_t anim_counter = 0;
  uint32_t current_time, start_time, current_period;
  uint32_t estim_latch_time, estim_delta;
  float angle;
  uint32_t tmp;
#ifdef FAKE_HALL_SENSOR
  static uint32_t fake_hall_counter = 0;
#endif

  ROM_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

  /*
    Latch in the new data. Be sure to wait first for it to be ready.

    (Though if we have to wait, that indicates a timing problem, not enough
    time to do one round of data until next timer interrupt. Eventually,
    we should detect this so we can fix it if it occurs, relax timing.)
  */
  wait_for_spi_to_tlcs(&tlc1_udma_running, SSI_TLC1_BASE);
  wait_for_spi_to_tlcs(&tlc2_udma_running, SSI_TLC2_BASE);
  wait_for_spi_to_tlcs(&tlc3_udma_running, SSI_TLC3_BASE);
  latch_data_to_tlcs();
  current_time = HWREG(WTIMER0_BASE + TIMER_O_TAV);
  start_time = last_hall;
  current_period = last_hall_period;

  cur = current_tlc_frame_buf;
  start_shift_out_gs_to_tlc_udma(tlc1_frame_buf[cur], SSI_TLC1_DMA,
                                 SSI_TLC1_BASE, &tlc1_udma_running);
  start_shift_out_gs_to_tlc_udma(tlc2_frame_buf[cur], SSI_TLC2_DMA,
                                 SSI_TLC2_BASE, &tlc2_udma_running);
  start_shift_out_gs_to_tlc_udma(tlc3_frame_buf[cur], SSI_TLC3_DMA,
                                 SSI_TLC3_BASE, &tlc3_udma_running);
  cur = 1 - cur;
  current_tlc_frame_buf = cur;

  led_stuff();
  ++do_next_frame;

  /*
    Estimate the angle of rotation when the scanline we are about to generate
    is latched.

    This timer interrupt we are writing the data into the buffer, the next
    interrupt we will shift it out, so latch will be in two timer periods.
  */
  estim_latch_time = current_time - 2*4*4096;
  estim_delta = start_time - estim_latch_time;
  if (current_period < 1)
    current_period = 1;
  if (estim_delta > current_period)
    estim_delta = current_period;
  angle = (float)(2.0f*M_PI) * (float)estim_delta / (float)current_period;
  bm_scanline(angle, 32, tlc1_frame_buf[cur]);
  bm_scanline(angle+(M_PI*2.0f/3.0f), 32, tlc2_frame_buf[cur]);
  bm_scanline(angle+(M_PI*4.0f/3.0f), 32, tlc3_frame_buf[cur]);

  ++anim_counter;

  /* Show the status of the HALL sensor. */
#ifndef FAKE_HALL_SENSOR
  if (ROM_GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4))
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN, LED_GREEN);
  else
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN, LED_RED);
#endif

  /* Restart HALL capture a little while after last event. */
  tmp = hall_capture_delay;
  if (tmp)
    hall_capture_delay = tmp-1;
  else
  {
    ROM_TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT);
    ROM_IntEnable(INT_WTIMER0A);
  }
#ifdef FAKE_HALL_SENSOR
  if (++fake_hall_counter > 5000)
  {
    fake_hall_counter = 0;
    record_hall_sensor(current_time);
  }
#endif
}


void
IntHandlerSSI0(void)
{
  ROM_SSIIntClear(SSI_TLC1_BASE, ROM_SSIIntStatus(SSI_TLC1_BASE, 1));
  if (!ROM_uDMAChannelIsEnabled(SSI_TLC1_DMA))
    tlc1_udma_running = 0;
}


void
IntHandlerSSI2(void)
{
  ROM_SSIIntClear(SSI_TLC2_BASE, ROM_SSIIntStatus(SSI_TLC2_BASE, 1));
  if (!ROM_uDMAChannelIsEnabled(SSI_TLC2_DMA))
    tlc2_udma_running = 0;
}


void
IntHandlerSSI3(void)
{
  ROM_SSIIntClear(SSI_TLC3_BASE, ROM_SSIIntStatus(SSI_TLC3_BASE, 1));
  if (!ROM_uDMAChannelIsEnabled(SSI_TLC3_DMA))
    tlc3_udma_running = 0;
}


 __attribute__ ((unused))
static uint32_t
mandelbrot_val(float cx, float cy, uint32_t N)
{
  float zx = 0;
  float zy = 0;
  uint32_t i = 1;

  do
  {
    float a;

    if (zx*zx + zy*zy > 4.0f)
      break;
    a = zx*zx - zy*zy + cx;
    zy = 2*zx*zy + cy;
    zx = a;
    ++i;
  } while (i <= N);

  return i;
}


int main()
{
  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  generate_test_image();

  /* Setup GPIO to read HALL sensor. */
  setup_hall_gpio_n_timer();

  /* Configure LED GPIOs. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_BLUE|LED_GREEN);
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, 0);

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

  config_tlc_gpio();
  /* A small delay seems to help before communicating with the TLCs. */
  ROM_SysCtlDelay(MCU_HZ/3/10);
  serial_output_str("Loading TLC 1...\r\n");
  init_tlc_dc(SSI_TLC1_BASE, DC_VALUE,
              GPIO_MODE1_BASE, GPIO_MODE1_PIN,
              GPIO_XLAT1_BASE, GPIO_XLAT1_PIN);
#ifndef SINGLE_BLADE
  serial_output_str("Loading TLC 2...\r\n");
  init_tlc_dc(SSI_TLC2_BASE, DC_VALUE,
              GPIO_MODE2_BASE, GPIO_MODE2_PIN,
              GPIO_XLAT2_BASE, GPIO_XLAT2_PIN);
  serial_output_str("Loading TLC 3...\r\n");
  init_tlc_dc(SSI_TLC3_BASE, DC_VALUE,
              GPIO_MODE3_BASE, GPIO_MODE3_PIN,
              GPIO_XLAT3_BASE, GPIO_XLAT3_PIN);
#endif
  serial_output_str("TLC load ok!\r\n");
  init_udma_for_tlc();

  /*
    Once we start the timer 1A, we will get interrupts that sends data
    to the TLCs and latch it. So we must do this after everything else
    is set up correctly.
  */
  setup_pwm_GSCLK23();
  setup_pwm_GSCLK1_n_timer();

  for (;;) {
    static uint32_t prior_hall= 0xffffffffUL;
    uint32_t hall;

    hall = last_hall;
    if (hall != prior_hall)
    {
      ROM_UARTCharPut(UART0_BASE, '>');
      println_float(last_hall_period/(float)MCU_HZ, 2, 4);
      println_uint32(hall_int_counts);
      prior_hall = hall;

      println_uint32(scanline_time);
    }
  }
}
