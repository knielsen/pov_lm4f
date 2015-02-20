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
#include "inc/hw_nvic.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"
#include "driverlib/interrupt.h"

#include "pov_config.h"
#include "gfx.h"
#include "nrf24l01p.h"


#define NUM_LEDS 32
#define LEDS_PER_TLC 16
#define NUM_TLC ((NUM_LEDS*3+(LEDS_PER_TLC-1))/LEDS_PER_TLC)
#define TLC_GS_BYTES (12 * LEDS_PER_TLC * NUM_TLC / 8)
#define NUM_RGB_LEDS (NUM_TLC*LEDS_PER_TLC/3)
#define PWM_CYCLES (4*4096)

/*
  Useful for testing.
  When defined, the actual hall sensor is ignored, instead it fakes that
  the hall triggers with some reasonable frequency.
*/
//#define FAKE_HALL_SENSOR 1
/* Define to use only a single blade, for testing. */
//#define SINGLE_BLADE 1
/* Define if no nRF24L01+ connected. */
//#define DISABLE_NRF 1
//#define HALL1 1
#define HALL2 1
//#define HALL3 1

/*
  Current pinouts:

                  PCB#1  PCB#2  PCB#3
   SIN             PA5    PB7    PD3
   GSCLK (20MHz)   PC4    PF4    PB5
   MODE            PA6    PC7    PD1
   SCLK            PA2    PB4    PD0
   XLAT            PA3    PE2    PE4
   BLANK           PC6    PE3    PE5
   SOUT            PA4    PB6    PD2

   HALL            PC5   (PD7)  (PD6)

  nRF24L01 pinout:

  Rx:
    PF2  SCK        GND *1 2. VCC
    PF3  CSN        PB3 .3 4. PF3
    PF0  MISO       PF2 .5 6. PF1
    PF1  MOSI       PF0 .7 8. PB0
    PB0  IRQ
    PB3  CE
*/


#define GPIO_MODE1_PERIPH SYSCTL_PERIPH_GPIOA
#define GPIO_MODE1_BASE GPIO_PORTA_BASE
#define GPIO_MODE1_PIN GPIO_PIN_6
#define GPIO_MODE2_PERIPH SYSCTL_PERIPH_GPIOC
#define GPIO_MODE2_BASE GPIO_PORTC_BASE
#define GPIO_MODE2_PIN GPIO_PIN_7
#define GPIO_MODE3_PERIPH SYSCTL_PERIPH_GPIOD
#define GPIO_MODE3_BASE GPIO_PORTD_BASE
#define GPIO_MODE3_PIN GPIO_PIN_1

#define GPIO_XLAT1_PERIPH SYSCTL_PERIPH_GPIOA
#define GPIO_XLAT1_BASE GPIO_PORTA_BASE
#define GPIO_XLAT1_PIN GPIO_PIN_3
#define GPIO_XLAT2_PERIPH SYSCTL_PERIPH_GPIOE
#define GPIO_XLAT2_BASE GPIO_PORTE_BASE
#define GPIO_XLAT2_PIN GPIO_PIN_2
#define GPIO_XLAT3_PERIPH SYSCTL_PERIPH_GPIOE
#define GPIO_XLAT3_BASE GPIO_PORTE_BASE
#define GPIO_XLAT3_PIN GPIO_PIN_4

#define GPIO_BLANK1_PERIPH SYSCTL_PERIPH_GPIOC
#define GPIO_BLANK1_BASE GPIO_PORTC_BASE
#define GPIO_BLANK1_PIN GPIO_PIN_6
#define GPIO_BLANK2_PERIPH SYSCTL_PERIPH_GPIOE
#define GPIO_BLANK2_BASE GPIO_PORTE_BASE
#define GPIO_BLANK2_PIN GPIO_PIN_3
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



/* A simple logging facility, a finite or cyclic buffer. */
static uint32_t tb[64*2];
static uint32_t tbi = 0;
 __attribute__ ((unused))
static void
tlog(uint32_t tag, uint32_t val)
{
  if (tbi >= (sizeof(tb)/sizeof(tb[0])))
    return;
  tb[tbi++] = tag;
  tb[tbi++] = val;
  // tbi = tbi % (sizeof(tb)/sizeof(tb[0]));
}


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
static char *
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


 __attribute__ ((unused))
static void
println_uint32(uint32_t val)
{
  char buf[13];
  char *p = uint32_tostring(buf, val);
  *p++ = '\r';
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


/*
  Configure wtimer0a to output a PWM signal for gsclk1 on PC4.
  Configure wtimer0b for capture of the hall1 sensor on PC5.
*/
static void
setup_hall_gpio_n_gsclk1(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  ROM_GPIOPinConfigure(GPIO_PC4_WT0CCP0);
  ROM_GPIOPinConfigure(GPIO_PC5_WT0CCP1);
  ROM_GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);
  ROM_GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_8MA,
                       GPIO_PIN_TYPE_STD);

  /*
    I could not get the timer capture to work in count-up mode.
    Perhaps count-up for capture is just not supported on my chip, though
    I did not see this anywhere in the datasheet...
  */

  ROM_TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR |
                     TIMER_CFG_A_PWM | TIMER_CFG_B_CAP_TIME);
  /*
    Set wtimer0A start and compare values.
    High at 3, low at 1 -> 80MHz/4 = 20MHz clock.
  */
  ROM_TimerLoadSet(WTIMER0_BASE, TIMER_A, 3);
  ROM_TimerMatchSet(WTIMER0_BASE, TIMER_A, 1);
  /* Set wtimer0B to count the full 32-bit range ffffffff..0 */
  ROM_TimerLoadSet(WTIMER0_BASE, TIMER_B, 0xffffffffUL);
  ROM_TimerMatchSet(WTIMER0_BASE, TIMER_B, 0);
  ROM_TimerControlEvent(WTIMER0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);

  ROM_IntMasterEnable();
#ifdef HALL1
  ROM_TimerIntEnable(WTIMER0_BASE, TIMER_CAPB_EVENT);
  ROM_IntEnable(INT_WTIMER0B);
#endif

  ROM_TimerEnable(WTIMER0_BASE, TIMER_BOTH);
}


/*
  Configure wtimer5a for capture of the hall3 sensor on PD6.
  Configure wtimer5b for capture of the hall2 sensor on PD7.
*/
static void
setup_hall23(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

  /* PD7 is special (NMI), needs unlock to be re-assigned to timer. */
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
  HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

  ROM_GPIOPinConfigure(GPIO_PD6_WT5CCP0);
  ROM_GPIOPinConfigure(GPIO_PD7_WT5CCP1);
  ROM_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7);

  /*
    I could not get the timer capture to work in count-up mode.
    Perhaps count-up for capture is just not supported on my chip, though
    I did not see this anywhere in the datasheet...
  */

  ROM_TimerConfigure(WTIMER5_BASE, TIMER_CFG_SPLIT_PAIR |
                     TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_CAP_TIME);
  /* Set wtimer5A/B to count the full 32-bit range ffffffff..0 */
  ROM_TimerLoadSet(WTIMER5_BASE, TIMER_A, 0xffffffffUL);
  ROM_TimerMatchSet(WTIMER5_BASE, TIMER_A, 0);
  ROM_TimerControlEvent(WTIMER5_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

  ROM_TimerLoadSet(WTIMER5_BASE, TIMER_B, 0xffffffffUL);
  ROM_TimerMatchSet(WTIMER5_BASE, TIMER_B, 0);
  ROM_TimerControlEvent(WTIMER5_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);

  ROM_IntMasterEnable();
#ifdef HALL3
  ROM_TimerIntEnable(WTIMER5_BASE, TIMER_CAPA_EVENT);
#endif
#ifdef HALL2
  ROM_TimerIntEnable(WTIMER5_BASE, TIMER_CAPB_EVENT);
#endif
#if defined(HALL2) || defined(HALL3)
  ROM_IntEnable(INT_WTIMER5B);
#endif

  ROM_TimerEnable(WTIMER5_BASE, TIMER_BOTH);
}


static volatile uint32_t last_hall = 0xffffffffUL;
static volatile uint32_t last_hall_period = 0;
static volatile uint32_t prevlast_hall_period = 0;
static volatile uint32_t hall_int_counts = 0;

static volatile uint32_t hall_capture_delay = 0;
static const uint32_t hall_deglitch_delay = 7;  /* Units of PWM period. */

static uint32_t
record_hall_sensor(uint32_t timer_val)
{
  uint32_t period;
  uint32_t b = prevlast_hall_period;
  uint32_t a = last_hall_period;

  period = last_hall - timer_val;
  /*
    De-glitch some spurious hall signals that may come from stray magnetic
    fields from motor coils or current supply wire.

    When we are running with stable rotation speed, ignore signals that
    are too far from what we expect.
  */
  if (period >= 11000000UL && period <= 19000000UL)
  {
    uint32_t olddel = b >= a ? b - a : a - b;
    uint32_t newdel = period >= a ? period - a : a - period;
    /*
      Avoid errorneously skipping when we are changing speed, by accepting
      jitter in period up to 3 times the difference between last two periods.

      Otherwise discard a period that is more than 1% off the last one.
    */
    if (newdel > olddel * 3 && newdel > a / 100)
    {
      //tlog(4, period);
      return 0;
    }
  }

  prevlast_hall_period = a;
  last_hall_period = period;
  last_hall = timer_val;
  ++hall_int_counts;
  //tlog(3, period);
  return 1;
}


void
IntHandlerWTimer0B(void)
{
  /*
    Wait a little while before taking the interrupt again.
    This to work-around a very unstable signal around the low->high
    transition.
  */
  ROM_TimerIntClear(WTIMER0_BASE, TIMER_CAPB_EVENT);

#ifndef FAKE_HALL_SENSOR
  if (!record_hall_sensor(ROM_TimerValueGet(WTIMER0_BASE, TIMER_B)))
  {
    ROM_IntDisable(INT_WTIMER0B);
    hall_capture_delay = hall_deglitch_delay;
  }
#endif
}


void
IntHandlerWTimer5B(void)
{
  /*
    Wait a little while before taking the interrupt again.
    This to work-around a very unstable signal around the low->high
    transition.
  */
  ROM_IntDisable(INT_WTIMER5B);
  ROM_TimerIntClear(WTIMER5_BASE, TIMER_CAPA_EVENT|TIMER_CAPB_EVENT);
  hall_capture_delay = hall_deglitch_delay;

#ifndef FAKE_HALL_SENSOR
  if (
#ifdef HALL3
      record_hall_sensor(ROM_TimerValueGet(WTIMER5_BASE, TIMER_A))
#endif
#ifdef HALL2
      record_hall_sensor(ROM_TimerValueGet(WTIMER5_BASE, TIMER_B))
#endif
      )
  {
  ROM_IntDisable(INT_WTIMER5B);
  hall_capture_delay = hall_deglitch_delay;
  }
#endif
}


/*
  Configure Timer2A as gsclk2 on PF4.
  Configure Timer2B to trigger interrupt at the end of each TLC5940 PWM period.
*/
static void
setup_pwm_GSCLK2_n_timer(void)
{
  /* Enable the timer. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  /* Enable the GPIO module for the timer pin. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  /* Configure PF4 to be I/O pin for timer 2 A. */
  ROM_GPIOPinConfigure(GPIO_PF4_T2CCP0);
  /*
    Configure PB1: direction controlled by hardware, 8mA drive strength,
    normal drive (no pullup).
  */
  ROM_GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_4);
  ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_8MA,
                       GPIO_PIN_TYPE_STD);

  /* Configure 16-bit timer A as PWM, and B in periodic mode. */
  ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR |
                     TIMER_CFG_A_PWM | TIMER_CFG_B_PERIODIC);

  /*
    Set timer2A start and compare values.
    High at 3, low at 1 -> 80MHz/4 = 20MHz clock.
  */
  ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, 3);
  ROM_TimerMatchSet(TIMER2_BASE, TIMER_A, 1);

  /* Set timer interrupt at GSCLK PWM period. */
  ROM_TimerLoadSet(TIMER2_BASE, TIMER_B, PWM_CYCLES);
  /*
    Set the ILD bit in timer2B config register, so that we can change the
    timer period on-the-fly, and the new period length will take effect at the
    start of the next period.
  */
  HWREG(TIMER2_BASE + TIMER_O_TBMR) |= TIMER_TBMR_TBILD;

  /* Enable interrrupts. */
  ROM_IntMasterEnable();
  ROM_TimerIntEnable(TIMER2_BASE, TIMER_TIMB_TIMEOUT);
  ROM_IntEnable(INT_TIMER2B);

  /* Start the timer. */
  ROM_TimerEnable(TIMER2_BASE, TIMER_BOTH);
}


/* Configure Timer1 to output a PWM signal for GSCLK3 on PB5. */
static void
setup_pwm_GSCLK3(void)
{
  /* Enable the timer. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  /* Enable the GPIO module for the timer pin. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  /* Configure PB5 to be I/O pin for timer 1 B. */
  ROM_GPIOPinConfigure(GPIO_PB5_T1CCP1);

  /*
    Configure PB2 and PB3: direction controlled by hardware, 8mA drive
    strength, normal drive (no pullup).
  */
  ROM_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_5);
  ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_8MA,
                       GPIO_PIN_TYPE_STD);

  /* Configure 16-bit timer in PWM mode. */
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM);
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
                     SSI_MODE_MASTER, 10000000, bits_per_word);
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
  Find the adjusted dot-correction value for a LED, to account for the
  fact that outer LEDs have to swipe a larger area than inner ones.
*/
static uint32_t
adjusted_dc_value(uint32_t led_idx, uint8_t dc_value)
{
  /* Let's round up to avoid too-dim LEDs in inner circles. */
  return ((led_idx+1)*dc_value+(NUM_LEDS-1))/NUM_LEDS;
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
  uint32_t led_num;

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
  led_num = NUM_LEDS*3-1;
  for (i = 0; i < LEDS_PER_TLC * NUM_TLC; ++i)
  {
    ROM_SSIDataPut(ssi_base, adjusted_dc_value(led_num/3, dc_value));
    --led_num;
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
  ROM_SysCtlDelay(5);
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
  led_num = NUM_LEDS*3-1;
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
    {
      ROM_UARTCharPut(UART0_BASE, '\r');
      ROM_UARTCharPut(UART0_BASE, '\n');
    }

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
        if (value != adjusted_dc_value(led_num/3, dc_value))
          ++dc_errors;
        --led_num;
        value = 0;
        bits_collected = 0;
      }
    }
  }
  ROM_UARTCharPut(UART0_BASE, '!');
  ROM_UARTCharPut(UART0_BASE, '\r');
  ROM_UARTCharPut(UART0_BASE, '\n');

  if (dc_errors > 0)
  {
    if (error_retry_count > 0)
    {
      --error_retry_count;
      goto try_again;
    }
    serial_output_str("All retries failed to correctly load TLC, giving up.\r\n");
    for (;;)
    {
    }
  }

  /* Leave the SPI in GS write mode. */
  config_spi_tlc_write(ssi_base, 8);
}

static uint32_t udma_control_block[256]
  __attribute__ ((section(".udma_ctl"), aligned(1024)));

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
  while (*running_flag)
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


/* nRF24L01+ code for receiving video frames wirelessly. */

static void
config_nrf_ssi_gpio(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  /* PF0 is special (NMI), needs unlock to be re-assigned to SSI1. */
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

  ROM_GPIOPinConfigure(GPIO_PF2_SSI1CLK);
  ROM_GPIOPinConfigure(GPIO_PF0_SSI1RX);
  ROM_GPIOPinConfigure(GPIO_PF1_SSI1TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_1);
  /* CSN pin, high initially */
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
  /* CE pin, low initially */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
  ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
  /* IRQ pin as input. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);

  /*
    Configure the SPI for correct mode to read from nRF24L01+.

    We need CLK inactive low, so SPO=0.
    We need to setup and sample on the leading, rising CLK edge, so SPH=0.

    The datasheet says up to 10MHz SPI is possible, depending on load
    capacitance. Let's go with a slightly cautious 8MHz, which should be
    aplenty.
  */
  ROM_SSIDisable(SSI1_BASE);
  ROM_SSIConfigSetExpClk(SSI1_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 8000000, 8);
  ROM_SSIEnable(SSI1_BASE);

  /*
    Tiva Errata says that SSI0 and SSI1 cannot use uDMA at the same time.
    So just set up SSI1 with interrupts, no uDMA.
  */
  ROM_IntEnable(INT_SSI1);
}


static void
bzero(uint8_t *buf, uint32_t len)
{
  while (len > 0)
  {
    *buf++ = 0;
    --len;
  }
}


static inline void
csn_low(uint32_t csn_base, uint32_t csn_pin)
{
  ROM_GPIOPinWrite(csn_base, csn_pin, 0);
}


static inline void
csn_high(uint32_t csn_base, uint32_t csn_pin)
{
  ROM_GPIOPinWrite(csn_base, csn_pin, csn_pin);
}


static inline void
ce_low(uint32_t ce_base, uint32_t ce_pin)
{
  ROM_GPIOPinWrite(ce_base, ce_pin, 0);
}


static inline void
ce_high(uint32_t ce_base, uint32_t ce_pin)
{
  ROM_GPIOPinWrite(ce_base, ce_pin, ce_pin);
}


static void
ssi_cmd(uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len,
        uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint32_t i;
  uint32_t data;

  /* Take CSN low to initiate transfer. */
  csn_low(csn_base, csn_pin);

  for (i = 0; i < len; ++i)
  {
    ROM_SSIDataPut(ssi_base, sendbuf[i]);
    while (ROM_SSIBusy(ssi_base))
      ;
    ROM_SSIDataGet(ssi_base, &data);
    recvbuf[i] = data;
  }

  /* Take CSN high to complete transfer. */
  csn_high(csn_base, csn_pin);

#ifdef SSI_DEBUG_OUTPUT
  /* For debug, output the data sent and received. */
  serial_output_str("Tx: ");
  for (i = 0; i < len; ++i)
  {
    serial_output_hexbyte(sendbuf[i]);
    if ((i %8) == 7)
      serial_output_str(" ");
  }
  serial_output_str("\r\nRx: ");
  for (i = 0; i < len; ++i)
  {
    serial_output_hexbyte(recvbuf[i]);
    if ((i %8) == 7)
      serial_output_str(" ");
  }
  serial_output_str("\r\n");
#endif
}


/*
  Asynchronous SSI transfer to nRF24L01+ using only fifo (no dma).
  Performs a transaction over SPI <= 8 bytes.
  First call nrf_async_start(). While nrf_async_start() or nrf_async_cont()
  returns zero, call nrf_async_cont() for each SSI interrupt event (only
  transfer done can occur).
*/
struct nrf_async_cmd {
  uint32_t ssi_base;
  uint32_t csn_base;
  uint32_t csn_pin;
  uint8_t *recvbuf;
  uint8_t ssi_active;
};


/*
  Start a command, max 8 bytes.
  recvbuf must remain valid for the duration of the operation, or it can be
  NULL to ignore anything received.
  sendbuf can be discarded once nrf_async_cmd_start() returns.
*/
static int32_t
nrf_async_cmd_start(struct nrf_async_cmd *a,
                    uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len,
                    uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  if (len > 8)
    return -1;
  a->ssi_base = ssi_base;
  a->csn_base = csn_base;
  a->csn_pin = csn_pin;
  a->recvbuf = recvbuf;
  a->ssi_active = 1;
  /* Take CSN low to initiate transfer. */
  csn_low(csn_base, csn_pin);
  /* Empty any left-over stuff in the RX FIFO. */
  while (HWREG(ssi_base + SSI_O_SR) & SSI_SR_RNE)
    (void)(HWREG(ssi_base + SSI_O_DR));

  /* Set up so we get an interrupt when last bit has been transmitted. */
  HWREG(ssi_base + SSI_O_CR1) |= SSI_CR1_EOT;
  HWREG(ssi_base + SSI_O_IM) |= SSI_TXFF;

  while (len > 0)
  {
    HWREG(ssi_base + SSI_O_DR) = *sendbuf++;
    --len;
  }

  return 0;
}


static int32_t
nrf_async_cmd_cont(struct nrf_async_cmd *a)
{
  uint8_t *p;

  if (!ROM_SSIBusy(a->ssi_base))
  {
    a->ssi_active = 0;
    /*
      Now that the transfer is completely done, set the TX interrupt back
      to disabled and trigger at 1/2 full fifo.
    */
    HWREG(a->ssi_base + SSI_O_IM) &= ~SSI_TXFF;
    HWREG(a->ssi_base + SSI_O_CR1) &= ~SSI_CR1_EOT;

    /* Take CSN high to complete transfer. */
    csn_high(a->csn_base, a->csn_pin);

    /* Empty the receive fifo (and return the data if so requested. */
    p = a->recvbuf;
    while (HWREG(a->ssi_base + SSI_O_SR) & SSI_SR_RNE)
    {
      uint8_t v = HWREG(a->ssi_base + SSI_O_DR);
      if (p)
        *p++ = v;
    }
    return 1;
  }
  else
    return 0;
}


/*
  Asynchronous SSI fetch of received packet from nRF24L01+ using interrupt
  (no dma, due to the Tiva errata that SSI0 and SSI1 cannot both use DMA at the
  same time).

  First call nrf_async_start(). While nrf_async_start() or nrf_async_cont()
  returns zero, call nrf_async_cont() for each SSI interrupt event (Tx fifo
  less that 1/2 full and end-of-transfer).
*/
struct nrf_async_rcv {
  uint32_t ssi_base;
  uint32_t csn_base;
  uint32_t csn_pin;
  uint8_t *status_out;
  uint8_t *packet_out;
  uint32_t remain_rx_bytes;
  uint32_t remain_tx_bytes;
};


/*
  Start a receive TX packet command (R_RX_PAYLOAD).
  The packet data is written to memory at `packet_out', size `len'.
  The value of register STATUS is written to *status_out, if non-NULL.
*/
static int32_t
nrf_async_rcv_start(struct nrf_async_rcv *a, uint8_t *status_out,
                    uint8_t *packet_out, uint32_t len,
                    uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  static uint8_t dummy_byte;

  a->ssi_base = ssi_base;
  a->csn_base = csn_base;
  a->csn_pin = csn_pin;
  a->status_out = status_out ? status_out : &dummy_byte;
  a->packet_out = packet_out;
  a->remain_rx_bytes = len+1;

  /* Take CSN low to enter receive mode. */
  csn_low(csn_base, csn_pin);
  /* Empty any left-over stuff in the RX FIFO. */
  while (HWREG(ssi_base + SSI_O_SR) & SSI_SR_RNE)
    (void)(HWREG(ssi_base + SSI_O_DR));

  if (len+1 > 8)
  {
    uint32_t i;

    HWREG(ssi_base + SSI_O_DR) = nRF_R_RX_PAYLOAD;
    for (i = 1; i < 8; ++i)
      HWREG(ssi_base + SSI_O_DR) = 0;
    a->remain_tx_bytes = len - (8-1);

    /* Set up to get an interrupt when the SSI TX fifo is 1/2 full or less. */
    HWREG(ssi_base + SSI_O_CR1) &= ~SSI_CR1_EOT;
    HWREG(ssi_base + SSI_O_IM) |= SSI_TXFF;
  }
  else
  {
    HWREG(ssi_base + SSI_O_DR) = nRF_R_RX_PAYLOAD;
    while (len > 0)
    {
      HWREG(ssi_base + SSI_O_DR) = 0;
      --len;
    }
    a->remain_tx_bytes = 0;

    /* Everything fits in the SSI FIFO, so just get an interrupt at the end. */
    HWREG(ssi_base + SSI_O_CR1) |= SSI_CR1_EOT;
    HWREG(ssi_base + SSI_O_IM) |= SSI_TXFF;
  }

  return 0;
}


/* Called to empty the SSI receive FIFO. */
static inline void
nrf_async_rcv_grab_rx_bytes(struct nrf_async_rcv *a)
{
  uint32_t rx_not_empty = HWREG(a->ssi_base + SSI_O_SR) & SSI_SR_RNE;
  if (rx_not_empty)
  {
    uint32_t remain_rx = a->remain_rx_bytes;
    if (a->status_out)
    {
      /* First byte is STATUS register. */
      *(a->status_out) = HWREG(a->ssi_base + SSI_O_DR);
      --remain_rx;
      a->status_out = NULL;
      rx_not_empty = HWREG(a->ssi_base + SSI_O_SR) & SSI_SR_RNE;
    }

    if (rx_not_empty)
    {
      uint8_t *p = a->packet_out;
      do
      {
        *p++ = HWREG(a->ssi_base + SSI_O_DR);
        --remain_rx;
      } while (HWREG(a->ssi_base + SSI_O_SR) & SSI_SR_RNE);
      a->packet_out = p;
    }
    a->remain_rx_bytes = remain_rx;
  }
}


static int32_t
nrf_async_rcv_cont(struct nrf_async_rcv *a)
{
  uint32_t remain_rx, remain_tx;

  /* First grab everything in the SSI receive fifo. */
  nrf_async_rcv_grab_rx_bytes(a);
  remain_rx = a->remain_rx_bytes;
  remain_tx = a->remain_tx_bytes;
  /*
    Now fill up the SSI transmit fifo with more (dummy) bytes.

    However, we need to ensure we never fill in more that 8 bytes ahead of
    what we have received. Otherwise we might end up with 8 bytes in the SSI
    transmit fifo + one more byte in the SSI output shift register. This would
    result in 9 receive bytes to be put into the SSI receive fifo, so if we
    were late in responding to interrupts we could end up overflowing the
    receive fifo and lose one byte.
  */
  if (remain_tx > 0 && remain_tx + 8 > remain_rx)
  {
    do
    {
      HWREG(a->ssi_base + SSI_O_DR) = 0;
      --remain_tx;
    } while (remain_tx > 0 && remain_tx + 8 > remain_rx);
    a->remain_tx_bytes = remain_tx;
    if (remain_tx == 0)
    {
      /*
        The entire length of the packet to be received has been queued.
        Switch over to get the next interrupt only at the very end of the
        transfer.

        Note that we fall through to test for end-of-transmission already in
        this interrupt invocation, so that we do not risk loosing the event
        if we happen to get slow and SSI already completed the transfer.
      */
      HWREG(a->ssi_base + SSI_O_CR1) |= SSI_CR1_EOT;
      HWREG(a->ssi_base + SSI_O_IM) |= SSI_TXFF;
    }
  }

  if (remain_tx == 0 && !ROM_SSIBusy(a->ssi_base))
  {
    /*
      Now that the transfer is completely done, set the TX interrupt back
      to disabled and trigger at 1/2 full fifo.
    */
    HWREG(a->ssi_base + SSI_O_IM) &= ~SSI_TXFF;
    HWREG(a->ssi_base + SSI_O_CR1) &= ~SSI_CR1_EOT;

    /* Take CSN high to complete transfer. */
    csn_high(a->csn_base, a->csn_pin);

    /* Grab any remaining data from the Rx FIFO. */
    nrf_async_rcv_grab_rx_bytes(a);
    return 1;
  }
  else
    return 0;
}


static void
nrf_flush_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_TX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_flush_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_RX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg_n(uint8_t reg, const uint8_t *data, uint32_t len,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[6], recvbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg(uint8_t reg, uint8_t val,
              uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg_n(reg, &val, 1, ssi_base, csn_base, csn_pin);
}


static int32_t
nrf_write_reg_n_start(struct nrf_async_cmd *a, uint8_t reg, const uint8_t *data,
                      uint8_t *recvbuf, uint32_t len, uint32_t ssi_base,
                      uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[8];
  if (len > 7)
    len = 7;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  return nrf_async_cmd_start(a, recvbuf, sendbuf, len+1, ssi_base,
                             csn_base, csn_pin);
}


static int32_t
nrf_write_reg_start(struct nrf_async_cmd *a, uint8_t reg, uint8_t val,
                    uint8_t *recvbuf,
                    uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  return nrf_write_reg_n_start(a, reg, &val, recvbuf, 1,
                               ssi_base, csn_base, csn_pin);
}


static int32_t
nrf_read_reg_n_start(struct nrf_async_cmd *a, uint8_t reg, uint8_t *recvbuf,
                     uint32_t len, uint32_t ssi_base,
                     uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[8];
  if (len > 7)
    len = 7;
  sendbuf[0] = nRF_R_REGISTER | reg;
  bzero(&sendbuf[1], len);
  return nrf_async_cmd_start(a, recvbuf, sendbuf, len+1, ssi_base,
                             csn_base, csn_pin);
}


static int32_t
nrf_read_reg_start(struct nrf_async_cmd *a, uint8_t reg, uint8_t *recvbuf,
                   uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  return nrf_read_reg_n_start(a, reg, recvbuf, 1,
                              ssi_base, csn_base, csn_pin);
}


/*
  Continously receive packets on nRF24L01+ in receiver mode.

  As packets are received, they are passed to a callback function. This
  callback is invoked from interrupt context, so it should ideally be fairly
  quick and has to be aware of the general interrupt caveats.

  Operation is purely interrupt-driven, without using uDMA; this complies with
  the Tiva errata that says that SSI0 and SSI1 cannot both use uDMA at the
  same time.
*/
struct nrf_async_receive_multi {
  void (*consumepacket)(uint8_t *, void *);
  void *cb_data;
  uint32_t ssi_base;
  uint32_t csn_base, csn_pin;
  uint32_t ce_base, ce_pin;
  uint32_t irq_base, irq_pin;
  union {
    struct nrf_async_rcv rcv;
    struct nrf_async_cmd cmd;
  } substate;
  uint8_t packet[32];
  uint8_t nrf_rcv_running;
  uint8_t nrf_cmd_running;
  uint8_t recvbuf[2];
  uint8_t state;
};
enum nrf_async_receive_multi_states {
  ST_NRF_ARM_A1,
  ST_NRF_ARM_A2,
  ST_NRF_ARM_A3,
  ST_NRF_ARM_A4
};

static uint32_t
nrf_async_receive_multi_cont(struct nrf_async_receive_multi *a,
                             uint32_t is_ssi_event);
static int32_t
nrf_async_receive_multi_start(struct nrf_async_receive_multi *a,
                              void (*consumepacket)(uint8_t *, void *),
                              void *cb_data, uint32_t ssi_base,
                               uint32_t csn_base, uint32_t csn_pin,
                               uint32_t ce_base, uint32_t ce_pin,
                               uint32_t irq_base, uint32_t irq_pin)
{
  a->consumepacket = consumepacket;
  a->cb_data = cb_data;
  a->ssi_base = ssi_base;
  a->csn_base = csn_base;
  a->csn_pin = csn_pin;
  a->ce_base = ce_base;
  a->ce_pin = ce_pin;
  a->irq_base = irq_base;
  a->irq_pin = irq_pin;
  a->nrf_rcv_running = 0;
  a->nrf_cmd_running = 0;
  a->state = ST_NRF_ARM_A1;

  /* Assert CE to enter receive mode. */
  ce_high(ce_base, ce_pin);

  /*
    Enable interrupt when the IRQ line goes low, which happens when data is
    ready in the Rx FIFO (RX_DR).
  */
  ROM_GPIOPinIntEnable(irq_base, irq_pin);
  return 0;
}


/*
  Called to continue a multi-packet receive session.
  This should be called when an event occurs, either in the form of
  an SSI interrupt or in the form of a GPIO interrupt on the nRF24L01+
  IRQ pin (the is_ssi_event flag tells which).

  The two interrupts should be configured to have the same priority, so that
  one of them does not attempt to pre-empt the other; that would lead to
  nasty races.

  Returns 1 if the nRF24L01+ is now idle (no packets pending in Rx FIFO), 0
  if activity is still on-going.
*/
static uint32_t
nrf_async_receive_multi_cont(struct nrf_async_receive_multi *a,
                             uint32_t is_ssi_event)
{
  if (is_ssi_event && a->nrf_rcv_running)
  {
    if (nrf_async_rcv_cont(&a->substate.rcv))
      a->nrf_rcv_running = 0;
    else
      return 0;
  }
  else if (is_ssi_event && a->nrf_cmd_running)
  {
    if (nrf_async_cmd_cont(&a->substate.cmd))
      a->nrf_cmd_running = 0;
    else
      return 0;
  }

resched:
  switch (a->state)
  {
  case ST_NRF_ARM_A1:
    ROM_GPIOPinIntDisable(a->irq_base, a->irq_pin);
    /* Clear the RX_DS interrupt. */
    a->nrf_cmd_running = 1;
    nrf_write_reg_start(&a->substate.cmd, nRF_STATUS, nRF_RX_DR, a->recvbuf,
                        a->ssi_base, a->csn_base, a->csn_pin);
    a->state = ST_NRF_ARM_A2;
    return 0;

  case ST_NRF_ARM_A2:
    /* Read FIFO status to check if there is any data ready. */
    a->nrf_cmd_running = 1;
    nrf_read_reg_start(&a->substate.cmd, nRF_FIFO_STATUS, a->recvbuf,
                       a->ssi_base, a->csn_base, a->csn_pin);
    a->state = ST_NRF_ARM_A3;
    return 0;

  case ST_NRF_ARM_A3:
    if (a->recvbuf[1] & nRF_RX_EMPTY)
    {
      /*
        No more packets in the Rx fifo. Enable the IRQ interrupt and wait for
        more packets to arrive.
      */
      a->state = ST_NRF_ARM_A1;
      ROM_GPIOPinIntEnable(a->irq_base, a->irq_pin);
      return 1;
    }

    /* The Rx FIFO is non-empty, so read a packet. */
    a->nrf_rcv_running = 1;
    nrf_async_rcv_start(&a->substate.rcv, a->recvbuf, a->packet, 32,
                        a->ssi_base, a->csn_base, a->csn_pin);
    a->state = ST_NRF_ARM_A4;
    return 0;

  case ST_NRF_ARM_A4:
    /* Deliver the received packet to the callback. */
    (*(a->consumepacket))(a->packet, a->cb_data);
    /* Now go check if there are more packets available. */
    a->state = ST_NRF_ARM_A2;
    goto resched;

  default:
    /* This shouldn't really happen ... */
    return 1;
  }
}


/*
  Configure nRF24L01+ as Rx or Tx.
    channel - radio frequency channel to use, 0 <= channel <= 127.
    power - nRF_RF_PWR_<X>DBM, <X> is 0, 6, 12, 18 dBm.
*/
static void
nrf_init_config(uint8_t is_rx, uint32_t channel, uint32_t power,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  static const uint8_t addr[3] = { 0xe7, 0xe7, 0xe7 };

  if (is_rx)
    nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  else
    nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Disable auto-ack, saving 9 bits/packet. Else 0x3f. */
  nrf_write_reg(nRF_EN_AA, 0, ssi_base, csn_base, csn_pin);
  /* Enable only pipe 0. */
  nrf_write_reg(nRF_EN_RXADDR, nRF_ERX_P0, ssi_base, csn_base, csn_pin);
  /* 3 byte adresses. */
  nrf_write_reg(nRF_SETUP_AW, nRF_AW_3BYTES, ssi_base, csn_base, csn_pin);
  /* Disable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, 0, ssi_base, csn_base, csn_pin);
  nrf_write_reg(nRF_RF_CH, channel, ssi_base, csn_base, csn_pin);
  /* Use 2Mbps, and set transmit power. */
  nrf_write_reg(nRF_RF_SETUP, nRF_RF_DR_HIGH | power,
                ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_RX_ADDR_P0, addr, 3, ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_TX_ADDR, addr, 3, ssi_base, csn_base, csn_pin);
  /* Set payload size for pipe 0. */
  if (is_rx)
    nrf_write_reg(nRF_RX_PW_P0, 32, ssi_base, csn_base, csn_pin);
  else
    nrf_write_reg(nRF_RX_PW_P0, 8, ssi_base, csn_base, csn_pin);
  /* Disable pipe 1-5. */
  nrf_write_reg(nRF_RX_PW_P1, 0, ssi_base, csn_base, csn_pin);
  /* Disable dynamic payload length. */
  nrf_write_reg(nRF_DYNDP, 0, ssi_base, csn_base, csn_pin);
  /* Allow disabling acks. */
  nrf_write_reg(nRF_FEATURE, nRF_EN_DYN_ACK, ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


static void
config_nrf_interrupts(void)
{
  ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_LOW_LEVEL);
  ROM_IntMasterEnable();
  ROM_IntEnable(INT_GPIOB);
}


static struct nrf_async_receive_multi receive_multi_state;
static volatile uint8_t receive_multi_running = 0;
static volatile uint8_t nrf_irq_pending = 0;
static volatile uint8_t nrf_last_activity = 0;
static volatile uint8_t nrf_idle = 0;

void
IntHandlerGPIOb(void)
{
  uint32_t irq_status = HWREG(GPIO_PORTB_BASE + GPIO_O_MIS) & 0xff;
  if (irq_status & GPIO_PIN_0)
  {
    /* Rx IRQ. */
    if (receive_multi_running)
    {
      nrf_idle = nrf_async_receive_multi_cont(&receive_multi_state, 0);
    }
    else
    {
      /*
        Clear the interrupt request and disable further interrupts until we can
        clear the request from the device over SPI.
      */
      HWREG(GPIO_PORTB_BASE + GPIO_O_IM) &= ~GPIO_PIN_0 & 0xff;
      HWREG(GPIO_PORTB_BASE + GPIO_O_ICR) = GPIO_PIN_0;
      nrf_irq_pending = 1;
    }
  }
}


void
IntHandlerSSI1(void)
{
  if (receive_multi_running)
    nrf_idle = nrf_async_receive_multi_cont(&receive_multi_state, 1);
}


static inline uint32_t
hall_timer_value(void)
{
#ifdef HALL1
  return HWREG(WTIMER0_BASE + TIMER_O_TBV);
#endif
#ifdef HALL2
  return HWREG(WTIMER5_BASE + TIMER_O_TBV);
#endif
#ifdef HALL3
  return HWREG(WTIMER5_BASE + TIMER_O_TAV);
#endif
}


static void
my_recv_cb(uint8_t *packet, void *data)
{
  nrf_last_activity = hall_timer_value();
  if (packet[0] == 255)
  {
    /* Command packet. */
    if (packet[1] == 1)
      pov_config_accept_packet(packet);
  }
  else if (packet[0] == 254)
  {
    /* Debug packet. */
    if (packet[1] == 255)
      ROM_SysCtlReset();
  }
  else
    accept_packet(packet);
}


static void
start_receive_packets(uint32_t ssi_base, uint32_t csn_base,
                      uint32_t csn_pin, uint32_t ce_base, uint32_t ce_pin)
{
  nrf_idle = 0;
  receive_multi_running = 1;
  nrf_last_activity = hall_timer_value();
  nrf_async_receive_multi_start(&receive_multi_state, my_recv_cb, NULL,
                                ssi_base, csn_base, csn_pin, ce_base, ce_pin,
                                GPIO_PORTB_BASE, GPIO_PIN_0);
}


static volatile uint8_t do_next_frame = 0;
static volatile uint8_t current_tlc_frame_buf = 0;
static uint8_t tlc1_frame_buf[2][TLC_GS_BYTES];
static uint8_t tlc2_frame_buf[2][TLC_GS_BYTES];
static uint8_t tlc3_frame_buf[2][TLC_GS_BYTES];
static volatile float scanline_angle;
static volatile float scanline_width_unity;


void
IntHandlerTimer2B(void)
{
  uint8_t cur;
  static uint32_t anim_counter = 0;
  uint32_t current_time, start_time, current_period;
  uint32_t estim_latch_time, estim_delta;
  uint32_t tmp;
#ifdef FAKE_HALL_SENSOR
  static uint32_t fake_hall_counter = 0;
#else
  uint32_t since_hall;
#endif

  if (!(HWREG(TIMER2_BASE + TIMER_O_MIS) & TIMER_TIMB_TIMEOUT))
  {
    /*
      For some reason, I get occasional spurious interrupts on the Timer 2B
      interrupt vector. They seem to be related to SSI3 DMA completion.

      The interrupt routine is entered with the "masked interrupt status"
      being zero, suggesting that no interrupt was pending ...
    */
    return;
  }
  /* Clear the timer2b timeout interrupt. */
  HWREG(TIMER2_BASE + TIMER_O_ICR) =  TIMER_TIMB_TIMEOUT;

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
  current_time = hall_timer_value();
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

  ++do_next_frame;

  /*
    Estimate the angle of rotation when the scanline we are about to generate
    is latched.

    This timer interrupt we are writing the data into the buffer, the next
    interrupt we will shift it out, so latch will be in two timer periods.
  */
  estim_latch_time = current_time - 2*PWM_CYCLES;

#ifndef FAKE_HALL_SENSOR
  /*
    Try to adjust for the rotation period not being an integer number of PWM
    periods.

    When the hall sensor triggers, we will increase the length of the
    following PWM period, so that the latching of data starts at an integer
    multiplum of the PWM period relative to the hall trigger. If the rotation
    period (and hall trigger) is 100% stable, this will ensure that pixels
    always start at the exact same point.

    Since we generate scanlines two interrupts ahead of the latch, and shift
    them out over SPI one interrupt ahead, when we detect the hall signal we
    actually increase the length of the interrupt only two periods later.
  */
  since_hall = start_time-current_time;
  if (since_hall < PWM_CYCLES)
    estim_latch_time += PWM_CYCLES - since_hall;
  else if (since_hall < 2*PWM_CYCLES)
  {
    estim_latch_time += 2*PWM_CYCLES - since_hall;
    HWREG(TIMER2_BASE + TIMER_O_TBILR) = 3*PWM_CYCLES - since_hall;
  }
  else
    HWREG(TIMER2_BASE + TIMER_O_TBILR) = PWM_CYCLES;
#endif

  estim_delta = start_time - estim_latch_time;
  if (current_period < 1)
    current_period = 1;
  if (estim_delta > current_period)
    estim_delta = current_period;
  scanline_angle =
    (float)(2.0f*F_PI) * (float)estim_delta / (float)current_period;
  scanline_width_unity = (float)PWM_CYCLES / (float)current_period;
  /*
    Trigger a software-interrupt to perform the actual scanline generation.
    (This is time-consuming, so we want it to be pre-emptable).
  */
  HWREG(NVIC_SW_TRIG) = INT_UDMA - 16;

  ++anim_counter;

  /* Restart HALL capture a little while after last event. */
  tmp = hall_capture_delay;
  if (tmp)
    hall_capture_delay = tmp-1;
  else
  {
#ifdef HALL1
    ROM_TimerIntClear(WTIMER0_BASE, TIMER_CAPB_EVENT);
    ROM_IntEnable(INT_WTIMER0B);
#endif
#ifdef HALL2
    ROM_TimerIntClear(WTIMER5_BASE, TIMER_CAPB_EVENT);
    ROM_IntEnable(INT_WTIMER5B);
#endif
#ifdef HALL3
    ROM_TimerIntClear(WTIMER5_BASE, TIMER_CAPA_EVENT);
    ROM_IntEnable(INT_WTIMER5A);
#endif
  }
#ifdef FAKE_HALL_SENSOR
  if (++fake_hall_counter > 5000)
  {
    fake_hall_counter = 0;
    record_hall_sensor(current_time);
  }
#endif
}


static volatile uint32_t scanline_time, max_scanline_time;

void
IntHandlerDMA(void)
{
  uint32_t t_start, t_stop, t;
  uint8_t cur;
  static const float ang_adj = F_PI*2.0f*(-29.0f)/360.0f;
  float angle = scanline_angle + ang_adj;
  float angle_width = scanline_width_unity;

  /*
    This is not really related to DMA.

    We just use this as an available interrupt to do software-triggered,
    low-priority interrupt so that we can be pre-empted while handling the
    scanline conversion (which is time-consuming).
  */
  cur = current_tlc_frame_buf;
  t_start = hall_timer_value();
  bm_scanline(angle, angle_width, 32, tlc1_frame_buf[cur]);
  bm_scanline(angle+(121.3f/180.0f*F_PI), angle_width, 32, tlc2_frame_buf[cur]);
  bm_scanline(angle+(2.0f*120.7051f/180.0f*F_PI), angle_width, 32, tlc3_frame_buf[cur]);
  t_stop = hall_timer_value();
  t = t_start - t_stop;
  scanline_time = t;
  if (t > max_scanline_time)
    max_scanline_time = t;
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


int main()
{
  uint32_t last_frame_time = 0;
  uint32_t last_time;
  int res;

  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  ROM_FPULazyStackingEnable();

  generate_test_image();

  /* Setup GPIO to read HALL sensor. */
  setup_hall_gpio_n_gsclk1();
  setup_hall23();

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 500000,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));

  /* Read the config from eeprom. */
  serial_output_str("Serial configured, now reading config from EEPROM...\r\n");
  res = pov_config_read();
  if (res)
  {
    serial_output_str("Error from read config: ");
    println_uint32(res);
  }
  serial_output_str("bm_mode="); println_uint32(bm_mode);
  serial_output_str("dc_value="); println_uint32(dc_value);

  /* Set interrupts to use no sub-priorities. */
  ROM_IntPriorityGroupingSet(7);
  /*
    Setup the priorities of the interrupts we use.

    The interrupts associated with the latching to TLCs and the Hall sensor(s)
    have the highest priorities.

    The interrupts for nRF24L01+ wireless reception have medium prio (the
    nRF24L01+ has a three-level fifo, so latency is not critical).

    We use uDMA software request interrupt as a low-priority software-triggered
    interrupt so that a high-prio interrupt can yield to a lower-prio one
    to allow preemption (used for scanline conversion which consumes a lot of
    CPU).
  */
  ROM_IntPrioritySet(INT_WTIMER0B, 0 << 5);
  ROM_IntPrioritySet(INT_WTIMER5A, 0 << 5);
  ROM_IntPrioritySet(INT_WTIMER5B, 0 << 5);
  ROM_IntPrioritySet(INT_TIMER2B, 0 << 5);
  ROM_IntPrioritySet(INT_SSI0, 0 << 5);
  ROM_IntPrioritySet(INT_SSI2, 0 << 5);
  ROM_IntPrioritySet(INT_SSI3, 0 << 5);
  ROM_IntPrioritySet(INT_GPIOB, 4 << 5);
  ROM_IntPrioritySet(INT_SSI1, 4 << 5);
  ROM_IntPrioritySet(INT_UDMA, 7 << 5);
  ROM_IntEnable(INT_UDMA);

  config_tlc_gpio();
  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  ROM_SysCtlDelay(MCU_HZ/3/10);
  serial_output_str("Loading TLC 1...\r\n");
  init_tlc_dc(SSI_TLC1_BASE, dc_value,
              GPIO_MODE1_BASE, GPIO_MODE1_PIN,
              GPIO_XLAT1_BASE, GPIO_XLAT1_PIN);
#ifndef SINGLE_BLADE
  serial_output_str("Loading TLC 2...\r\n");
  init_tlc_dc(SSI_TLC2_BASE, dc_value,
              GPIO_MODE2_BASE, GPIO_MODE2_PIN,
              GPIO_XLAT2_BASE, GPIO_XLAT2_PIN);
  serial_output_str("Loading TLC 3...\r\n");
  init_tlc_dc(SSI_TLC3_BASE, dc_value,
              GPIO_MODE3_BASE, GPIO_MODE3_PIN,
              GPIO_XLAT3_BASE, GPIO_XLAT3_PIN);
#endif
  serial_output_str("TLC load ok!\r\n");

  config_spi_tlc_write(SSI_TLC1_BASE, 8);
  config_spi_tlc_write(SSI_TLC2_BASE, 8);
  config_spi_tlc_write(SSI_TLC3_BASE, 8);
  init_udma_for_tlc();

  /* Setup for receiving data on nRF24L01+. */
  config_nrf_ssi_gpio();
  config_nrf_interrupts();
  /* Wait another 20 millisec (nRF24L01+ datasheet says min 4.5 millisec). */
  ROM_SysCtlDelay(MCU_HZ/3/1000*20);
#ifndef DISABLE_NRF
  nrf_init_config(1 /* Rx */, 2, nRF_RF_PWR_0DBM,
                  SSI1_BASE, GPIO_PORTF_BASE, GPIO_PIN_3);
  /* Set Rx in receive mode. */
  ce_high(GPIO_PORTB_BASE, GPIO_PIN_3);

  start_receive_packets(SSI1_BASE, GPIO_PORTF_BASE, GPIO_PIN_3,
                        GPIO_PORTB_BASE, GPIO_PIN_3);
#endif

  /*
    Once we start the timer 1A, we will get interrupts that sends data
    to the TLCs and latch it. So we must do this after everything else
    is set up correctly.
  */
  setup_pwm_GSCLK3();
  setup_pwm_GSCLK2_n_timer();

  serial_output_str("Starting main loop...\r\n");
  last_time = hall_timer_value();
  for (;;) {
    uint32_t frame_start;
    char text_buf[40];
    char *p;
    uint32_t now_time;

    p = text_buf;
    *p++ = 's';
    *p++ = 'l';
    *p++ = '=';
    p = uint32_tostring(p, scanline_time);
    *p++ = '(';
    p = uint32_tostring(p, max_scanline_time);
    *p++ = ')';
    *p++ = ' ';
    *p++ = 'f';
    *p++ = 'r';
    *p++ = '=';
    p = uint32_tostring(p, last_frame_time);

    frame_start = hall_timer_value();
    gen_anim_frame(0 /* text_buf */);
    last_frame_time = frame_start - hall_timer_value();

    /* Constrain the frame rate. */
    do {
      now_time = hall_timer_value();
    } while (last_time - now_time < MCU_HZ/50);
    next_anim_frame();
    last_time = now_time;
  }
}
