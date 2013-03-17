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


#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3

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

    ROM_UARTCharPut(UART0_BASE, '!');
  }
}
