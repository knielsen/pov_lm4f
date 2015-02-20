#include <inttypes.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"


static void
sd_card_deselect(void)
{
  /* Set CS high (card not selected). */
  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
}


static void
sd_card_select(void)
{
  /* Set CS low (card selected). */
  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
}


void
sd_config_peripheral(void)
{
  /* Setup GPIO pins for SSI3. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_GPIOPinConfigure(GPIO_PD0_SSI3CLK);
  /* ROM_GPIOPinConfigure(GPIO_PD1_SSI3FSS); */
  ROM_GPIOPinConfigure(GPIO_PD2_SSI3RX);
  ROM_GPIOPinConfigure(GPIO_PD3_SSI3TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);

  /* Setup PD1 for CS. */
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
  sd_card_deselect();

}


static void
sd_empty_fifo(void)
{
  uint32_t dummy;
  while(ROM_SSIDataGetNonBlocking(SSI3_BASE, &dummy))
    ;
}


static void
sd_config_lowspeed(void)
{
  /* Configure us as the master. */
  ROM_SSIDisable(SSI3_BASE);
  ROM_SSIConfigSetExpClk(SSI3_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_1,
                     SSI_MODE_MASTER, 256000, 8);
  ROM_SSIEnable(SSI3_BASE);

  sd_empty_fifo();
}


static void
sd_config_highspeed(void)
{
  /* Configure us as the master. */
  ROM_SSIDisable(SSI3_BASE);
  ROM_SSIConfigSetExpClk(SSI3_BASE, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_1,
                     SSI_MODE_MASTER, 4000000, 8);
  ROM_SSIEnable(SSI3_BASE);

  sd_empty_fifo();
}


static void
sd_send_spi(uint8_t byte)
{
  ROM_SSIDataPut(SSI3_BASE, byte);
}


static void
sd_get_spi(void)
{
  uint32_t byte;
  ROM_SSIDataGet(SSI3_BASE, &byte);
  return byte & 0xff;
}


static void
wait_for_idle(void)
{
  do
  {
    sd_send_spi(0xff);
  } while (sd_get_spi() != 0xff);
}


static void
sd_send_10_ff(void)
{
  uint32_t i;
  for (i = 0; i < 10; ++i)
  {
    sd_send_spi(0xff);
    (void)sd_get_spi();
  }
}


static uint32_t
sd_send_cmd(uint32_t cmd, uint32_t arg)
{
  uint32_t res;

  sd_card_select();
  wait_for_idle();
  sd_send_spi(cmd | 0x40);
  (void)sd_get_spi();
  sd_send_spi(arg >> 24);
  (void)sd_get_spi();
  sd_send_spi((arg >> 16) & 0xff);
  (void)sd_get_spi();
  sd_send_spi((arg >> 8) & 0xff);
  (void)sd_get_spi();
  sd_send_spi(arg & 0xff);
  (void)sd_get_spi();
  sd_send_spi(cmd ? 0x87 : 0x95);
  res = sd_get_spi();
  while (res & 0x80)
    sd_send_spi(0xff);
  return res;
}


void
sd_init_card(void)
{
  uint32_t res;

  sd_config_lowspeed();
  sd_send_10_ff();

  /* Send command 0, GO_IDLE_STATE, until "ok" response. */
  do
  {
    res = sd_send_cmd(0, 0);
    sd_card_deselect();
  } while (res != 0x01);

  res = sd_send_cmd(8, 0x1aa);
  if (res & 0x04)
  {
    /* Error means SD 1 type. */
    sd_card_deselect();
    sd_type = SD_TYPE_SD1;
  }
  else
  {
    sd_type = SD_TYPE_SD2
}
