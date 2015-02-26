#include <inttypes.h>
#include <string.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"

#include "pov.h"
#include "sdcard.h"
#include "serial_dbg.h"
#include "ev_fat.h"
#include "pov_config.h"


/*
  Sd-card pinout:

    CS    PB1  (card select)
    DI    PF1  (data into sd-card, eg. MOSI)
    DO    PF0  (data out of sd-card, eg. MISO)
    SCLK  PF2  (SPI clock)
    CD    PB2  (card detect)
*/


enum enum_sd_type { SD_TYPE_SD1, SD_TYPE_SD2, SD_TYPE_SDHC };


static void
config_spi_lowspeed(uint32_t base)
{
  /*
    Configure the SPI for correct mode to initialise the SD card.

    We need CLK inactive low, so SPO=0.
    We need to setup and sample on the leading, rising CLK edge, so SPH=0.

    We start at a 400 kHz for the initial config, then we increase the
    speed later.
  */

  ROM_SSIDisable(base);
  ROM_SSIConfigSetExpClk(base, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 400000, 8);
  ROM_SSIEnable(base);
}


static void
config_spi_highspeed(uint32_t base)
{
  /*
    Configure the SPI for correct mode to read from SD card.

    We need CLK inactive low, so SPO=0.
    We need to setup and sample on the leading, rising CLK edge, so SPH=0.

    We use 10 MHz for fast reading after initial configuration is successful.
  */

  ROM_SSIDisable(base);
  ROM_SSIConfigSetExpClk(base, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 20000000, 8);
  ROM_SSIEnable(base);
}


void
sd_config_spi(void)
{
  config_spi_highspeed(SD_SSI_BASE);
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
sd_cs_low(void)
{
  csn_low(SD_CS_BASE, SD_CS_PIN);
}


static inline void
sd_cs_high(void)
{
  csn_high(SD_CS_BASE, SD_CS_PIN);
}


void
sd_setup_systick(void)
{
  ROM_SysTickPeriodSet(0xffffff+1);
  /* Force reload. */
  HWREG(NVIC_ST_CURRENT) = 0;
  ROM_SysTickEnable();
}


static inline uint32_t
get_time(void)
{
  return HWREG(NVIC_ST_CURRENT);
}


static inline uint32_t
calc_time_from_val(uint32_t start, uint32_t stop)
{
  return (start - stop) & 0xffffff;
}


static inline uint32_t
calc_time(uint32_t start)
{
  uint32_t stop = HWREG(NVIC_ST_CURRENT);
  return calc_time_from_val(start, stop);
}


/*
  Delay until specified amount of systicks have passed.

  As systick is a 24-bit counter, the amount cannot exceed 0xffffff, or a bit
  more than 16000000.
*/
static void
delay_systicks(uint32_t cycles)
{
  uint32_t last = 0;
  uint32_t start = get_time();

  for (;;)
  {
    uint32_t elapsed = calc_time(start);
    if (elapsed >= cycles)
      return;
    /*
      The `elapsed` value wraps around at 0xffffff. If we are waiting for some
      value close to this, then it might wrap around in-between testing it at
      just under the limit and the next test. In this case, we also need to
      terminate the wait, or we would wait another full period or even end up
      hanging indefinitely.
    */
    if (elapsed < last)
      return;
    last = elapsed;
  }
}


static void
delay_us(uint32_t us)
{
  /* This assumes that MCU_HZ is divisible by 1000000. */
  uint32_t cycles = (MCU_HZ/1000000)*us;
#if (MCU_HZ % 1000000)
#error delay_us() computes delay incorrectly if MCU_HZ is not a multiple of 1000000
#endif

  while (cycles > 0xffffff)
  {
    delay_systicks(0xffffff);
    cycles -= 0xffffff;
  }
  delay_systicks(cycles);
}


static inline uint32_t
sd_ssi_rx_not_empty(void)
{
  return HWREG(SD_SSI_BASE + SSI_O_SR) & SSI_SR_RNE;
}


static struct {
  uint32_t sofar;
  uint8_t *buf;
  uint32_t main_length;
  uint32_t read_length;
  uint8_t got_error;
  uint8_t done;
} sd_state;


static void noop_handler() { }
void (*sd_ssi1_handler)(void) = noop_handler;
static void sd_data_read_wait_cmd17_response(void);
static void sd_data_read_wait_data_token(void);
static void sd_data_read_data_main(void);
static void sd_data_read_data_remaining(void);

static void
sd_error()
{
  /* Disable the interrupt. */
  HWREG(SD_SSI_BASE + SSI_O_IM) &= ~(uint32_t)SSI_TXFF;
  barrier();
  sd_ssi1_handler = noop_handler;
  /* Wait for Tx FIFO to drain. */
  HWREG(SD_SSI_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
  while (HWREG(SD_SSI_BASE + SSI_O_RIS) & SSI_RIS_TXRIS)
    ;
  /* Remove any bytes left-over in the Rx FIFO. */
  while (HWREG(SD_SSI_BASE + SSI_O_SR) & SSI_SR_RNE)
    (void)(HWREG(SD_SSI_BASE + SSI_O_DR));

  serial_output_str("Error during read from SD card.\r\n");
  sd_state.got_error= 1;
  sd_state.done = 1;
  barrier();
}


__attribute((noinline))
static void
sd_start_read_block(uint32_t sector, uint8_t *dst, uint32_t len, int card_type)
{
  uint32_t i;
  uint32_t arg;

  sd_cs_low();
  sd_state.sofar = 0;
  sd_state.read_length = len;
  if (len <= (512+2-8))
    sd_state.main_length = len;
  else
    sd_state.main_length = (512+2-8);
  sd_state.buf = dst;
  sd_state.got_error = 0;
  sd_state.done = 0;
  barrier();
  for (i = 0; i < 1500; ++i)
  {
    uint32_t data;
    ROM_SSIDataPut(SD_SSI_BASE, 0xff);
    ROM_SSIDataGet(SD_SSI_BASE, &data);
    if ((data & 0xff) == (uint32_t)0xff)
      break;
  }
  if (i == 1500)
  {
    serial_output_str("Error due to timeout in wait for idle\r\n");
    sd_error();
    return;
  }

  /* Empty any left-over stuff in the RX FIFO. */
  while (HWREG(SD_SSI_BASE + SSI_O_SR) & SSI_SR_BSY)
    ;
  while (HWREG(SD_SSI_BASE + SSI_O_SR) & SSI_SR_RNE)
    (void)(HWREG(SD_SSI_BASE + SSI_O_DR));
  /* Fill in the command CMD17 to the Tx FIFO. */
  HWREG(SD_SSI_BASE + SSI_O_DR) = 17 | 0x40;
  arg = (card_type == SD_TYPE_SDHC ? sector : sector << 9);
  HWREG(SD_SSI_BASE + SSI_O_DR) = arg >> 24;
  HWREG(SD_SSI_BASE + SSI_O_DR) = (arg >> 16) & 0xff;
  HWREG(SD_SSI_BASE + SSI_O_DR) = (arg >> 8) & 0xff;
  HWREG(SD_SSI_BASE + SSI_O_DR) = arg & 0xff;
  HWREG(SD_SSI_BASE + SSI_O_DR) = 0x87;         /* Dummy CRC */
  HWREG(SD_SSI_BASE + SSI_O_DR) = 0xff;
  HWREG(SD_SSI_BASE + SSI_O_DR) = 0xff;

  /* Enable interrupt when Tx FIFO is 1/2 full or less. */
  sd_ssi1_handler = sd_data_read_wait_cmd17_response;
  barrier();
  HWREG(SD_SSI_BASE + SSI_O_CR1) &= ~SSI_CR1_EOT;
  HWREG(SD_SSI_BASE + SSI_O_IM) |= SSI_TXFF;
}


__attribute((noinline))
static void
sd_cmd17_error(uint8_t data)
{
  serial_output_str("Error from cmd17: ");
  println_uint32(data);
  sd_error();
}


__attribute__((noinline))
static void
sd_data_read_wait_cmd17_response(void)
{
  while (sd_ssi_rx_not_empty())
  {
    uint8_t data = HWREG(SD_SSI_BASE + SSI_O_DR) & 0xff;
    HWREG(SD_SSI_BASE + SSI_O_DR) = 0xff;
    if (likely(data & 0x80))
    {
      /* ToDo: Need some kind of timeout here? */
      continue;
    }
    if (unlikely(data != 0))
    {
      sd_cmd17_error(data);
      return;
    }

    sd_ssi1_handler = sd_data_read_wait_data_token;
    sd_data_read_wait_data_token();
    return;
  }
}


__attribute__((noinline))
static void
sd_data_read_wait_data_token(void)
{
  while (sd_ssi_rx_not_empty())
  {
    uint8_t data = HWREG(SD_SSI_BASE + SSI_O_DR) & 0xff;
    HWREG(SD_SSI_BASE + SSI_O_DR) = 0xff;
    if (likely(data == 0xff))
    {
      /* ToDo: Need some kind of timeout here? */
      continue;
    }
    if (unlikely(data != 0xfe))
    {
      sd_error();
      return;
    }

    sd_ssi1_handler = sd_data_read_data_main;
    sd_data_read_data_main();
    return;
  }
}


/*
  This callback reads the main bulk of data, when we know that we still need
  to transmit continue transmitting 0xff driver bytes and did not yet exceed
  the buffer length.
*/
__attribute__((noinline))
static void
sd_data_read_data_main(void)
{
  uint32_t sofar = sd_state.sofar;
  uint32_t main_length = sd_state.main_length;
  uint8_t *buf = sd_state.buf;

  while (sd_ssi_rx_not_empty())
  {
    uint8_t data = HWREG(SD_SSI_BASE + SSI_O_DR) & 0xff;
    buf[sofar++] = data;
    HWREG(SD_SSI_BASE + SSI_O_DR) = 0xff;
    if (unlikely(sofar == main_length))
    {
      sd_state.sofar = sofar;
      sd_ssi1_handler = sd_data_read_data_remaining;
      sd_data_read_data_remaining();
      return;
    }
  }
  sd_state.sofar = sofar;
}


/*
  Read any remaining bytes, where we might have to stop sending driver
  bytes and avoid filling in the buffer beyond the length.
*/
__attribute__((noinline))
static void
sd_data_read_data_remaining(void)
{
  uint32_t sofar = sd_state.sofar;
  uint32_t length = sd_state.read_length;
  uint8_t *buf = sd_state.buf;

  while (sd_ssi_rx_not_empty())
  {
    uint8_t data = HWREG(SD_SSI_BASE + SSI_O_DR) & 0xff;
    if (sofar < length)
      buf[sofar] = data;
    ++sofar;
    /*
      We need to drive 512 bytes of data plus 2 bytes of CRC. But we already
      clocked out 8 driver bytes before we started to count in sofar.
    */
    if (sofar < (512+2-8))
      HWREG(SD_SSI_BASE + SSI_O_DR) = 0xff;
    else if (sofar == (512+2-8))
    {
      /*
        We have queued sufficient driver bytes.
        So switch interrupt to only trigger at end of transmission.
      */
      HWREG(SD_SSI_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
    }
    else if (sofar == (512+2))
    {
      HWREG(SD_SSI_BASE + SSI_O_IM) &= ~(uint32_t)SSI_TXFF;
      barrier();
      sd_ssi1_handler = noop_handler;
      sd_state.done = 1;
      return;
    }
  }
  sd_state.sofar = sofar;
}


static inline uint32_t
is_sd_read_done(void)
{
  barrier();
  return sd_state.done;
}


static uint8_t
sd_cmd(uint8_t cmd, uint32_t arg)
{
  uint8_t cmd_buf[6];
  uint32_t i;
  uint32_t data;

  cmd_buf[0] = cmd | 0x40;
  cmd_buf[1] = (arg >> 24) & 0xff;
  cmd_buf[2] = (arg >> 16) & 0xff;
  cmd_buf[3] = (arg >> 8) & 0xff;
  cmd_buf[4] = arg & 0xff;
  cmd_buf[5] = cmd ? 0x87 : 0x95;

  for (i = 0; i < 15000; ++i)
  {
    ROM_SSIDataPut(SD_SSI_BASE, 0xff);
    ROM_SSIDataGet(SD_SSI_BASE, &data);
    if ((data & 0xff) == (uint32_t)0xff)
      break;
  }

  for (i = 0; i < 6; ++i)
  {
    ROM_SSIDataPut(SD_SSI_BASE, cmd_buf[i]);
    ROM_SSIDataGet(SD_SSI_BASE, &data);
  }

  do
  {
    ROM_SSIDataPut(SD_SSI_BASE, 0xff);
    ROM_SSIDataGet(SD_SSI_BASE, &data);
    /* ToDo: Handle timeout */
  }
  while (data & 0x80);
  /* Take CSN high to complete transfer. */
  return data;
}


static uint8_t
sd_acmd(uint8_t cmd, uint32_t arg)
{
  sd_cmd(55, 0);
  return sd_cmd(cmd, arg);
}


static void
sd_power_on(void)
{
  uint32_t i;
  unsigned long dummy_data;

  /*
    At power-on, first wait >= 1 ms, then send >= 74 SCLK pulses at between
    100 and 400 kHz with CS held high.

    We use 80 pulses (10 bytes) @ 400 kHz.

    And some extra delay for safety...
  */
  serial_output_str("SD-card power-on...\r\n");
  config_spi_lowspeed(SD_SSI_BASE);
  sd_cs_high();
  delay_us(50*1000);

  /*
    Temporarily switch the DI line to manual GPIO, so we can hold it high
    while inputting >= 74 SPI clocks to the SD card.
  */
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

  for (i = 0; i < 10; ++i)
  {
    ROM_SSIDataPut(SD_SSI_BASE, 0xff);
    ROM_SSIDataGet(SD_SSI_BASE, &dummy_data);
  }

  /* Switch the DI line back to SPI controlled. */
  ROM_GPIOPinConfigure(GPIO_PF1_SSI1TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_1);

  delay_us(50*100);
}


static uint8_t sd_buffer[512];
static int card_type;

static void
sd_card_init(void)
{
  uint8_t cmd_res;
  uint32_t i;

  sd_power_on();

  /*
    Now send a command 0, GO_IDLE_STATE.
    For some reason, just after power-up, the first CMD(0,0) fails, the card
    appears not ready and doesn't output anything on its DO pin.
    It works if we re-try the CMD(0,0), so for now let's leave it as this...
  */
  for (;;)
  {
    serial_output_str("Sending cmd(0,0)... ");
    sd_cs_low();
    cmd_res = sd_cmd(0, 0);
    if (cmd_res == 0x01)
      break;
    sd_cs_high();
  }
  serial_output_str("Done\r\n");

  serial_output_str("Send cmd(8, 0x1aa)\r\n");
  cmd_res = sd_cmd(8, 0x1aa);
  if (cmd_res & 0x04)
  {
    /* Error means SD 1 type. */
    card_type = SD_TYPE_SD1;
    serial_output_str("Card type SD1\r\n");
  }
  else
  {
    uint8_t buf[4];

    card_type = SD_TYPE_SD2;
    serial_output_str("Card type SD2\r\n");
    /* Receive 4 more bytes. */
    for (i = 0; i < 4; ++i)
    {
      uint32_t data;
      ROM_SSIDataPut(SD_SSI_BASE, 0xff);
      ROM_SSIDataGet(SD_SSI_BASE, &data);
      buf[i] = data;
    }
    if (buf[3] != 0xaa)
    {
      serial_output_str("Error, unexpected response from CMD8:");
      serial_output_hexbyte(buf[0]);
      serial_output_hexbyte(buf[1]);
      serial_output_hexbyte(buf[2]);
      serial_output_hexbyte(buf[3]);
      serial_output_str("\n");
      return;
    }
  }

  serial_output_str("Send acmd41's until ready\r\n");
  do
  {
    cmd_res = sd_acmd(41, (card_type == SD_TYPE_SD2 ? 0x40000000 : 0));
  } while (cmd_res != 0);

  if (card_type != SD_TYPE_SD1)
  {
    uint8_t buf[4];

    /* Send command 58 (READ_OCR) to check if SDHC card. */
    cmd_res = sd_cmd(58, 0);
    if (cmd_res)
    {
      serial_output_str("Error from CMD58\r\n");
      return;
    }
    /* Receive 4 more bytes. */
    for (i = 0; i < 4; ++i)
    {
      uint32_t data;
      ROM_SSIDataPut(SD_SSI_BASE, 0xff);
      ROM_SSIDataGet(SD_SSI_BASE, &data);
      buf[i] = data;
    }
    if ((buf[0] & 0xc0) == 0xc0)
      card_type = SD_TYPE_SDHC;
    serial_output_str("Card type SDHC\r\n");
  }

  sd_cs_high();
  delay_us(200000);
  config_spi_highspeed(SD_SSI_BASE);
  delay_us(200000);
  sd_cs_low();
}


static struct ev_file_status fs_stat;
static uint32_t file_sofar;
static uint32_t file_length;
static uint8_t file_open = 0;

static uint8_t
sd_read_block(uint32_t sector, uint8_t *dst)
{
  sd_start_read_block(sector, sd_buffer, 512, card_type);
  while (!is_sd_read_done())
    ;
  return sd_state.got_error;
}

static int
sd_open_file(void)
{
  uint32_t i;
  int res;

  sd_card_init();
  delay_us(50);

  fs_stat.state = 0;
  do
  {
    res = ev_file_get_first_block((bm_mode ? "POVFAN-T.000" : "POVFAN-R.000"),
                                  &fs_stat);
    if (res == EV_FILE_ST_STREAM_BYTES)
    {
      uint8_t err = sd_read_block(fs_stat.st_stream_bytes.sec, sd_buffer);
      if (err)
      {
        serial_output_str("Error reading block\r\n");
        res = EV_FILE_ST_EIOERR;
        break;
      }
      for (i = 0; i < fs_stat.st_stream_bytes.len; ++i)
      {
        if (ev_file_stream_bytes(sd_buffer[fs_stat.st_stream_bytes.offset + i],
                                 &fs_stat))
          break;
      }
    }
  } while (res != EV_FILE_ST_DONE && res >= 0);
  if (res < 0)
  {
    serial_output_str("Error from ev_fat: -");
    println_uint32((uint32_t)0 - (uint32_t)res);
  }
  return res;
}


/*
  Read one frame of animation from the SD-card.

  ToDo: Error handling could do with some improvements.
*/
void
sd_load_frame(uint8_t *frame, uint32_t size)
{
  int res;
  uint32_t i;
  uint32_t frame_sofar;

  if (!file_open)
  {
    res = sd_open_file();
    if (res < 0)
    {
      serial_output_str("Error from ev_fat: -");
      println_uint32((uint32_t)0 - (uint32_t)res);
      goto err;
    }
    file_open = 1;
    file_length = fs_stat.st_get_block_done.length;
    file_sofar = 0;
  }

  frame_sofar = 0;

  while (frame_sofar < size && file_sofar < file_length)
  {
    uint32_t sector = fs_stat.st_get_block_done.sector;
    uint8_t err;
    uint32_t chunk_len;

    err = sd_read_block(sector, sd_buffer);
    if (err)
    {
      serial_output_str("Data block read error: ");
      println_uint32(err);
      goto err;
    }

    /* Check if we have anything left from a previously partially read block. */
    if (file_sofar % 512)
    {
      chunk_len = 512 - (file_sofar % 512);
      if (chunk_len > size)
        chunk_len = size;
      memcpy(frame, sd_buffer + (file_sofar % 512), chunk_len);
    }
    else
    {
      chunk_len = size - frame_sofar;
      if (chunk_len > 512)
        chunk_len = 512;
      if (chunk_len + file_sofar > file_length)
        chunk_len = file_length - file_sofar;
      memcpy(frame + frame_sofar, sd_buffer, chunk_len);
    }
    frame_sofar += chunk_len;
    file_sofar += chunk_len;
    if (file_sofar >= file_length)
    {
      /* End-of-file reached. */
      file_open = 0;
      break;
    }
    if (file_sofar % 512)
    {
      /*
        This frame ended in the middle of a sector. So we need to continue the
        next frame from that point on.
      */
      break;
    }

    /* Move to next sector. */
    do
    {
      res = ev_file_get_next_block(&fs_stat);
      if (res == EV_FILE_ST_STREAM_BYTES)
      {
        uint8_t err = sd_read_block(fs_stat.st_stream_bytes.sec, sd_buffer);
        if (err)
        {
          serial_output_str("Error reading next block\r\n");
          goto err;
        }
        for (i = 0; i < fs_stat.st_stream_bytes.len; ++i)
        {
          if (ev_file_stream_bytes(sd_buffer[fs_stat.st_stream_bytes.offset + i],
                                   &fs_stat))
            break;
        }
      }
    } while (res != EV_FILE_ST_DONE && res >= 0);
    if (res < 0)
    {
      serial_output_str("Error from ev_fat (next block): -");
      println_uint32((uint32_t)0 - (uint32_t)res);
      file_open = 0;
      goto err;
    }
  }

  goto end;
err:
  file_open = 0;
end:
  sd_cs_high();
}


uint32_t
sd_card_present(void)
{
  return !ROM_GPIOPinRead(SD_CD_BASE, SD_CD_PIN);
}
