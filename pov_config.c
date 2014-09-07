/*
  Code for handle reading and writing of config to the eeprom.

  Generally, we can simplify updating the config, by simply writing the new
  config to eeprom and then executing a reset. This avoids any need need for
  possibly complex handling of migrating from one state to the other.

  It is fine to handle "online" config change also, of course.
*/

#include <inttypes.h>
#include <string.h>

#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/eeprom.h"

#include "pov_config.h"
#include "gfx.h"


/*
  Config layout in EEprom:

  offset  size  default  data
       0     2   0x8fea  Magic #1
       2     1     0xff  Sequence number (smaller is newer, wraps around)
       3     1     0xc4  bit 0-5: DC value (0-63)
                         bit 6: GFX mode BM_MODE_RECT12=0 / BM_MODE_TRI12=1
                         bit 7: spare (set to 1)
       4     8   0xff..  spare
      12     2        0  Current filename index (eg. POVFAN-R.XXX, POVFAN-T.YYY)
      14     2   0xd591  Magic #2


  Every write of config writes a new entry, in the slot after the most recently
  written one. And on read, we scan the whole eeprom looking for the newest
  entry, and read that one. Sequence number starts at 0xff and counts down.

  This minimises amount of writes to a single cell, thus maximising eeprom
  lifetime. Though it is probably a bit silly...
*/

#define MAGIC1 ((uint32_t)0x8fea)
#define MAGIC2 ((uint32_t)0xd591)

uint8_t bm_mode;
uint8_t dc_value;
uint16_t file_idx;


static int
is_pov_config(uint32_t *data)
{
  return (data[0] & 0xffff) == MAGIC1 && (data[3] >> 16) == MAGIC2;
}


static uint32_t
cfg_seq_no(uint32_t *data)
{
  return (data[0] >> 16) & 0xff;
}


/*
  Compare 8-bit values, taking wraparound into account.

  This means that 0x10 is considered smaller than 0x20, but larger than 0xf0.

  Returns -1, 0, or 1 corresponding to a < b, a == b, or a > b.
*/
static int wrap_cmp_8(uint8_t a, uint8_t b)
{
  uint8_t d = a - b;
  if (d == 0)
    return 0;
  else if (d >= 0x80)
    return -1;
  else
    return 1;
}


int
pov_config_write(void)
{
  uint32_t data[4];
  uint32_t best_seq_no;
  uint32_t i, best_idx, new_idx;
  uint32_t eeprom_size;
  uint8_t seq_no;
  uint32_t res;

  /*
    Let's find the most recent entry. We will write our entry in the following
    slot, with the next sequence number (one lower, wrapping around).
  */
  best_seq_no = 0x100;
  eeprom_size = ROM_EEPROMSizeGet();

  for (i = 0; i < eeprom_size; i += 16)
  {
    ROM_EEPROMRead(data, i, 16);
    if (!is_pov_config(data))
      continue;
    seq_no = cfg_seq_no(data);
    if (best_seq_no > 0xff || wrap_cmp_8(seq_no, best_seq_no) < 0)
    {
      best_seq_no = seq_no;
      best_idx = i;
    }
  }
  if (best_seq_no > 0xff)
  {
    seq_no = 0xff;
    new_idx = 0;
  }
  else
  {
    seq_no = (uint8_t)best_seq_no - (uint8_t)1;
    new_idx = best_idx + 16;
    if (new_idx >= eeprom_size)
      new_idx = 0;
  }

  data[0] = MAGIC1 | (seq_no << 16) | ((dc_value & 0x3f) << 24) |
    (bm_mode == BM_MODE_TRI12 ? (uint32_t)1 << 30 : 0) | ((uint32_t)1 << 31);
  data[1] = 0xffffffff;
  data[2] = 0xffffffff;
  data[3] = (file_idx & 0xffff) | (MAGIC2 << 16);

  /* Now write the new config. */
  res = ROM_EEPROMProgram(data, new_idx, 16);
  if (res == 0)
    return 0;
  else
    return 1;
}


int
pov_config_read(void)
{
  uint32_t res;
  uint32_t data[4];
  uint32_t i, best_seq_no, best_idx;
  uint32_t eeprom_size;
  uint8_t seq_no;

  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
  do
    res = ROM_EEPROMInit();
  while (res == EEPROM_INIT_ERROR);
  /* Not much we can do about EEPROM_INIT_RETRY here, really. */

  /* Now read all the eeprom to find the newest entry, if any. */
  best_seq_no = 0x100;
  eeprom_size = ROM_EEPROMSizeGet();

  for (i = 0; i < eeprom_size; i += 16)
  {
    ROM_EEPROMRead(data, i, 16);
    if (!is_pov_config(data))
      continue;
    seq_no = cfg_seq_no(data);
    if (best_seq_no > 0xff || wrap_cmp_8(seq_no, best_seq_no) < 0)
    {
      best_seq_no = seq_no;
      best_idx = i;
    }
  }
  if (best_seq_no <= 0xff)
  {
    /* Found a newest entry, load the config from it. */
    ROM_EEPROMRead(data, best_idx, 16);
    bm_mode = ((data[0] >> 30) & 1) ? BM_MODE_TRI12 : BM_MODE_RECT12;
    dc_value = (data[0] >> 24) & 0x3f;
    file_idx = data[3] & 0xffff;
    return 0;
  }

  /* If no config, set a default one. */
  bm_mode = BM_MODE_TRI12;
  dc_value = 4;
  file_idx = 0;
  return pov_config_write();
}


void
pov_config_accept_packet(uint8_t *packet)
{
  if (packet[2] >= 0 && packet[2] <= 63)
    dc_value = packet[2];
  if (packet[3] == 0)
    bm_mode = BM_MODE_RECT12;
  else if (packet[3] == 1)
    bm_mode = BM_MODE_TRI12;

  pov_config_write();

  /*
    Just reset the MCU. This is the easiest way to re-configure the DC value.
    Alternatively, we should temporarily disable the TLC5940 refresh, wait for
    everything to be idle, then re-load the DC values into the TLCs and start
    everything running again.
  */
  ROM_SysCtlReset();
}
