#include <stdint.h>
#include <math.h>

#include "gfx.h"

#include "tlc_lookup.h"

#define BM_SIZE_X 65
#define BM_SIZE_Y 65
/* Don't expect to change the size BM_BITS without code changes! */
#define BM_BITS 12
/* Round up to a whole number of 32-bit words. */
#define BM_SIZE_BYTES ((BM_SIZE_X*BM_SIZE_Y*BM_BITS + 31)/32*4)

static uint8_t bitmap[BM_SIZE_BYTES];

static inline uint16_t
bm_get_pixel(uint32_t x, uint32_t y)
{
  uint32_t pix_idx = y*BM_SIZE_X + x;
  uint32_t byte_idx = pix_idx + (pix_idx/2);
  uint16_t word = *(uint16_t *)&(bitmap[byte_idx]);
  if (pix_idx & 1)
    word >>= 4;
  else
    word &= 0xfff;
  return word;
}


static inline void
bm_put_pixel(uint16_t x, uint16_t y, uint16_t packed_col)
{
  uint32_t pix_idx = y*BM_SIZE_X + x;
  uint32_t byte_idx = pix_idx + (pix_idx/2);
  uint16_t *p = (uint16_t *)&(bitmap[byte_idx]);
  if (pix_idx & 1)
    *p = (*p & 0x000f) | (packed_col << 4);
  else
    *p = (*p & 0xf000) | (packed_col);
}

static void
bm_put_pixel_check(uint16_t x, uint16_t y, uint16_t packed_col)
{
  bm_put_pixel(x, y, packed_col);
#ifdef CHECK_PIXEL_PUT_GET
  if (bm_get_pixel(x, y) == packed_col)
    return;
  for (;;)
    ;
#endif
}


static inline uint16_t
pack_col(uint16_t r, uint16_t g, uint16_t b)
{
  return r | (g << 4) | (b << 8);
}


static inline uint16_t
unpack_r(uint16_t packed)
{
  return packed & 0xf;
}


static inline uint16_t
unpack_g(uint16_t packed)
{
  return (packed >> 4) & 0xf;
}


static inline uint16_t
unpack_b(uint16_t packed)
{
  return packed >> 8;
}


/*
  This is the main routine that converts a scanline from the bitmap into
  a vector of TLC5940 shift-out data.
*/
void
bm_scanline(float angle, int32_t n, uint8_t *scanline_buf)
{
  /* The distiance between samples for two LEDs next to each other. */
  static const float step = 1.0f;
  /* The distance from the center to the first LED. */
  static const float start = 1.0f;
  float cx = (BM_SIZE_X-1)/2.0f;
  float cy = (BM_SIZE_Y-1)/2.0f;
  float rx = cosf(angle);
  float ry = sinf(angle);
  float dx = step * rx;
  float dy = step * ry;

  /*
    We start from the outside and move towards the center, since that is how
    the TLCs are configured to shift-out.

    We add 0.5 here so that simple uint32_t cast does round-to-nearest.
  */
  cx += start*rx + (n-1)*dx + 0.5f;
  cy += start*ry + (n-1)*dy + 0.5f;

  /*
    Each RGB pixel results in 12*3 = 36 bits of shift-out data.

    We do two at a time, so that we get 9 bytes of data in each loop iteration
    and avoid having to shift-around data; this is especially useful as the
    data to shift out is big-endian, while the CPU is little-endian, so shifting
    by fractional byte width does not do what one would expect.

    We need two lookup tables for a total of 2*4096*8=64kB of flash memory.
    This is a little expensive, but should give a nice speedup over in-code
    bitmanipulations.

    Because of two-at-a-time, n must be divisible by two.
  */
  while (n > 0)
  {
    uint16_t packed1, packed2;
    uint32_t first1, first2, last1, last2;

    packed1 = bm_get_pixel((uint32_t)cx, (uint32_t)cy);
    first1 = tlc_lookup_even[packed1][0];
    last1 = tlc_lookup_even[packed1][1];
    *(uint32_t *)scanline_buf = first1;
    scanline_buf += 4;
    cx -= dx;
    cy -= dy;

    packed2 = bm_get_pixel((uint32_t)cx, (uint32_t)cy);
    first2 = tlc_lookup_odd[packed2][0];
    last2 = tlc_lookup_odd[packed2][1];
    *(uint32_t *)scanline_buf = last1 | first2;
    scanline_buf += 4;
    *scanline_buf++ = last2;
    cx -= dx;
    cy -= dy;
    n -= 2;
  }
}


 __attribute__ ((unused))
static void
bm_clear(void)
{
  uint16_t x, y;
  for (x = 0; x < BM_SIZE_X; ++x)
    for (y = 0; y < BM_SIZE_Y; ++y)
      bm_put_pixel(x, y, pack_col(0, 0, 0));
}


 __attribute__ ((unused))
static void
bm_put_disk(int32_t cx, int32_t cy, int32_t r, uint16_t packed_col)
{
  int16_t x, y;

  for (x = cx - r; x <= cx + r; ++x)
    for (y = cy - r; y <= cy + r; ++y)
      if (x >= 0 && x < BM_SIZE_X &&
          y >= 0 && y < BM_SIZE_Y &&
          (cx-x)*(cx-x) + (cy-y)*(cy-y) <= r*r)
        bm_put_pixel_check(x, y, packed_col);
}

void
generate_test_image(void)
{
/*
  Three disks, one of each colour red, blue, green.
*/
  static const uint16_t r = 10;
  static const uint16_t d = 15;

  bm_clear();
  bm_put_disk(BM_SIZE_X/2, BM_SIZE_Y/2+d, r, pack_col(15, 0, 0));
  bm_put_disk(BM_SIZE_X/2 + d*0.866f, BM_SIZE_Y/2 - d*0.5f, r, pack_col(0, 15, 0));
  bm_put_disk(BM_SIZE_X/2 - d*0.866f, BM_SIZE_Y/2 - d*0.5f, r, pack_col(0, 0, 15));

/*
  Fill up with white pixels.

  uint32_t x, y;
  for (x = 0; x < BM_SIZE_X; ++x)
    for (y = 0; y < BM_SIZE_Y; ++y)
      bm_put_pixel_check(x, y, 0xfff);
*/

/*
  Four quadrants, red, green, black, blue.

  int32_t x, y;
  for (x = 0; x < BM_SIZE_X; ++x)
  {
    for (y = 0; y < BM_SIZE_Y; ++y)
    {
      float dx = (float)x - BM_SIZE_X/2.0f;
      float dy = (float)y - BM_SIZE_Y/2.0f;
      float r = sqrtf(dx*dx+dy*dy);
      uint32_t c = r / 1.5f;
      if (c > 15)
        c = 15;
      if (x > BM_SIZE_X/2 && y > BM_SIZE_Y/2)
        bm_put_pixel(x, y, pack_col(c, 0, 0));
      else if (x < BM_SIZE_X/2 && y > BM_SIZE_Y/2)
        bm_put_pixel(x, y, pack_col(0, c, 0));
      else if (x > BM_SIZE_X/2 && y < BM_SIZE_Y/2)
        bm_put_pixel(x, y, pack_col(0, 0, c));
      else
        bm_put_pixel(x, y, pack_col(0, 0, 0));
    }
  }
*/
}
