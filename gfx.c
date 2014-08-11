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

#define TRI_BM_SIZE_X 16
#define TRI_BM_SIZE_Y 32
#define TRI_BM_SIZE_BYTES ((TRI_BM_SIZE_X*TRI_BM_SIZE_Y*(TRI_BM_SIZE_Y+1)/2*3+1)/2)

#if 2*TRI_BM_SIZE_BYTES < 3*BM_SIZE_BYTES
#error Too small bitmap buffer
#endif
static uint8_t bitmap_array[2*TRI_BM_SIZE_BYTES];
static uint32_t render_idx = 0;
static uint32_t receive_idx = 1;


uint8_t bm_mode = BM_MODE_RECT12;


static inline uint16_t
bm_get_pixel_idx(uint8_t *bitmap, uint32_t pix_idx)
{
  uint32_t byte_idx = pix_idx + (pix_idx/2);
  uint16_t word = *(uint16_t *)&(bitmap[byte_idx]);
  if (pix_idx & 1)
    word >>= 4;
  else
    word &= 0xfff;
  return word;
}


static inline uint16_t
bm_get_pixel(uint8_t *bitmap, uint32_t x, uint32_t y)
{
  return bm_get_pixel_idx(bitmap, y*BM_SIZE_X + x);
}


static inline void
bm_put_pixel(uint8_t *bitmap, uint16_t x, uint16_t y, uint16_t packed_col)
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
bm_put_pixel_check(uint8_t *bitmap, uint16_t x, uint16_t y, uint16_t packed_col)
{
  bm_put_pixel(bitmap, x, y, packed_col);
#ifdef CHECK_PIXEL_PUT_GET
  if (bm_get_pixel(bitmap, x, y) == packed_col)
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


static void
bm_scanline_rect12(float angle, int32_t n, uint8_t *scanline_buf)
{
  uint8_t *bitmap = &bitmap_array[BM_SIZE_BYTES*render_idx];
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

    packed1 = bm_get_pixel(bitmap, (uint32_t)cx, (uint32_t)cy);
    first1 = tlc_lookup_even[packed1][0];
    last1 = tlc_lookup_even[packed1][1];
    *(uint32_t *)scanline_buf = first1;
    scanline_buf += 4;
    cx -= dx;
    cy -= dy;

    packed2 = bm_get_pixel(bitmap, (uint32_t)cx, (uint32_t)cy);
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


/*
  Generate a scanline for a row of LEDs at a specific angle, from a
  framebuffer in the radial corrdinate format.
*/
static void
bm_scanline_tri12(float angle, float unity_width, int32_t n,
                  uint8_t *scanline_buf)
{
  uint32_t r;
  uint32_t ring_count, base_idx;
  uint32_t first, last;
  float angle1, angle2;
  uint8_t *bitmap = &bitmap_array[render_idx*TRI_BM_SIZE_BYTES];

  /* Sanity check, should be dead code. */
  if (render_idx >= 2)
    return;

  /* ToDo: Pass in angle in units of turns, instead. */
  angle1 = angle/(float)(2.0f*F_PI);
  angle2 = angle1 + unity_width;
  if ((angle2 - angle1) >= 1.0f/(float)(TRI_BM_SIZE_X*TRI_BM_SIZE_Y))
  {
    /*
      This means we are spinning too fast. One PWM period sweep can touch
      more than two pixels in the tri-framebuffer.

      The algorithm cannot handle averaging more than two pixels. So just
      do single-point sampling instead.
    */
    angle2 = angle1;
  }

  while (angle >= 1.0f)
    angle -= 1.0f;

  /*
    We scan outwards through each of the 32 rings.
    In each ring, we consider the angular range that will be swept by the LED
    during this PWM cycle. This range will intersect one or two pixels in the
    tri-framebuffer. If one, then that gives the colour value directly; if two,
    then the colour value is the weighted average corresponding to the fraction
    of each pixel swept.
  */

  ring_count = TRI_BM_SIZE_X;
  base_idx = 0;
  r = 1;
  last = 0;                          /* Redundant, but makes compiler happy */
  while (r <= n)
  {
    uint32_t idx1;
    int32_t int_a2;
    uint32_t packed;
    float a1, a2;

    a1 = angle1*(float)ring_count;
    a2 = angle2*(float)ring_count;
    int_a2 = (int32_t)a2;
    if ((float)int_a2 <= a1)
    {
      /* Just a single pixel. */
      idx1 = base_idx + int_a2;
      packed = bm_get_pixel_idx(bitmap, idx1);
    }
    else
    {
      /* Weighted average of two pixels. */
      uint32_t packed1, packed2;
      uint32_t idx2;
      float frac1, frac2;

      frac2 = (a2 - (float)int_a2)/(a2 - a1);
      frac1 = 1.0f - frac2;
      idx2 = base_idx + int_a2;
      idx1 = idx2 - 1;
      packed1 = bm_get_pixel_idx(bitmap, idx1);
      packed2 = bm_get_pixel_idx(bitmap, idx2);
      /*
        ToDo: If we were to do the weighted average on the TLC PWM values,
        which are 12 bit, instead of computing a 4-bit RGB value, we would
        get higher accuracy. But this does not take into account the rather
        high gamma correction of the RGB-to-PWM conversion, and anyway this
        code is much simpler.
      */
      packed = pack_col((int32_t)(0.5f + frac1*(float)unpack_r(packed1)
                                       + frac2*(float)unpack_r(packed2)),
                        (int32_t)(0.5f + frac1*(float)unpack_g(packed1)
                                       + frac2*(float)unpack_g(packed2)),
                        (int32_t)(0.5f + frac1*(float)unpack_b(packed1)
                                       + frac2*(float)unpack_b(packed2)));
    }

    if (r %2)
    {
      first = tlc_lookup_even[packed][0];
      last = tlc_lookup_even[packed][1];
      *(uint32_t *)scanline_buf = first;
      scanline_buf += 4;
    }
    else
    {
      first = tlc_lookup_odd[packed][0];
      *(uint32_t *)scanline_buf = last | first;
      scanline_buf += 4;
      last = tlc_lookup_odd[packed][1];
      *scanline_buf = last;
      scanline_buf += 1;
    }

    base_idx += ring_count;
    ring_count += r*TRI_BM_SIZE_Y;
    ++r;
  }
}


/*
  This is the main routine that converts a scanline from the bitmap into
  a vector of TLC5940 shift-out data.
*/
void
bm_scanline(float angle, float unity_width, int32_t n, uint8_t *scanline_buf)
{
  if (bm_mode == BM_MODE_RECT12)
    bm_scanline_rect12(angle, n, scanline_buf);
  else if (bm_mode == BM_MODE_TRI12)
    bm_scanline_tri12(angle, unity_width, n, scanline_buf);
}


static uint8_t last_run_num = 0;

void
accept_packet(uint8_t *packet)
{
  uint8_t *buf;
  uint32_t i, offset;
  uint8_t run_num = packet[0];
  if (run_num < last_run_num)
  {
    /* Full frame received, though last packet was lost. */
    receive_idx = (receive_idx+1)%3;
    render_idx = (render_idx+1)%3;
  }

  /* Copy the received bytes into the appropriate place in the bitmap. */
  buf = &bitmap_array[BM_SIZE_BYTES*receive_idx];
  offset = (uint32_t)run_num * 31;
  for (i = 0; i < 31 && i+offset < BM_SIZE_BYTES; ++i)
    buf[i+offset] = packet[i+1];

  /*
    In case of lost packet(s), copy from the previous received one to
    minimise glitches.
  */
  if (run_num > last_run_num + 1)
  {
    uint8_t *old_buf = &bitmap_array[BM_SIZE_BYTES*render_idx];
    for (i = ((uint32_t)last_run_num+1)*31; i < offset; ++i)
      buf[i] = old_buf[i];
  }

  if (run_num == 204)
  {
    /* We received the last packet of a frame. Move to the next one. */
    receive_idx = (receive_idx+1)%3;
    render_idx = (render_idx+1)%3;
    last_run_num = 0;
  }
  else
    last_run_num = run_num;
}


__attribute__ ((unused))
static void
bm_clear(uint8_t *bitmap)
{
  uint16_t x, y;
  for (x = 0; x < BM_SIZE_X; ++x)
    for (y = 0; y < BM_SIZE_Y; ++y)
      bm_put_pixel(bitmap, x, y, pack_col(0, 0, 0));
}


__attribute__ ((unused))
static void
bm_put_disk(uint8_t *bitmap, int32_t cx, int32_t cy, int32_t r,
            uint16_t packed_col)
{
  int16_t x, y;

  for (x = cx - r; x <= cx + r; ++x)
    for (y = cy - r; y <= cy + r; ++y)
      if (x >= 0 && x < BM_SIZE_X &&
          y >= 0 && y < BM_SIZE_Y &&
          (cx-x)*(cx-x) + (cy-y)*(cy-y) <= r*r)
        bm_put_pixel_check(bitmap, x, y, packed_col);
}

void
generate_test_image(void)
{
  uint8_t *bitmap = &bitmap_array[BM_SIZE_BYTES*render_idx];
/*
  Three disks, one of each colour red, blue, green.
*/
  static const uint16_t r = 10;
  static const uint16_t d = 15;

  bm_clear(bitmap);
  bm_put_disk(bitmap, BM_SIZE_X/2, BM_SIZE_Y/2+d, r,
              pack_col(15, 0, 0));
  bm_put_disk(bitmap, BM_SIZE_X/2 + d*0.866f, BM_SIZE_Y/2 - d*0.5f, r,
              pack_col(0, 15, 0));
  bm_put_disk(bitmap, BM_SIZE_X/2 - d*0.866f, BM_SIZE_Y/2 - d*0.5f, r,
              pack_col(0, 0, 15));

/*
  Horizontal line, for alignment.
  uint32_t i;
  bm_clear(bitmap);
  for (i= 0; i < BM_SIZE_X; ++i)
    bm_put_pixel(bitmap, i, BM_SIZE_Y/2, i%3==0 ? pack_col(15, 0, 0) : (i%3==1 ? pack_col(0, 15, 0) : pack_col(0, 0, 15)));
*/

/*
  Colour gradients.
  uint32_t i, j;
  bm_clear();
  for (i= 0; i < BM_SIZE_X; ++i)
  {
    for (j = 0; j < BM_SIZE_Y; ++j)
    {
      uint32_t r, g, b;
      b = i*15/BM_SIZE_X;
      g = j*15/BM_SIZE_Y;
      r = ((BM_SIZE_X-1-i) + (BM_SIZE_Y-1-j)) * 15 / (BM_SIZE_X + BM_SIZE_Y);
      bm_put_pixel(i, j, pack_col(r, g, b));
    }
  }
*/

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
