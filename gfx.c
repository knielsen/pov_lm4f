#include <stdint.h>
#include <string.h>
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
static volatile uint32_t render_idx = 0;
static uint32_t receive_idx = 1;


//uint8_t bm_mode = BM_MODE_RECT12;
uint8_t bm_mode = BM_MODE_TRI12;


#include "sintable_1024.c"

static float
fast_sin_mod1024(uint32_t angle_mod1024)
{
  return sintable_1024[angle_mod1024];
}


/*
  Compute approximate sine function, using table with no interpolation.
  The argument is in unity angles, so a full period is length 1, not 2PI.
  For speed, assumes that the angle is >= -16.
*/
__attribute__((unused))
static float
fast_sin_unity(float unity_angle)
{
  float idx_f = unity_angle*1024.0f;
  uint32_t idx_i = (uint32_t)(idx_f + 16384.5f);
  return fast_sin_mod1024(idx_i & 1023);
}


__attribute__((unused))
static float
fast_cos_unity(float unity_angle)
{
  float idx_f = unity_angle*1024.0f;
  uint32_t idx_i = (uint32_t)(idx_f + 16384.5f);
  return fast_sin_mod1024((idx_i+256) & 1023);
}


static uint16_t palette1_red_yellow[] = {
  0x02d,
  0x03d,
  0x03e,
  0x04e,
  0x05e,
  0x16f,
  0x17f,
  0x18f,
  0x19f,
  0x1af,
  0x1bf,
  0x2cf,
  0x2df,
  0x2ef,
  0x2ff,
  0x2ff,
  0x2ef,
  0x1df,
  0x1cf,
  0x2cf,
  0x2bf,
  0x2af,
  0x29f,
  0x38f,
  0x37f,
  0x36f,
  0x35f,
  0x34f,
  0x33e,
  0x23e,
  0x12e,
  0x01d
};


static uint32_t
tri_fb_idx(uint32_t r, uint32_t a)
{
#if TRI_BM_SIZE_X % 2 != 0
#error Following formulae assumes that TRI_BM_SIZE_X is even
#endif
  return a+r*(r-1)*(TRI_BM_SIZE_X/2);
}


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
bm_put_pixel_idx(uint8_t *bitmap, uint32_t pix_idx, uint16_t packed_col)
{
  uint32_t byte_idx = pix_idx + (pix_idx/2);
  uint16_t *p = (uint16_t *)&(bitmap[byte_idx]);
  if (pix_idx & 1)
    *p = (*p & 0x000f) | (packed_col << 4);
  else
    *p = (*p & 0xf000) | (packed_col);
}


static inline void
bm_put_pixel(uint8_t *bitmap, uint16_t x, uint16_t y, uint16_t packed_col)
{
  bm_put_pixel_idx(bitmap, y*BM_SIZE_X + x, packed_col);
}


static void
tri_bm_put_pixel(uint8_t *bitmap, uint32_t r, uint32_t a, uint16_t packed_col)
{
  bm_put_pixel_idx(bitmap, tri_fb_idx(r, a), packed_col);
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


static uint32_t
blend(uint32_t rgb1, uint32_t r2, uint32_t g2, uint32_t b2, float intensity2)
{
  uint32_t r1 = rgb1 & 0xf;
  uint32_t g1 = (rgb1 >> 4) & 0xf;
  uint32_t b1 = rgb1 >> 8;
  uint32_t saturate = 15 - (uint32_t)(intensity2 * 15.99f);
  if (r1 > saturate)
    r1 = saturate;
  if (g1 > saturate)
    g1 = saturate;
  if (b1 > saturate)
    b1 = saturate;
  return pack_col(r1+r2, g1+g2, b1+b2);
}


#include "font_addsbp.c"
#include "font_tonc.c"
//#define FONT_TO_USE addsbp_font
#define FONT_TO_USE tonc_font


static void
draw_char_aa(uint8_t *fb, char c, uint32_t r, uint32_t a,
             uint32_t col_r, uint32_t col_g, uint32_t col_b,
             const uint32_t r_offsets[8])
{
  float x_start, x_end, recip_font_pix_size;
  uint32_t i;
  const uint8_t *p;
  uint32_t ring_count = r*TRI_BM_SIZE_X;
  float fcol_r = (float)col_r+0.99f;
  float fcol_g = (float)col_g+0.99f;
  float fcol_b = (float)col_b+0.99f;
  uint32_t limit_i;
  uint32_t max_offset_r;

  if (c < ' ' || c > 127 || r < 1 || r > TRI_BM_SIZE_Y || a >= ring_count)
    return;
  p = &FONT_TO_USE[8*(c-' ')];

  max_offset_r = r_offsets[0];
  for (i = 1; i < 8; ++i)
    if (max_offset_r < r_offsets[i])
      max_offset_r = r_offsets[i];

  /* Get the start position, as a fraction [0,1). */
  x_start = (float)a/(float)ring_count;
  x_end = (float)(a+8)/(float)ring_count;
  recip_font_pix_size = 8.0f/(x_end-x_start);

  /* Loop over 8 rings, inner to outer. */
  limit_i = 8+max_offset_r;
  if (r + limit_i > TRI_BM_SIZE_Y+1)
    limit_i = (TRI_BM_SIZE_Y+1) - r;
  for (i = 0; i < limit_i; ++i)
  {
    uint32_t a_pos;
    float pix_width = 1.0f/(float)ring_count;
    float x, font_x, font_pix_inc;
    uint32_t font_in;
    uint32_t fb_idx;
    uint32_t font_bit;
    uint32_t font_l;

    font_bit = 0;

    /* Find the first pixel that overlaps with angle a. */
    a_pos = (uint32_t)(x_start*ring_count);
    fb_idx = tri_fb_idx(r, a_pos);

    x = (float)a_pos*pix_width;
    font_x = (x-x_start)*recip_font_pix_size;
    font_pix_inc = pix_width*recip_font_pix_size;
    if (font_x < 0.0f)
    {
      font_x += 1.0f;
      --font_bit;
      font_l = 0;
    }
    else
      font_l = i-r_offsets[font_bit];
    font_in = (font_l < 8 && font_bit < 8 ? (p[7-font_l]>>font_bit) & 1 : 0);

    /* Loop, processing all pixels that touch the character area. */
    while (font_bit != 8)
    {
      float in;

      font_x = font_x + font_pix_inc;

      if (font_x >= 1.0f)
      {
        font_x -= 1.0f;
        ++font_bit;
        font_l = (font_bit < 8 ? i-r_offsets[font_bit] : 0);
        in = (float)font_in * (1.0f-font_x);
        font_in = (font_l < 8 && font_bit < 8 ? (p[7-font_l]>>font_bit) & 1 : 0);
        in += (float)font_in * font_x;
      }
      else
        in = (float)font_in;

      if (in > 0.0f)
        bm_put_pixel_idx(fb, fb_idx, blend(bm_get_pixel_idx(fb, fb_idx),
                                           (uint32_t)(in*fcol_r),
                                           (uint32_t)(in*fcol_g),
                                           (uint32_t)(in*fcol_b), in));
      ++a_pos;
      ++fb_idx;
      if (a_pos >= ring_count)
      {
        a_pos -= ring_count;
        fb_idx -= ring_count;
      }
    }
    ++r;
    ring_count += TRI_BM_SIZE_X;
  }
}


__attribute__ ((unused))
static void
draw_string(uint8_t *fb, const char *s, uint32_t r, uint32_t a,
            uint32_t col_r, uint32_t col_g, uint32_t col_b)
{
  uint32_t i = strlen(s);
  static const uint32_t offsets[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  while (i > 0)
  {
    --i;
    draw_char_aa(fb, s[i], r, a, col_r, col_g, col_b, offsets);
    a += 8;
    if (a >= r*TRI_BM_SIZE_X)
      a -= r*TRI_BM_SIZE_X;
  }
}


__attribute__((unused))
static void
scrolltext1(uint8_t *fb, uint32_t c, const char *text)
{
  const uint32_t len = strlen(text);
  static const uint32_t base_r = 21;
  static const uint32_t max_a = base_r*TRI_BM_SIZE_X;
  static const uint32_t ramp_in = 38;
  static const uint32_t bounce_interval = 19;
  static const uint32_t bounce_duration = 14;
  static const uint32_t bounce_size = 70;
  static const float bounce_height = 6.4f;
  uint32_t r, a;
  uint32_t i;
  uint32_t t = c;
  int32_t idx;
  uint32_t offsets[8];
  uint32_t offset_idx;
  uint32_t frac_t;
  uint32_t bounce_t, bounce_a;

  for (i = 0; i < 8; ++i)
    offsets[i] = TRI_BM_SIZE_Y;

  r = base_r;
  a = r*TRI_BM_SIZE_X*3/4;
  idx = t/8;

  bounce_t = t % bounce_interval;
  if (bounce_t < bounce_duration)
    bounce_a = max_a*3/4 + max_a/5 + (max_a*3/(5*42))*(((t/bounce_interval)*29)%43);

  frac_t = t%8;
  offset_idx = 7 - frac_t;
  for (i = 0; i < max_a; ++i)
  {
    uint32_t col_r, col_g, col_b;

    if (i <= ramp_in)
      offsets[offset_idx] = (ramp_in-i)/(ramp_in/(TRI_BM_SIZE_Y-base_r));
    else if ((max_a-1-i) <= ramp_in)
      offsets[offset_idx] = (ramp_in-(max_a-1-i))/(ramp_in/(TRI_BM_SIZE_Y-base_r));
    else if (bounce_t < bounce_duration && (bounce_a - a)%max_a < bounce_size)
    {
      float v1, v2, amplitude;
      int32_t offset;
      v1 = (float)((bounce_a - a)%max_a) / (float)(2*(bounce_size-1));
      v2 = (float)(bounce_t) / (float)(2*(bounce_duration-1));
      amplitude = fast_sin_unity(v2);
      offset = bounce_height*amplitude*fast_sin_unity(v1);
      if (offset < 0)
        offset = 0;
      offsets[offset_idx] = offset;
    }
    else
      offsets[offset_idx] = 0;

    if (frac_t == i%8)
    {
      uint32_t str_idx;
      col_r = 0;
      col_g = 0;
      col_b = 15;
      if (idx >= 0 && (str_idx = idx % (len + 10)) < len)
      {
        draw_char_aa(fb, text[str_idx], r, a, col_r, col_g, col_b, offsets);
      }
      --idx;
      offset_idx = 0;
    }
    else
      ++offset_idx;

    ++a;
    if (a >= r*TRI_BM_SIZE_X)
      a -= r*TRI_BM_SIZE_X;
    ++t;
  }
}


static float
distance(float x1, float y1, float x2, float y2)
{
  float dx = x2 - x1;
  float dy = y2 - y1;
  return sqrtf(dx*dx + dy*dy);
}


__attribute__((unused))
static void
plasma1(uint8_t *fb, uint32_t c)
{
  uint32_t r, a;
  uint32_t idx = 0;
  uint32_t ring_count = 0;

  for (r = 1; r <= TRI_BM_SIZE_Y; ++r)
  {
    float unity_fact = (1.0f/(float)TRI_BM_SIZE_X)/r;
    ring_count += TRI_BM_SIZE_X;
    for (a = 0; a < ring_count; ++a)
    {
      float a_unity, t;
      float x, y, d1, d2;
      uint32_t palette_idx;
      uint32_t packed_col;
      float c1, c2, c3;

      a_unity = (float)a * unity_fact;
      x = ((float)r-0.5f)*fast_cos_unity(a_unity);
      y = ((float)r-0.5f)*fast_sin_unity(a_unity);
      d1 = distance(x, y, -25.6f, -19.2f);
      d2 = distance(x, y, 12.8f, 25.6f);

      t = (float)(c%(12*1024)) * (2.0f/1024.0f);
      c1 = fast_sin_unity(d1*fast_cos_unity(t)*0.04f + t*4.0f);
      c2 = fast_cos_unity(y*0.02f+t*3.0f);
      c3 = fast_cos_unity(d2*0.035f) + fast_sin_unity(t);

      palette_idx = (uint32_t)((c1+c2+c3+4.0f)*(32.0f/3.0f));
      packed_col = palette1_red_yellow[palette_idx %
            (sizeof(palette1_red_yellow)/sizeof(palette1_red_yellow[0]))];

      bm_put_pixel_idx(fb, idx, packed_col);
      ++idx;
    }
  }
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
  int even_odd;
  /* Sanity check, should be dead code. */
  if (render_idx >= 2)
    return;

  /* ToDo: Pass in angle in units of turns, instead. */
  angle1 = -angle/(float)(2.0f*F_PI);
  if (angle1 < 0.0f)
    do
      angle1 += 1.0f;
    while (angle1 < 0.0f);
  else
    while (angle1 >= 1.0f)
      angle1 -= 1.0f;
  if (unity_width >= 1.0f/(float)(TRI_BM_SIZE_X*TRI_BM_SIZE_Y))
  {
    /*
      This means we are spinning too fast. One PWM period sweep can touch
      more than two pixels in the tri-framebuffer.

      The algorithm cannot handle averaging more than two pixels. So just
      do single-point sampling instead.
    */
    angle2 = angle1;
  }
  else
    angle2 = angle1 + unity_width;

  /*
    We scan outwards through each of the 32 rings.
    In each ring, we consider the angular range that will be swept by the LED
    during this PWM cycle. This range will intersect one or two pixels in the
    tri-framebuffer. If one, then that gives the colour value directly; if two,
    then the colour value is the weighted average corresponding to the fraction
    of each pixel swept.
  */

  ring_count = TRI_BM_SIZE_X*n;
  base_idx = tri_fb_idx(n, 0);
  r = n;
  even_odd = 1;
  last = 0;                          /* Redundant, but makes compiler happy */
  while (r > 0)
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

    if (even_odd)
    {
      first = tlc_lookup_even[packed][0];
      last = tlc_lookup_even[packed][1];
      *(uint32_t *)scanline_buf = first;
      scanline_buf += 4;
      even_odd = 0;
    }
    else
    {
      first = tlc_lookup_odd[packed][0];
      *(uint32_t *)scanline_buf = last | first;
      scanline_buf += 4;
      last = tlc_lookup_odd[packed][1];
      *scanline_buf = last;
      scanline_buf += 1;
      even_odd = 1;
    }

    ring_count -= TRI_BM_SIZE_X;
    base_idx -= ring_count;
    --r;
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

  if (bm_mode != BM_MODE_RECT12)
    return;
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
tri_bm_clear(uint8_t *bitmap)
{
  uint32_t r, a;
  for (r = 1; r <= TRI_BM_SIZE_X; ++r)
    for (a = 0; a < r*TRI_BM_SIZE_Y; ++a)
      tri_bm_put_pixel(bitmap, r, a, pack_col(0, 0, 0));
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


static void
generate_test_image_rect12(void)
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


__attribute__ ((unused))
static void
generate_test_image_tri12_quadrants(void)
{
  uint8_t *bitmap = &bitmap_array[render_idx*TRI_BM_SIZE_BYTES];
  uint32_t r, a;
  uint32_t quarter_angle;

  /* different colour gradients in each quadrant. */
  tri_bm_clear(bitmap);

  quarter_angle = 0;
  for (r = 1; r <= TRI_BM_SIZE_Y; ++r)
  {
    quarter_angle += TRI_BM_SIZE_X/4;
    for (a = 0; a < quarter_angle; ++a)
    {
      uint32_t v = (r-1) % 16;
      uint16_t red = pack_col(v, 0, 0);
      uint16_t green = pack_col(0, v, 0);
      uint16_t blue = pack_col(0, 0, v);
      uint16_t yellow = pack_col(v, v, v/4);
      tri_bm_put_pixel(bitmap, r, a, red);
      tri_bm_put_pixel(bitmap, r, a+quarter_angle, green);
      tri_bm_put_pixel(bitmap, r, a+2*quarter_angle, blue);
      tri_bm_put_pixel(bitmap, r, a+3*quarter_angle, yellow);
    }
  }
}


__attribute__ ((unused))
static void
generate_test_image_tri12_checker(void)
{
  uint8_t *bitmap = &bitmap_array[render_idx*TRI_BM_SIZE_BYTES];
  uint32_t r, a;
  uint32_t ring_count;

  /* different colour gradients in each quadrant. */
  tri_bm_clear(bitmap);

  ring_count = 0;
  for (r = 1; r <= TRI_BM_SIZE_Y; ++r)
  {
    ring_count += TRI_BM_SIZE_X;
    for (a = 0; a < ring_count; ++a)
    {
      tri_bm_put_pixel(bitmap, r, a, a%5!=2 ? 0x000 : 0xfff);
    }
  }
}


__attribute__ ((unused))
static void
generate_test_image_tri12_plasma(uint32_t idx, uint32_t counter, const char *t)
{
  uint8_t *bitmap = &bitmap_array[idx*TRI_BM_SIZE_BYTES];

  /* different colour gradients in each quadrant. */
  plasma1(bitmap, counter);
  /* ToDo: I could add some simple but cool colour/blink control codes! */
  scrolltext1(bitmap, counter,
              "Welcome to LABITAT at the Made2014 Celebrations, held on "
              "August 29-30! This is the POV fan by knielsen. Using 3 wings "
              "each with 32 RGB LEDs, it has a display resolution of 65x65 "
              "rectangular or 32 radial by 512 tangential polar. Colour depth "
              "is 12 bit non-linear (4096 colours). Driven by the Texas "
              "Instruments TM4C1232 microcontroller @ 80 MHz, 32 kB RAM, it "
              "can display animations generated on-board, or video streamed "
              "from a PC over nRf24L01+ wireless. Wireless transfer speed is "
              "155 kByte/second.         My LED cube goes to ELEVEN!");
  if (t)
    draw_string(bitmap, t, 13, 0, 10, 0, 10);
}


void
generate_test_image(void)
{
  if (bm_mode == BM_MODE_RECT12)
  {
    generate_test_image_rect12();
  }
  else if (bm_mode == BM_MODE_TRI12)
  {
    //generate_test_image_tri12_quadrants();
    generate_test_image_tri12_checker();
    generate_test_image_tri12_plasma(render_idx, 200, NULL);
  }
}


void
next_anim_frame(const char *extra_text)
{
  static uint32_t counter = 0;
  if (bm_mode == BM_MODE_RECT12)
  {
  }
  else if (bm_mode == BM_MODE_TRI12)
  {
    uint32_t idx = receive_idx;
    generate_test_image_tri12_plasma(idx, counter++, extra_text);
    render_idx = idx;
    receive_idx = 1 - idx;
  }
}
