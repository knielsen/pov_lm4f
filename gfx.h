/*
  Current bitmap mode.

  The BM_MODE_RECT12 is a 65x65 square bitmap with 12 bit colour per pixel
  (4 bit for each of R, G, B).

  The BM_MODE_TRI12 is a polar-coordinate based bitmap, with 12 bit colour (4
  bit R, G, B). There are 32 pixels in the radial direction from center to
  perimeter. There are 16*N pixels in the tangential direction in ring N,
  giving pixels with approximately same size independently of distance from
  center.
*/
#define BM_MODE_RECT12 0
#define BM_MODE_TRI12 1

extern void bm_scanline(float angle, float unity_width, int32_t n,
                        uint8_t *scanline_buf);
extern void generate_test_image(void);
extern void accept_packet(uint8_t *packet);
extern void gen_anim_frame(const char *extra_text);
extern void next_anim_frame(void);
extern uint8_t *get_anim_frame(uint32_t *size_p);
extern void anim_reset(void);
