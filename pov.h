#define F_PI 3.141592654f

/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


#define likely(x) __builtin_expect((x)!=0, 1)
#define unlikely(x) __builtin_expect((x)!=0, 0)

static inline void barrier(void)
{
        __asm volatile("" : : : "memory");
}


/*
  SD-card IOs.

  Note that to change these, may require additional changes in
  config_nrf_sd_ssi_gpio().
*/
#define SD_SSI_BASE SSI1_BASE
#define SD_CS_BASE GPIO_PORTB_BASE
#define SD_CS_PIN GPIO_PIN_1
#define SD_CD_BASE GPIO_PORTB_BASE
#define SD_CD_PIN GPIO_PIN_2


void led_on(void);
void led_off(void);
