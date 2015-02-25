extern void (*sd_ssi1_handler)(void);

void sd_setup_systick(void);
void sd_config_spi(void);
uint32_t sd_card_present(void);
void sd_load_frame(uint8_t *frame, uint32_t size);
