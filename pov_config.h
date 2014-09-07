extern uint8_t bm_mode;
extern uint8_t dc_value;
extern uint16_t file_idx;

extern int pov_config_write(void);
extern int pov_config_read(void);
extern void pov_config_accept_packet(uint8_t *packet);
