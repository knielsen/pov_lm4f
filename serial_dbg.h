void serial_output_hexbyte(uint8_t byte);
void serial_output_str(const char *str);
char *uint32_tostring(char *buf, uint32_t val);
void println_uint32(uint32_t val);
void float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after);
void println_float(float f, uint32_t dig_before, uint32_t dig_after);
void serial_hexdump(uint8_t *buf, uint32_t len);
