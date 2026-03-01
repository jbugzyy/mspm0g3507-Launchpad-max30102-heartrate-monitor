#ifndef CORE_H
#define CORE_H
#include <stdint.h>
/* Clock frequency in kHz (defined in core.c) */
extern const int CLK_KHZ;
/* Delay function (milliseconds) */
void delay_ms(uint32_t ms);
/*uart send function*/
void put(const unsigned char *ptr_str);
void put_hex8(uint8_t v);
void put_hex16(uint16_t v);
void put_hex32(uint32_t v);
void put_num(uint32_t n);
void put_nl(void);
#endif /* CORE_H */
