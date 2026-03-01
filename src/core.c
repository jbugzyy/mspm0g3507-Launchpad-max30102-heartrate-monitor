#include "core.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>
const int CLK_KHZ = CPUCLK_FREQ/1000;
void delay_ms(uint32_t ms) {
while (ms--) {
delay_cycles(CLK_KHZ);
}
}
/**print char arry over uart*/
void put(const unsigned char *ptr_str){
while (*ptr_str)
{
DL_UART_Main_fillTXFIFO(UART_0_INST, ptr_str, 1);
//DL_UART_transmitData(UART_0_INST, *ptr_str);
while (DL_UART_Main_isBusy(UART_0_INST));
ptr_str++;
}
}
/**print 8 bit number in hex over uart*/
void put_hex8(uint8_t v) {
char buf[5]; // "0xFF"
snprintf(buf, sizeof(buf), "0x%02X", v);
put((unsigned char *)buf);
}
/**print 16 bit number in hex over uart*/
void put_hex16(uint16_t v) {
char buf[7]; // "0xFFFF"
snprintf(buf, sizeof(buf), "0x%04X", v);
put((unsigned char *)buf);
}
/**print 32 bit number in hex over uart*/
void put_hex32(uint32_t v) {
char buf[11]; // "0xFFFFFFFF"
snprintf(buf, sizeof(buf), "0x%08X", v);
put((unsigned char *)buf);
}
/**print integer over uart */
void put_num(uint32_t n) {
char b[12];
int i = 0;
if (n == 0) {
b[i++] = '0';
} else {
while (n) {
b[i++] = '0' + (n % 10);
n /= 10;
}
for (int j = 0; j < i / 2; j++) {
char t = b[j];
b[j] = b[i - 1 - j];
b[i - 1 - j] = t;
}
}
b[i] = 0;
put((unsigned char *)b);
}
/**print newline and return over uart*/
void put_nl(void)
{
put((unsigned char *)"\r\n");
}
