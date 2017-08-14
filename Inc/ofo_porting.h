#ifndef OFO_PORTING_H
#define OFO_PORTING_H

#include <stdint.h>

extern void ofoP_sleep_ms(int timeMs);

extern void ofoP_gps_power_on(void);
extern void ofoP_gps_power_off(void);
extern void ofoP_gps_uart_init(void *pUartRcvDeal);
extern void ofoP_gps_uart_enable(void);
extern void ofoP_gps_uart_disable(void);

extern void ofoP_gprs_power_on(void);
extern void ofoP_gprs_power_off(void);
extern void ofoP_gprs_uart_init(void *pUartRcvDeal);
extern void ofoP_gprs_uart_enable(void);
extern void ofoP_gprs_uart_disable(void);
extern void ofoP_gprs_uart_send(char *SendData, uint16_t SendLen);

extern void ofoP_gprs_uart_send_timer_creat(void *pTimerHandle, uint16_t IntervalMs);
extern void ofoP_gprs_timer_start(void);
extern void ofoP_gprs_stop_timer(void);

void ofoP_debug_uart_init(void);
void ofoP_debug_uart_send(char *SendData, uint16_t SendLen);


#endif
