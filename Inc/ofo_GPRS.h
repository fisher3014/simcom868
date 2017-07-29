#ifndef GPRS_H
#define GPRS_H

#include <stdint.h>

//结构体
typedef struct
{
    unsigned  char mac[6];        //MAC
    unsigned  char ser[2];        //MAC
    unsigned  int time;         //开锁时间
} SafeData;

//GPRS 通信超时
#define GPRS_TIMEROUT  180

//#define ServerTest

//定义发送数据长度
#define outBufferLen 540
#define inBufferLen  128

#define GPRS_UART_TX_BUF_SIZE                1024                                         /**< GPRS UART TX buffer size. */
#define GPRS_UART_RX_BUF_SIZE                128                                          /**< GPRS UART RX buffer size. */

#define GPRS_RX_PIN_NUMBER 6
#define GPRS_TX_PIN_NUMBER 5

#define UG_PWRKEY 30
#define UG_PWEDWN 19
#define UG_RESET  18
#define UG_STATUS 17
#define UG_EN_PIN 29
#define UG_DTR_PIN 28

//#define GPRS_SLEEP_MODE nrf_gpio_pin_clear(UG_DTR_PIN);nrf_delay_ms(2)
//#define GPRS_WAKEUP_MODE nrf_gpio_pin_set(UG_DTR_PIN);nrf_delay_ms(2)

extern void gprs_start_communicate(void);
extern void gprs_response_data_deal(void);
extern void uart_rx_data_deal(uint8_t rxData);
//GPRS进入睡眠状态
void GPRS_StartSleep(void);

//uart 结束通信
void GPRS_uart_close(uint8_t isCloseGPRS);
//GPRS 参数初始化
extern void GPRS_Parameter_init(void);
//GPRS 发送数据
void GPRS_SendData(char *SendData, uint16_t SendLen);
//GPRS 开关
void gprs_power_on(void);
//GPRS 关
void GPRS_close(void);
//
void Encrypted(void);

extern uint8_t isGPRSPostData;//
extern uint8_t isGPRSDownLoadFW;//GPRS下载
//是否启动SleepMode
extern uint8_t isGPRSStartSleep;
extern uint16_t GPRSSleepTime;
#endif
