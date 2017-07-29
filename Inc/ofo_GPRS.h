#ifndef GPRS_H
#define GPRS_H

#include <stdint.h>

typedef struct{
    char gprs_sim_ver[18];
    char gprs_csq[3];
    char gprs_imsi[16];
}gprs_info_t;

typedef int8_t (*httpResponseDeal)(char *);

// init gprs module
extern void gprs_init(httpResponseDeal userFunc);

// send http request, return 0 is gprs idle, -2 is busy, -1 is pData == null
extern int gprs_http_request_send(char * pData);

// parse uart data from gprs module
extern void gprs_uart_rx_data_deal(uint8_t rxData);
extern void gprs_response_data_deal(void);

extern gprs_info_t *gprs_get_info(void);

#endif
