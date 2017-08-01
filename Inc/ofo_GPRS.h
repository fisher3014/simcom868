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
extern void ofoE_gprs_init(httpResponseDeal userFunc);

// send http request, return 0 is gprs idle, -2 is busy, -1 is pData == null
extern int ofoE_gprs_http_request_send(char * pData);

// called by main loop
extern void ofoE_gprs_response_data_deal(void);

extern gprs_info_t *ofoE_gprs_get_info(void);

//return 0 is power off ok, 1 is go to sleep mode
extern int ofoE_gprs_close(void);


#endif
