#ifndef GPS_H
#define GPS_H
#include <stdint.h>

typedef enum{
    FALSE = 0,
    TRUE,
}BOOL;

typedef struct
{
    double lat;
    double lng;
    BOOL isvalid;
}gps_info_t;

extern void gps_init(void);

extern void gps_close(void);

extern gps_info_t *gps_get_info(void);

extern void gps_uart_rx_data_deal(uint8_t rxData);

#endif
