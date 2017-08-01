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

extern void ofoE_gps_init(void);

extern void ofoE_gps_close(void);

extern gps_info_t *ofoE_gps_get_info(void);

#endif
