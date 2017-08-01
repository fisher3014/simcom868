#include <string.h>
#include <stdlib.h>
#include "ofo_gps.h"
#include "ofo_porting.h"

#define GPS_RX_BUF_MAX      128

typedef struct{
    gps_info_t gprsInfo;
}gps_para_t;

gps_para_t gGpsPara;


//经纬度转换成10进制
static double GpsEncodingToDegrees( char* indata )
{
    double a = strtod( indata, 0 ) ;
    double d = (int)a / 100 ;
    a -= d * 100 ;
    return d + (a / 60) ;
}

void gps_uart_rx_data_deal(uint8_t rxData)
{
    static uint16_t index = 0;
    static uint8_t rcvBuf[GPS_RX_BUF_MAX] = {0};

    rcvBuf[index++] = rxData;
    
    if (((rcvBuf[index - 1] == 0x0A) && (rcvBuf[index - 2] == 0x0D)) || (index >= GPS_RX_BUF_MAX))
    {
            /*Message ID,UTC,status(AorV),Latitude,N/S Indicator,Longitude,E/W Indicator,
            Speed Over Ground,Course Over Ground,Date,Magnetic Variation,East/West Indicator,
            Mode,Checksum*/
            //$GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V*57
            if (strstr((char *)rcvBuf, "RMC,") != NULL)
            {
								char tmpBuf[GPS_RX_BUF_MAX] = {0};
                char *pch = NULL;
                uint8_t arrayIndex = 0;

                memcpy(tmpBuf, rcvBuf, index);
                pch = strtok((char *)tmpBuf, ",");
                while (pch != NULL)
                {
                    switch (arrayIndex)
                    {
                        case 2:
                        {
                            if (strcmp(pch, "A") != 0)
                            {
                                gGpsPara.gprsInfo.isvalid = FALSE;
                                index = 0;
                                return;
                            }
                            break;
                        }
                        //Latitude
                        case 3:
                        {
                            gGpsPara.gprsInfo.lat = GpsEncodingToDegrees(pch);
                            break;
                        }
                        //N/S Indicator
                        case 4:
                        {
                            gGpsPara.gprsInfo.lat = gGpsPara.gprsInfo.lat * (-1.0);
                            break;
                        }
                        //Longitude
                        case 5:
                        {
                            gGpsPara.gprsInfo.lng = GpsEncodingToDegrees(pch);
                            break;
                        }
                        //E/W Indicator
                        case 6:
                        {
                            gGpsPara.gprsInfo.lng = gGpsPara.gprsInfo.lat *(-1.0);
                            break;
                        }
                    }
                        
                    arrayIndex++;
                    pch = strtok(NULL, ",");
                }
                gGpsPara.gprsInfo.isvalid = TRUE;
            }
            index = 0;
    }
}

void ofoE_gps_close(void)
{
    ofoP_gps_uart_disable();
    ofoP_gps_power_off();
}

gps_info_t *ofoE_gps_get_info(void)
{
    return &(gGpsPara.gprsInfo);
}

void ofoE_gps_init(void)
{
    ofoP_gps_uart_init(gps_uart_rx_data_deal);
    ofoP_gps_uart_enable();

    ofoP_gps_power_on();
    gGpsPara.gprsInfo.isvalid = FALSE;
}













