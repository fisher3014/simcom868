#include <string.h>
#include <stdlib.h>
#include "ofo_GPS.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"


#define GPS_RX_BUF_MAX      128

typedef struct{
    gps_info_t gprsInfo;
}gps_para_t;

gps_para_t gpsPara;

static UART_HandleTypeDef huart2;


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
                                gpsPara.gprsInfo.isvalid = FALSE;
                                index = 0;
                                return;
                            }
                            break;
                        }
                        //Latitude
                        case 3:
                        {
                            gpsPara.gprsInfo.lat = GpsEncodingToDegrees(pch);
                            break;
                        }
                        //N/S Indicator
                        case 4:
                        {
                            gpsPara.gprsInfo.lat = gpsPara.gprsInfo.lat * (-1.0);
                            break;
                        }
                        //Longitude
                        case 5:
                        {
                            gpsPara.gprsInfo.lng = GpsEncodingToDegrees(pch);
                            break;
                        }
                        //E/W Indicator
                        case 6:
                        {
                            gpsPara.gprsInfo.lng = gpsPara.gprsInfo.lat *(-1.0);
                            break;
                        }
                    }
                        
                    arrayIndex++;
                    pch = strtok(NULL, ",");
                }
                gpsPara.gprsInfo.isvalid = TRUE;
            }
            index = 0;
    }
}

static void _gpr_uart_enable(void)
{
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
}

static void gps_uart_disable(void)
{
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_RXNE);
}

static void _gps_uart_init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

void gps_close(void)
{
    gps_uart_disable();
}

gps_info_t *gps_get_info(void)
{
    return &(gpsPara.gprsInfo);
}

/*
static void _gps_uart_send_data(char *SendData, uint16_t SendLen)
{
    if(HAL_UART_Transmit(&huart2, (uint8_t*)SendData, SendLen,100)!= HAL_OK)
    {
        Error_Handler();
    }
}*/

static void _gps_power_on(void)
{
    /*if 868 module uart2 and gps uart connected send AT+CGNSPWR=1 to enable gps and
        send at cmds to get gps data
      if not, pull up GNSS_EN, and 868 will send gprs data to uart automatically
      */
    /*
    _gps_uart_send_data("AT+CGNSPWR=1\r\n", strlen("AT+CGNSPWR=1\r\n"));
    HAL_Delay(100);
    _gps_uart_send_data("AT+CGPSINF=32\r\n", strlen("AT+CGPSINF=32\r\n"));*/
}

void gps_init(void)
{
    _gps_uart_init();
    _gpr_uart_enable();

    _gps_power_on();
    gpsPara.gprsInfo.isvalid = FALSE;
}













