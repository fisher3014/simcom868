#include <string.h>
#include "ofo_GPRS.h"
#include "stm32l4xx_hal.h"
#include "json_utils.h"
#include "base64.h"
#include "stm32l4xx_hal_tim.h"

#define DEBUG_SERVER_GPRS

#ifdef DEBUG_SERVER_GPRS
//test server
#define Remote_Server "http://47.88.63.224:9609/titania"
#define Remote_Host "47.88.63.224:9609"
#define Remote_Post "/titania"
#else
#define Remote_Server "http://3glock.lock.ofo.com/titania"
#define Remote_Host "3glock.lock.ofo.com"
#define Remote_Post "/titania"
#endif

#define AT_HTTPPARA2_PARA   "AT+HTTPPARA=\"URL\",\""##Remote_Server##"\"\r\n"

#define REQUEST_DATA_LEN    540
#define RESPONSE_DATA_LEN   128

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


typedef enum{
    AT_INVALID_CMD = 0,
    AT_LINK_CHECK_CMD,
    AT_CLOSE_ECHO_CMD,
    AT_GSV_CMD,
    AT_CPIN_SIMCOM_CMD,
    AT_CIMI_CMD,
    AT_CSQ_SIMCOM_CMD,
    AT_CGREG_CMD,
    AT_SAPBR1_CMD, // config scene
    AT_SAPBR2_CMD, // config scene
    AT_SAPBR3_CMD, // active gprs
    AT_SAPBR4_CMD, // query ip
    AT_HTTPINIT_CMD,
    AT_HTTPPARA1_CMD, // config http para
    AT_HTTPPARA2_CMD, // config url
    AT_HTTPPARA3_CMD, // config http content
    AT_HTTPPARA4_CMD, // config http timeout
    AT_HTTPPOST_PRE_CMD, // config post length
    AT_HTTPPOST_PRE1_CMD, // put data to module
    AT_HTTPPPOSTACTION_CMD, // module put data to server
    AT_HTTPREAD_CMD,
    AT_CFUN_ENTER_CMD, // enter flying mode
    AT_CFUN1_EXIT_CMD, // exit flying mode
    AT_ENTER_SLEEP_MODE, //set 868 low clk
    AT_HTTPTERM_CMD,
    AT_MAX_CMD,
}atCmdId_e;

typedef enum{
    CMD_RESULT_INIT = 0,
    CMD_RESULT_SUCCESS,
    CMD_RESULT_FAILED,
}atCmdExecResult_e;

typedef void (* atRequestSendFunc)(atCmdId_e);
typedef void (* atResponseDealFunc)(void);

typedef struct{
    uint16_t timeval; // 1 == 100ms
    uint8_t retryTimes;
    atCmdExecResult_e atCmdExecResult;
}cmdAttribute_t;

typedef struct{
    atCmdId_e atCmdId;
    char *pCmdStr;
    atRequestSendFunc pReqFunc;
    atResponseDealFunc pRspFunc;
    cmdAttribute_t  cmdAttribute;
}atCmdCenter_t;

typedef enum{
  GPRS_POWER_OFF = 0,
  GPRS_POWER_ON,
  GPRS_INITING,   //establishing connection
  GPRS_READY,
  GPRS_SENDING,
}gprs_status_e;

typedef struct{
    uint16_t gprsDelayCnt;  // 0 is not delay, any others do delay 1 == 100ms
    atCmdId_e currentCmdId;
    uint8_t isGetServerData;
    uint8_t isGprsSleeping;
    gprs_status_e gprsStatus;
    char requestBuffer[REQUEST_DATA_LEN];
    char responseBuffer[RESPONSE_DATA_LEN];
    httpResponseDeal httpResUserFunc;
    gprs_info_t gprsInfo;
}gprsPara_t;

static void _at_request_cmd(atCmdId_e cmdId);
static void _at_dummy_deal(void);
static void _at_link_check_response_deal(void);
static void _at_close_echo_response_deal(void);
static void _at_gsv_response_deal(void);
static void _at_cpin_response_deal(void);
static void _at_cimi_cmd_reponse_deal(void);
static void _at_csq_response_deal(void);
static void _at_cgreg_response_deal(void);
static void _at_sapbr1_response_deal(void);
static void _at_sapbr2_response_deal(void);
static void _at_sapbr3_response_deal(void);
static void _at_sapbr4_response_deal(void);
static void _at_httpinit_response_deal(void);
static void _at_httppara1_response_deal(void);
static void _at_httppara2_response_deal(void);
static void _at_httppara3_response_deal(void);
static void _at_httppara4_response_deal(void);
static void _at_httppost_pre_response_deal(void);
static void _at_httppost_pre1_response_deal(void);
static void _at_httppost_action_response_deal(void);
static void _at_httpread_response_deal(void);
static void _at_cfun_enter_response_deal(void);
static void _at_cfun1_exit_response_deal(void);
static void _at_enter_sleep_response_deal(void);
static void _at_httpterm_response_deal(void);

static void _gprs_uart_send_timer_creat(void);
static void _gprs_establish_connection(void);

static void _gprs_timer_handler(void);
static int _string_find(const char *pSrc, const char *pDst);
static void _gprs_uart_send_data(char *SendData, uint16_t SendLen);
static void _gprs_power_on(void);
static void _gprs_power_off(void);
static void _gprs_uart_init(void);
static void _gprs_sleep_ms(int timeMs);

TIM_HandleTypeDef gprsTimerId;
UART_HandleTypeDef huart2;

static gprsPara_t gGprsPara;

atCmdCenter_t gAtCmdCenter[AT_MAX_CMD] = {
        {AT_INVALID_CMD,            NULL,                                               _at_request_cmd,  _at_dummy_deal,                    {0,   0,   CMD_RESULT_INIT}},
        {AT_LINK_CHECK_CMD,         "AT\r\n",                                           _at_request_cmd, _at_link_check_response_deal,      {5,   100, CMD_RESULT_INIT}},
        {AT_CLOSE_ECHO_CMD,         "ATE0\r\n",                                         _at_request_cmd, _at_close_echo_response_deal,      {5,   100, CMD_RESULT_INIT}},
        {AT_GSV_CMD,                "AT+GSV\r\n",                                       _at_request_cmd, _at_gsv_response_deal,             {5,   100, CMD_RESULT_INIT}},
        {AT_CPIN_SIMCOM_CMD,        "AT+CPIN?\r\n",                                     _at_request_cmd, _at_cpin_response_deal,            {5,   20,  CMD_RESULT_INIT}},
        {AT_CIMI_CMD,               "AT+CIMI\r\n",                                      _at_request_cmd, _at_cimi_cmd_reponse_deal,         {5,   20,  CMD_RESULT_INIT}},
        {AT_CSQ_SIMCOM_CMD,         "AT+CSQ\r\n",                                       _at_request_cmd, _at_csq_response_deal,             {5,   100, CMD_RESULT_INIT}},
        {AT_CGREG_CMD,              "AT+CGREG?\r\n",                                    _at_request_cmd, _at_cgreg_response_deal,           {5,   100, CMD_RESULT_INIT}},
        {AT_SAPBR1_CMD,             "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n",            _at_request_cmd, _at_sapbr1_response_deal,          {5,   2,   CMD_RESULT_INIT}},
        {AT_SAPBR2_CMD,             "AT+SAPBR=3,1,\"APN\",\"ofo.gdsp\"\r\n",            _at_request_cmd, _at_sapbr2_response_deal,          {5,   2,   CMD_RESULT_INIT}},
        {AT_SAPBR3_CMD,             "AT+SAPBR=1,1\r\n",                                 _at_request_cmd, _at_sapbr3_response_deal,          {400, 2,   CMD_RESULT_INIT}},
        {AT_SAPBR4_CMD,             "AT+SAPBR=2,1\r\n",                                 _at_request_cmd, _at_sapbr4_response_deal,          {5,   2,   CMD_RESULT_INIT}},
        {AT_HTTPINIT_CMD,           "AT+HTTPINIT\r\n",                                  _at_request_cmd, _at_httpinit_response_deal,        {5,   2,   CMD_RESULT_INIT}},
        {AT_HTTPPARA1_CMD,          "AT+HTTPPARA=\"CID\",1\r\n",                        _at_request_cmd, _at_httppara1_response_deal,       {5,   2,   CMD_RESULT_INIT}},
        {AT_HTTPPARA2_CMD,          AT_HTTPPARA2_PARA,                                  _at_request_cmd, _at_httppara2_response_deal,       {5,   2,   CMD_RESULT_INIT}},
        {AT_HTTPPARA3_CMD,          "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n", _at_request_cmd, _at_httppara3_response_deal,       {5,   2,   CMD_RESULT_INIT}},
        {AT_HTTPPARA4_CMD,          "AT+HTTPPARA=\"TIMEOUT\",40\r\n",                   _at_request_cmd, _at_httppara4_response_deal,       {5,   2,   CMD_RESULT_INIT}},
        {AT_HTTPPOST_PRE_CMD,       "AT+HTTPDATA=",                                     _at_request_cmd, _at_httppost_pre_response_deal,    {5,   2,   CMD_RESULT_INIT}},
        {AT_HTTPPOST_PRE1_CMD,      NULL,                                               _at_request_cmd, _at_httppost_pre1_response_deal,   {100, 2,   CMD_RESULT_INIT}},
        {AT_HTTPPPOSTACTION_CMD,    "AT+HTTPACTION=1\r\n",                              _at_request_cmd, _at_httppost_action_response_deal, {400, 2,   CMD_RESULT_INIT}},
        {AT_HTTPREAD_CMD,           "AT+HTTPREAD\r\n",                                  _at_request_cmd, _at_httpread_response_deal,        {100, 2,   CMD_RESULT_INIT}},
        {AT_CFUN_ENTER_CMD,         "AT+CFUN=0\r\n",                                    _at_request_cmd, _at_cfun_enter_response_deal,      {5,   2,   CMD_RESULT_INIT}},
        {AT_CFUN1_EXIT_CMD,         "AT+CFUN=1\r\n",                                    _at_request_cmd, _at_cfun1_exit_response_deal,      {5,   2,   CMD_RESULT_INIT}},
        {AT_ENTER_SLEEP_MODE,       "AT+CSCLK=1\r\n",                                   _at_request_cmd, _at_enter_sleep_response_deal,     {5,   2,   CMD_RESULT_INIT}},
        {AT_HTTPTERM_CMD,           "AT+HTTPTERM\r\n",                                  _at_request_cmd, _at_httpterm_response_deal,        {5,   2,   CMD_RESULT_INIT}},
    };

gprs_info_t *gprs_get_info(void)
{
    return &(gGprsPara.gprsInfo);
}

static void _gprs_sleep_ms(int timeMs)
{
    HAL_Delay(timeMs);
}

static void _at_config_http_content_len(void)
{   
    static char tmpBuf[30] = {0};

    sprintf(tmpBuf, "%s%d,10000\r\n", gAtCmdCenter[AT_HTTPPOST_PRE_CMD].pCmdStr, strlen(gGprsPara.requestBuffer));
    _gprs_uart_send_data(tmpBuf, strlen(tmpBuf));
}

static void _at_send_post_data_to_module(void)
{
    _gprs_uart_send_data(gGprsPara.requestBuffer, strlen(gGprsPara.requestBuffer));
}

static void _at_request_cmd(atCmdId_e cmdId)
{
    static atCmdId_e tmpCmdID = AT_INVALID_CMD;
    static uint16_t sendCnts = 0;
    static uint16_t timeval = 0; // 1 == 100ms

    // cmd invalid
    if (AT_INVALID_CMD == cmdId)
    {
        // clear previous valid cmd
        if (AT_INVALID_CMD != tmpCmdID)
        {
            tmpCmdID = AT_INVALID_CMD;
            sendCnts = 0;
            timeval = 0;
        }
        return;
    }

    if (CMD_RESULT_FAILED == gAtCmdCenter[cmdId].cmdAttribute.atCmdExecResult)
    {
        gAtCmdCenter[cmdId].cmdAttribute.atCmdExecResult = CMD_RESULT_INIT;
        sendCnts = 0;
        timeval = 0;
    }

    // retry times control
    if (tmpCmdID == cmdId)
    {   
        if (sendCnts >= gAtCmdCenter[cmdId].cmdAttribute.retryTimes)
        {
            sendCnts = 0;
            timeval = 0;
            if (AT_CFUN_ENTER_CMD == cmdId || AT_CFUN1_EXIT_CMD == cmdId)
            {
                // reboot gprs
                gprs_close();
                _gprs_establish_connection();
                tmpCmdID = AT_INVALID_CMD;
            }
            else
            {
                gGprsPara.currentCmdId = AT_CFUN_ENTER_CMD;
                tmpCmdID = AT_CFUN_ENTER_CMD;
            }
            return;
        }
    }
    else
    {
        tmpCmdID = cmdId;
        sendCnts = 0;
        timeval = 0;
    }

    // before response(error or success) time out don't send another AT cmd
    if (timeval == 0)
    {
        if (AT_HTTPPOST_PRE_CMD == cmdId)
        {
            _at_config_http_content_len();
        }
        else if (AT_HTTPPOST_PRE1_CMD == cmdId)
        {
            _at_send_post_data_to_module();
        }
        else
        {
            _gprs_uart_send_data(gAtCmdCenter[cmdId].pCmdStr, strlen(gAtCmdCenter[cmdId].pCmdStr));
        }

        sendCnts++;
        timeval++;
    }
    else
    {
        timeval++;
        if (timeval >= gAtCmdCenter[cmdId].cmdAttribute.timeval)
        {
            timeval = 0;
        }
    }
}

static void _at_dummy_deal(void)
{
    //do nothing dummy
}

static void _gprs_cmd_deal(atCmdId_e cmdId)
{
    //during gprs cmd delay, don't send any other cmd
    if (0 != gGprsPara.gprsDelayCnt)
    {
        gGprsPara.gprsDelayCnt--;
        return;
    }
    
    _at_request_cmd(gGprsPara.currentCmdId);
}

static void _at_link_check_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
				memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_CLOSE_ECHO_CMD;
        //nrf_gpio_pin_clear(UG_PWRKEY);
    }
}

static void _at_close_echo_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_GSV_CMD;
    }
}

static void _at_gsv_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "Revision:") != NULL) //存在该字段
    {
        int strindex = _string_find(gGprsPara.responseBuffer, "Revision:");
        if (strindex >= 0)
        {
            strindex += strlen("Revision: ");
            memcpy(gGprsPara.gprsInfo.gprs_sim_ver, gGprsPara.responseBuffer + strindex, 16);
					  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
            gGprsPara.currentCmdId = AT_CPIN_SIMCOM_CMD;
        }
        else
				{
						memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
				}
    }
}

static void _at_cpin_response_deal(void)
{
		static uint8_t okFlag = 0;
	
		if (1 == okFlag)
		{
			if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
			{
				memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
				gGprsPara.currentCmdId = AT_CIMI_CMD; 
			}
		}
	
    if (strstr(gGprsPara.responseBuffer, "+CPIN:") != NULL)
    {
        if (strstr(gGprsPara.responseBuffer, "+CPIN: READY\r\n") != NULL) 
        {
            //gGprsPara.currentCmdId = AT_CIMI_CMD; 
						okFlag = 1;
        }
        else
        {
            gAtCmdCenter[AT_CPIN_SIMCOM_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
        }
        memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
    }
}

static void _at_cimi_cmd_reponse_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
	  {
	    memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
		  gGprsPara.currentCmdId = AT_CSQ_SIMCOM_CMD;
	  }
}

static void _at_csq_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "+CSQ:") != NULL)
    {
        int strindex = _string_find(gGprsPara.responseBuffer, "+CSQ: ");
        strindex += strlen("+CSQ: ");
        gGprsPara.gprsInfo.gprs_csq[0] = *(gGprsPara.responseBuffer + strindex);
        if (*(gGprsPara.responseBuffer + strindex + 1) != ',')
        {
            gGprsPara.gprsInfo.gprs_csq[1] = *(gGprsPara.responseBuffer + strindex + 1);
        }
        
        if (strcmp(gGprsPara.gprsInfo.gprs_csq, "0") == 0)
        {
					  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
            gAtCmdCenter[AT_CSQ_SIMCOM_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
        }
        else
        {
					  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
            gGprsPara.currentCmdId = AT_CGREG_CMD;
        }
    }
}

static void _at_cgreg_response_deal(void)
{
    static uint8_t okFlag = 0;

    if (okFlag == 1)
	{
		if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
		{

			memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
			gGprsPara.currentCmdId = AT_SAPBR1_CMD;
            /*
			memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
			gGprsPara.currentCmdId = AT_ENTER_SLEEP_MODE;*/
			
			okFlag = 0;
		}
	}

    if (strstr(gGprsPara.responseBuffer, "+CGREG:") != NULL)
    {
        if (*(gGprsPara.responseBuffer+strlen(gGprsPara.responseBuffer)-3) == '1' || *(gGprsPara.responseBuffer+strlen(gGprsPara.responseBuffer)-3) == '5')
        {
            okFlag = 1;
            memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        }
        else
        {
            memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
            gAtCmdCenter[AT_CGREG_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
        }
    }
}

static void _at_sapbr1_response_deal(void)
{
    
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_SAPBR2_CMD;
    }
}

static void _at_sapbr2_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_SAPBR3_CMD;
    }
}

static void _at_sapbr3_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_SAPBR4_CMD;
    }
    else if (strstr(gGprsPara.responseBuffer, "ERROR") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gAtCmdCenter[AT_SAPBR3_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
    }
}

static void _at_sapbr4_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_HTTPINIT_CMD;
    }
}

static void _at_httpinit_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_HTTPPARA1_CMD;
    }
}

static void _at_httppara1_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_HTTPPARA2_CMD;
    }
}

static void _at_httppara2_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_HTTPPARA3_CMD;
    }
}

static void _at_httppara3_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_HTTPPARA4_CMD;
    }
}

static void _at_httppara4_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
		memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));

        gGprsPara.currentCmdId = AT_INVALID_CMD;
        gGprsPara.gprsStatus = GPRS_READY;
        /*
        if (GPRS_INITING == gGprsPara.gprsStatus)
        {
            gGprsPara.currentCmdId = AT_INVALID_CMD;
        }
        else
        {
            gGprsPara.currentCmdId = AT_HTTPPOST_PRE_CMD;
        }*/
    }
}

static void _at_httppost_pre_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "DOWNLOAD\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_HTTPPOST_PRE1_CMD;
    }
}

static void _at_httppost_pre1_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_HTTPPPOSTACTION_CMD;
    }
}

static void _at_httppost_action_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "+HTTPACTION: 1") != NULL)
    {
        if (strstr(gGprsPara.responseBuffer, "+HTTPACTION: 1,200") != NULL)
        {
            //how to update fw  TODO
            /*
						memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
            gGprsPara.currentCmdId = AT_HTTPTERM_CMD;*/
            memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
            gGprsPara.currentCmdId = AT_HTTPREAD_CMD;

        }
        else
        {
					  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
            gAtCmdCenter[AT_HTTPPPOSTACTION_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
        }
    }
}

static void _at_httpread_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "+CME ERROR:") != NULL)
    {
        //error
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gAtCmdCenter[AT_HTTPREAD_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
    }
}

static void _at_cfun_enter_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
        gGprsPara.gprsDelayCnt = 50;
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_CFUN1_EXIT_CMD;
    }
}

static void _at_cfun1_exit_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
			  memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_CPIN_SIMCOM_CMD;
    }
}

static void _at_enter_sleep_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
		memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.isGprsSleeping = 1;
        gprs_close();
    }   
}

static void _at_httpterm_response_deal(void)
{
    if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL)
    {
        memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        gGprsPara.currentCmdId = AT_INVALID_CMD;
    }
}
/**********************************************************/


//串口分包发送
//static int uart_Datapacket(char * outData, uint16_t len);
//find string in string, return the first start location or -1 if can not find
static int _string_find(const char *pSrc, const char *pDst)
{
    int i, j;
    for (i = 0; pSrc[i] != '\0'; i++)
    {
        if (pSrc[i] != pDst[0])
            continue;
        j = 0;
        while (pDst[j] != '\0' && pSrc[i + j] != '\0')
        {
            j++;
            if (pDst[j] != pSrc[i + j])
                break;
        }
        if (pDst[j] == '\0')
            return i;
    }
    return -1;
}

// GPRS time-out handler type. */
static void _gprs_timer_handler(void)
{    
    _gprs_cmd_deal(gGprsPara.currentCmdId);  
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&gprsTimerId);

    _gprs_timer_handler();
}


//GPRS 进程处理 定时器发送at指令 两个ID分开
void gprs_response_data_deal(void)
{
    int ret = 0;

    //设备启动
    if (GPRS_SENDING == gGprsPara.gprsStatus || GPRS_INITING == gGprsPara.gprsStatus)
    {
        gAtCmdCenter[gGprsPara.currentCmdId].pRspFunc();

        if (gGprsPara.isGetServerData == 1)
        {
            gGprsPara.isGetServerData = 0;
            if (NULL != gGprsPara.httpResUserFunc)
            {
                ret = gGprsPara.httpResUserFunc(gGprsPara.responseBuffer);
                if (ret < 0)
                {
                    static uint8_t postTimes = 0;
                    if (postTimes >= 10)
                    {
                        postTimes = 0;
                        gGprsPara.currentCmdId = AT_INVALID_CMD;
                        gprs_close();
                        _gprs_establish_connection();
                    }
                    else
                    {   
                        gGprsPara.currentCmdId = AT_HTTPPOST_PRE_CMD; 
                        postTimes++;
                    }
                }
            }
            else
            {
                gGprsPara.currentCmdId = AT_INVALID_CMD;
            }
            memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
        }
    }
}

void gprs_uart_rx_data_deal(uint8_t rxData)
{
    static uint16_t index = 0;
    static uint8_t isStartGetData = 0;
    static char rcvBuf[RESPONSE_DATA_LEN] = {0};
    static char tmpBuf[RESPONSE_DATA_LEN] = {0};
    
    rcvBuf[index++] = rxData;

    if (((rcvBuf[index - 1] == 0x0A && (rcvBuf[index - 2] == 0x0D))) || (index >= sizeof(rcvBuf)))
    {
        memcpy(gGprsPara.responseBuffer, rcvBuf, index); //cpy data to uart buffer
        memset(rcvBuf, 0, strlen((char *)rcvBuf));      

        if (gGprsPara.currentCmdId == AT_HTTPREAD_CMD) //获取到服务器数据
        {
            //
            if (strstr(gGprsPara.responseBuffer, "OK\r\n") != NULL) //停止接收数据
            {
                //stop rev data
                isStartGetData = 0;
                gGprsPara.isGetServerData = 1;
                gGprsPara.currentCmdId = AT_INVALID_CMD;
                memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
                memcpy(gGprsPara.responseBuffer, tmpBuf, strlen(tmpBuf));
            }

            if (isStartGetData == 1)
            {
                memset(tmpBuf, 0, sizeof(tmpBuf));
                memcpy(tmpBuf, gGprsPara.responseBuffer, strlen(gGprsPara.responseBuffer));
                gGprsPara.isGetServerData = 1;
                
                memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
            }
            //
            if (strstr(gGprsPara.responseBuffer, "+HTTPREAD:") != NULL) //接收回来的数据,开始接收数据
            {
                /* code */
                //start rev data
                isStartGetData = 1;
                memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
            }
        }

        if (gGprsPara.currentCmdId == AT_CIMI_CMD)
        {
            if (strstr(gGprsPara.responseBuffer, "\r\n") != NULL)
            {
                if (strlen(gGprsPara.responseBuffer) > 10)
                {
                    memcpy(gGprsPara.gprsInfo.gprs_imsi, gGprsPara.responseBuffer, 15);
                    memset(gGprsPara.responseBuffer, 0, sizeof(gGprsPara.responseBuffer));
                }
            }
        }
        index = 0;
    }
}

static void _gprs_uart_send_data(char *SendData, uint16_t SendLen)
{
    if(HAL_UART_Transmit(&huart2, (uint8_t*)SendData, SendLen,100)!= HAL_OK)
    {
        Error_Handler();
    }
}

//GPRS Parameter init
void gprs_init(httpResponseDeal userFunc)
{
    gGprsPara.httpResUserFunc = userFunc;

    // uart between gprs module and mcu init
    _gprs_uart_init();
    
    // at cmd send, timer is 100ms
    _gprs_uart_send_timer_creat();

    // just connect server, don't send http cmd
    _gprs_establish_connection();
}

int gprs_http_request_send(char * pData)
{
    if (pData == NULL)
    {
        return -1;
    }

    if (GPRS_READY == gGprsPara.gprsStatus)
    {
        memset(gGprsPara.requestBuffer, 0, sizeof(gGprsPara.requestBuffer));
        strcpy(gGprsPara.requestBuffer, pData);
        gGprsPara.gprsStatus = GPRS_SENDING;
        gGprsPara.currentCmdId = AT_HTTPPOST_PRE_CMD;

        return 0;
    }

    return -2;
}

static void _gprs_stop_timer(void)
{
    if (HAL_TIM_Base_Stop_IT(&gprsTimerId) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

int gprs_close(void)
{
    //事件开始
    gGprsPara.currentCmdId = AT_INVALID_CMD; //无效
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_RXNE);
    //100ms start
    _gprs_stop_timer();
    //osTimerStop(GPRS_timer_id);

    _gprs_sleep_ms(50);
    //睡眠的状态下不关闭模块
    if (gGprsPara.isGprsSleeping == 0)
    {
        _gprs_power_off();
        return 0;
    }
    else
    {
        return 1;
    }
}

static void _gprs_uart_init(void)
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

static void _gprs_uart_enable(void)
{
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
}

static void _gprs_power_on(void)
{
    gGprsPara.gprsStatus = GPRS_POWER_ON;
//for formal pcba, not for develop kit
/*
    nrf_gpio_cfg_output(UG_EN_PIN);
    nrf_gpio_cfg_output(UG_PWRKEY);
    //sleep pin
    nrf_gpio_cfg_output(UG_DTR_PIN);
    //
    nrf_gpio_pin_set(UG_EN_PIN);
    _gprs_sleep_ms(200);
    nrf_gpio_pin_set(UG_PWRKEY);*/
}

static void _gprs_timer_start(void)
{
    // start 100ms timer
    if (HAL_TIM_Base_Start_IT(&gprsTimerId) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }

}

static void _gprs_uart_send_timer_creat(void)
{
    uint32_t uwPrescalerValue = 0;

  __HAL_RCC_TIM3_CLK_ENABLE();
  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
      HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);

      /* Enable the TIMx global Interrupt */
      HAL_NVIC_EnableIRQ(TIM3_IRQn);
    
    uwPrescalerValue = 6400 - 1; // 64m / 6400 = 10k

    gprsTimerId.Instance = TIM3;

    gprsTimerId.Init.Period            = 1000 - 1;
    gprsTimerId.Init.Prescaler         = uwPrescalerValue;
    gprsTimerId.Init.ClockDivision     = 0;
    gprsTimerId.Init.CounterMode       = TIM_COUNTERMODE_UP;
    gprsTimerId.Init.RepetitionCounter = 0;

    if (HAL_TIM_Base_Init(&gprsTimerId) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    if (HAL_TIM_Base_Start(&gprsTimerId) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }
}

static void _gprs_establish_connection(void)
{
    _gprs_power_on();
    _gprs_sleep_ms(100);

    // init parameters
    gGprsPara.isGetServerData = 0;
    // just connect server, don't send http cmd
    gGprsPara.gprsStatus = GPRS_INITING;
    gGprsPara.currentCmdId = AT_LINK_CHECK_CMD;

    _gprs_uart_enable();
    _gprs_timer_start();
    
}

static void _gprs_power_off(void)
{
    gGprsPara.gprsStatus = GPRS_POWER_OFF;
/*
    nrf_gpio_pin_set(Green_LED_PIN);    //Green LED OFF
    nrf_gpio_cfg_output(UG_PWEDWN);
    nrf_gpio_cfg_output(UG_EN_PIN);
    nrf_gpio_pin_set(UG_PWEDWN);
    _gprs_sleep_ms(100);
    nrf_gpio_pin_clear(UG_PWEDWN);
    _gprs_sleep_ms(2);
    nrf_gpio_pin_clear(UG_EN_PIN);*/
}

