#include <string.h>
#include "ofo_GPRS.h"
//#include "Cmsis_os.h"
#include "stm32l4xx_hal.h"
#include "json_utils.h"
#include "base64.h"
#include "stm32l4xx_hal_tim.h"


/*
* @Author: liang
* @Date:   2017-03-22 11:29:41
* @Last Modified by:   liangchunyan
* @Last Modified time: 2017-05-22 12:23:12
*/

#define EXECUTE_TIMES 2 //执行次数
#define DelayTime 100 //延时时间
#define MonitorTime GPRS_TIMEROUT*10//监控GPRS通信时间80s
#define Monitor_QUERY_Count 20 //监控网络注册的次数
#define Monitor_HTTPPOST_Count 3//监控post data的次数
#define Monitor_GETCONNECT_Count 10 //监控post长度
#define Monitor_POSTTIME_Count 30*10 //监控POST data 的时间
#define PWD_LIVETIME 60
#define KEY_GROUP_LEN 3

typedef enum{
    AT_TIME_START_RESEVER = 0,
    AT_TIME_COMM_CMDS,// 500ms
    AT_TIME_ENTER_FLYING_OVER,  // 5s
    AT_TIME_TIMER_1MIN,// 1min
    AT_TIME_HTTP_COMM_CMDS, // 10s
    AT_TIME_HTTP_40S, // 40s
}atTimeStart_e;

//AT 相关
uint16_t UrlLen = 0;
//与UG96模块交互的数据
static char *outDataBuffer = NULL;
static uint8_t *data_array = NULL;
static char *inDataBuffer = NULL;

//GPRS发送数据长度
static uint16_t SendDataLen = 0;
/*解析服务器数据*/
//添加密钥
extern void Addkey(const char *keyvalue);
//解析数据
static void ParsedData(char *indata);
//组合数据
static void GPRS_CmbData(void);//GPRS 组合数据
//服务器数据内容

/*监控GPRS模块通信*/
uint8_t  isGPRSPostData = 0; //GPRS工作
uint8_t  isGPRSSleep = 0; //GPRS睡眠
//
static uint8_t Encryptout[50];
//是否启动睡眠秘书
uint8_t isGPRSStartSleep = 0;
uint16_t GPRSSleepTime = 0;

/**************************new frame*******************************/

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

#define GPRS_MAX_COMM_TIME      1800

#define APP_VERSION "ofoV1.0.8"

#define KEY_CODE_LEN 4

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

typedef struct
{
    atCmdId_e atCmdId;
    char *pCmdStr;
    atRequestSendFunc pReqFunc;
    atResponseDealFunc pRspFunc;
    cmdAttribute_t  cmdAttribute;
}atCmdCenter_t;

typedef struct{
    uint16_t gprsDelayCnt;  // 0 is not delay, any others do delay 1 == 100ms
    uint16_t gprsMaxCommTime;   // if gprs has opened over 3mins, whatever, close gprs
    atCmdId_e currentCmdId;
    uint8_t isGetServerData;
}gprsPara_t;

typedef struct
{
	char PWD_Value[KEY_CODE_LEN+1];			//閿佺殑瀵嗙爜
	uint8_t PWD_Valid;					//瀵嗙爜鏄惁鏈夋晥锛?鏈夋晥锛?鏃犳晥
	uint16_t   PWD_LiveTime; 				//瀵嗙爜瀛樻椿鏃堕棿
	uint8_t PWD_isNew; //鏄惁鏄柊瀵嗛挜
} _LockPWD;

typedef struct
{
//device mac address
	char DEV_MacAddr[13];
//device token
	char DEV_Token[16];
//device GPRS CSQ 手机网络信号
	char GPRS_CSQ[3];
//UG96 IMSI
	char GPRS_IMSI[16];
//UG96 SIMVer
	char GPRS_SIMVer[18];
//设备地址
	char DEV_SPAddr[25];
//0  交易数据包  1 心跳数据包  10 文件下载请求数据包
	uint8_t DEV_ACTION;
//设备软件的版本号
	char DEV_MCUVer[10];
//设备电量
	uint16_t DEV_Vot;
//设备3轴信息
	char DEV_3D[25];
//开锁时间
	uint32_t DEV_STTime;
//闭锁时间
	uint32_t DEV_SPTime;
//开锁密码，共1组
	_LockPWD DEV_PWD;
} send_status_t;

/*rev Data status*/
typedef struct
{
//当前时间
	uint32_t SVTime;
	//版本号
	char MCUVer[10];
 //闭锁时间
	uint32_t DEV_SPTime;
//开锁密码，共3组
	_LockPWD DEV_PWD[3];
	//心跳时间
	uint16_t PLUS;
} rev_status_t;

typedef struct
{
	unsigned char BLE_name[20];		  //BLE 骞挎挱鐨勫悕绉?	"ofo_Oversea",
	const char HW[20];            //"HardWare Ver: 1.0.1",
	const char HW_T[0x20];          //__TIME__ __DATE__,
	unsigned char Lock_serialnum[20];  //閿佺殑搴忓垪鍙?	"HW NO :000002",
	unsigned char BLE_secretkey[20];	  //BLE杩炴帴瀵嗛挜 "0123456789012345",	
	unsigned int UDID_key;	  //UUID瀵嗛挜鍔犻€?	0xffffffff
	unsigned char isWrite[4];    //鏄惁宸茬粡鍐?
} _Dev_Parm;

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

static void _gprs_start(void);
static void MX_USART2_UART_Init(void);
static void gprs_timer_handler(void);
static int StringFind(const char *pSrc, const char *pDst);


static void ofo_sleep_ms(int timeMs);

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef gprsTimerId;
/* Private function prototypes -----------------------------------------------*/

// gprs send timer init
//APP_TIMER_DEF(GPRS_timer_id);
/*
osTimerDef(gprs_timer, GPRS_timer_handler);
osTimerId GPRS_timer_id;*/

UART_HandleTypeDef huart2;

static gprsPara_t gGprsPara = {0, GPRS_MAX_COMM_TIME, AT_INVALID_CMD, 0};
_LockPWD LockPWD[KEY_GROUP_LEN] = {0};

_Dev_Parm dev_Parm = {
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //"ofo_Oversea",
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //"Titania V1.0",
	__TIME__ __DATE__,
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //"HW NO :000002",
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //"0123456789012345",
	0xffffffff,
	{0x00, 0x00, 0x00, 0x00}
};

send_status_t SendStatus={{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//DEV_MacAddr
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//DEV_Token
{0x00,0x00,0x00},//GPRS_CSQ
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//GPRS_IMSI
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//GPRS_SIMVer
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//DEV_SPAddr
0x00,//DEV_ACTION
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//DEV_MCUVer
0x00,//DEV_Vot
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//DEV_3D
1483228800,//DEV_STTime
1483228800,//DEV_SPTime
{{0x00,0x00,0x00,0x00,0x00},//PWD_Value
 0x00,//PWD_Valid
 300,//PWD_LiveTime
 0x00//PWD_isNew
}//	DEV_PWD
};

rev_status_t RevStatus={1483228800,//SVTime
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//	MCUVer
1483228800,//DEV_SPTime
//DEV_PWD
{{{0x00,0x00,0x00,0x00,0x00},//PWD_Value
 0x00,//PWD_Valid
 300,//PWD_LiveTime
 0x00//PWD_isNew
},
{{0x00,0x00,0x00,0x00,0x00},//PWD_Value
 0x00,//PWD_Valid
 300,//PWD_LiveTime
 0x00//PWD_isNew
},
{{0x00,0x00,0x00,0x00,0x00},//PWD_Value
 0x00,//PWD_Valid
 300,//PWD_LiveTime
 0x00//PWD_isNew
}},
0//PLUS
};

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


static void ofo_sleep_ms(int timeMs)
{
    HAL_Delay(timeMs);
}

static void _at_config_http_content_len(void)
{
    //led_flash(Hal_Led_Green , 100 , 30);
    GPRS_CmbData();
    SendDataLen = strlen(outDataBuffer);
    if (outDataBuffer != NULL)
    {
        free(outDataBuffer);
        outDataBuffer = NULL;
    }
    
    sprintf(inDataBuffer, "%s%d,10000\r\n", gAtCmdCenter[AT_HTTPPOST_PRE_CMD].pCmdStr, SendDataLen);
    GPRS_SendData(inDataBuffer, strlen(inDataBuffer));
    memset(inDataBuffer, 0, inBufferLen);
}

static void _at_send_post_data_to_module(void)
{
    //combine data
    GPRS_CmbData();
    //串口分包发送
    GPRS_SendData(outDataBuffer, strlen(outDataBuffer));//发数据内容
    //释放
    if (outDataBuffer != NULL)
    {
        free(outDataBuffer);
        outDataBuffer = NULL;
    }
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
                GPRS_uart_close(1);
                _gprs_start();
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
            GPRS_SendData(gAtCmdCenter[cmdId].pCmdStr, strlen(gAtCmdCenter[cmdId].pCmdStr));
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

static void _light_control_during_gprs(void)
{
    //static uint8_t greenLedFlag = 0;

/*
    if (greenLedFlag == 1)
    {
        nrf_gpio_pin_clear(Green_LED_PIN);//Green LED On
        greenLedFlag = 0;
    }
    else
    {
        nrf_gpio_pin_set(Green_LED_PIN);    //Green LED OFF
        greenLedFlag = 1;
    }*/
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
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
				memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_CLOSE_ECHO_CMD;
        //nrf_gpio_pin_clear(UG_PWRKEY);
    }
}

static void _at_close_echo_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_GSV_CMD;
    }
}

static void _at_gsv_response_deal(void)
{
    if (strstr(inDataBuffer, "Revision:") != NULL) //存在该字段
    {
        int strindex = StringFind(inDataBuffer, "Revision:");
        if (strindex >= 0)
        {
            strindex += strlen("Revision: ");
            memcpy(SendStatus.GPRS_SIMVer, inDataBuffer + strindex, 16);
					  memset(inDataBuffer, 0, inBufferLen);
            gGprsPara.currentCmdId = AT_CPIN_SIMCOM_CMD;
        }
        else
				{
						memset(inDataBuffer, 0, inBufferLen);
				}
    }
}

static void _at_cpin_response_deal(void)
{
		static uint8_t okFlag = 0;
	
		if (1 == okFlag)
		{
			if (strstr(inDataBuffer, "OK\r\n") != NULL)
			{
				memset(inDataBuffer, 0, inBufferLen);
				gGprsPara.currentCmdId = AT_CIMI_CMD; 
			}
		}
	
    if (strstr(inDataBuffer, "+CPIN:") != NULL)
    {
        if (strstr(inDataBuffer, "+CPIN: READY\r\n") != NULL) 
        {
            //gGprsPara.currentCmdId = AT_CIMI_CMD; 
						okFlag = 1;
        }
        else
        {
            gAtCmdCenter[AT_CPIN_SIMCOM_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
        }
        memset(inDataBuffer, 0, inBufferLen);
    }
}

static void _at_cimi_cmd_reponse_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
	  {
	    memset(inDataBuffer, 0, inBufferLen);
		  gGprsPara.currentCmdId = AT_CSQ_SIMCOM_CMD;
	  }
}

static void _at_csq_response_deal(void)
{
    if (strstr(inDataBuffer, "+CSQ:") != NULL)
    {
        int strindex = StringFind(inDataBuffer, "+CSQ: ");
        strindex += strlen("+CSQ: ");
        SendStatus.GPRS_CSQ[0] = *(inDataBuffer + strindex);
        if (*(inDataBuffer + strindex + 1) != ',')
        {
            SendStatus.GPRS_CSQ[1] = *(inDataBuffer + strindex + 1);
        }
        
        if (strcmp(SendStatus.GPRS_CSQ, "0") == 0)
        {
					  memset(inDataBuffer, 0, inBufferLen);
            gAtCmdCenter[AT_CSQ_SIMCOM_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
        }
        else
        {
					  memset(inDataBuffer, 0, inBufferLen);
            gGprsPara.currentCmdId = AT_CGREG_CMD;
        }
    }
}

static void _at_cgreg_response_deal(void)
{
    static uint8_t okFlag = 0;

    if (okFlag == 1)
	{
		if (strstr(inDataBuffer, "OK\r\n") != NULL)
		{
			if (isGPRSSleep == 0)
			{
				memset(inDataBuffer, 0, inBufferLen);
				gGprsPara.currentCmdId = AT_SAPBR1_CMD;
			}
			else
			{
				memset(inDataBuffer, 0, inBufferLen);
				gGprsPara.currentCmdId = AT_ENTER_SLEEP_MODE;
			}
			okFlag = 0;
		}
	}

    if (strstr(inDataBuffer, "+CGREG:") != NULL)
    {
        if (*(inDataBuffer+strlen(inDataBuffer)-3) == '1' || *(inDataBuffer+strlen(inDataBuffer)-3) == '5')
        {
            okFlag = 1;
            memset(inDataBuffer, 0, inBufferLen);
        }
    }
}

static void _at_sapbr1_response_deal(void)
{
    
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_SAPBR2_CMD;
    }
}

static void _at_sapbr2_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_SAPBR3_CMD;
    }
}

static void _at_sapbr3_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_SAPBR4_CMD;
    }
    else if (strstr(inDataBuffer, "ERROR") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gAtCmdCenter[AT_SAPBR3_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
    }
}

static void _at_sapbr4_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_HTTPINIT_CMD;
    }
}

static void _at_httpinit_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_HTTPPARA1_CMD;
    }
}

static void _at_httppara1_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_HTTPPARA2_CMD;
    }
}

static void _at_httppara2_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_HTTPPARA3_CMD;
    }
}

static void _at_httppara3_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_HTTPPARA4_CMD;
    }
}

static void _at_httppara4_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_HTTPPOST_PRE_CMD;
    }
}

static void _at_httppost_pre_response_deal(void)
{
    if (strstr(inDataBuffer, "DOWNLOAD\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_HTTPPOST_PRE1_CMD;
    }
}

static void _at_httppost_pre1_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_HTTPPPOSTACTION_CMD;
    }
}

static void _at_httppost_action_response_deal(void)
{
    if (strstr(inDataBuffer, "+HTTPACTION: 1") != NULL)
    {
        if (strstr(inDataBuffer, "+HTTPACTION: 1,200") != NULL)
        {
            if (SendStatus.DEV_ACTION == 10)
            {
                //how to update fw  TODO
							memset(inDataBuffer, 0, inBufferLen);
                gGprsPara.currentCmdId = AT_HTTPTERM_CMD;
            }
            else
            {
							  memset(inDataBuffer, 0, inBufferLen);
                gGprsPara.currentCmdId = AT_HTTPREAD_CMD;
            }
        }
        else
        {
					  memset(inDataBuffer, 0, inBufferLen);
            gAtCmdCenter[AT_HTTPPPOSTACTION_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
        }
    }
}

static void _at_httpread_response_deal(void)
{
    if (strstr(inDataBuffer, "+CME ERROR:") != NULL)
    {
        //error
			  memset(inDataBuffer, 0, inBufferLen);
        gAtCmdCenter[AT_HTTPREAD_CMD].cmdAttribute.atCmdExecResult = CMD_RESULT_FAILED;
    }
}

static void _at_cfun_enter_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
        gGprsPara.gprsDelayCnt = 50;
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_CFUN1_EXIT_CMD;
    }
}

static void _at_cfun1_exit_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_CPIN_SIMCOM_CMD;
    }
}

static void _at_enter_sleep_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
			  memset(inDataBuffer, 0, inBufferLen);
        GPRS_uart_close(0);
    }   
}

static void _at_httpterm_response_deal(void)
{
    if (strstr(inDataBuffer, "OK\r\n") != NULL)
    {
        memset(inDataBuffer, 0, inBufferLen);
        gGprsPara.currentCmdId = AT_INVALID_CMD;
    }
}
/**********************************************************/


//串口分包发送
//static int uart_Datapacket(char * outData, uint16_t len);
//find string in string, return the first start location or -1 if can not find
static int StringFind(const char *pSrc, const char *pDst)
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
static void gprs_timer_handler(void)
{
    gGprsPara.gprsMaxCommTime--;
    if (0 == gGprsPara.gprsMaxCommTime)
    {
        GPRS_uart_close(0);
        return;
    }

    _light_control_during_gprs();
    
    _gprs_cmd_deal(gGprsPara.currentCmdId);  
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&gprsTimerId);

    gprs_timer_handler();
}


//GPRS 进程处理 定时器发送at指令 两个ID分开
void gprs_response_data_deal(void)
{
    //设备启动
    if (isGPRSPostData == 1 || isGPRSSleep == 1)
    {
        gAtCmdCenter[gGprsPara.currentCmdId].pRspFunc();

        if (gGprsPara.isGetServerData == 1)
        {
            gGprsPara.isGetServerData = 0;
            ParsedData(outDataBuffer);
            if (outDataBuffer != NULL)
            {
                free(outDataBuffer);
                outDataBuffer = NULL;
            }
        }
    }
}

//添加密码
void Addkey(const char *keyvalue)
{    
    sscanf(keyvalue, "%04s,%04s,%04s", LockPWD[0].PWD_Value, LockPWD[1].PWD_Value, LockPWD[2].PWD_Value); //KEY
    
    for(uint8_t i=0;i<3;i++)
    {
        LockPWD[i].PWD_LiveTime = PWD_LIVETIME;
            LockPWD[i].PWD_Valid = 1;
            LockPWD[i].PWD_isNew = 1;
    }
}

void uart_rx_data_deal(uint8_t rxData)
{
    static uint16_t index = 0;
    static uint8_t isStartGetData = 0;
    
    data_array[index++] = rxData;

    if (((data_array[index - 1] == 0x0A && (data_array[index - 2] == 0x0D))) || (index >= outBufferLen))
    {
        memcpy(inDataBuffer, data_array, index); //cpy data to uart buffer
        memset(data_array, 0, strlen((char *)data_array));      

        //接收到服务器数据
        if (gGprsPara.currentCmdId == AT_HTTPREAD_CMD) //获取到服务器数据
        {
            //
            if (strstr(inDataBuffer, "OK\r\n") != NULL) //停止接收数据
            {
                //stop rev data
                isStartGetData = 0;
                memset(inDataBuffer, 0, inBufferLen);
            }
            //开始接收服务器数据
            if (isStartGetData == 1)
            {
//              //解析服务器数据
                outDataBuffer = (char *)malloc(inBufferLen);
                if (outDataBuffer != NULL)
                {
                    memcpy(outDataBuffer, inDataBuffer, strlen(inDataBuffer));
                    gGprsPara.isGetServerData = 1;
                }
                //事件
                gGprsPara.currentCmdId = AT_INVALID_CMD; //无效
                memset(inDataBuffer, 0, inBufferLen);
            }
            //
            if (strstr(inDataBuffer, "+HTTPREAD:") != NULL) //接收回来的数据,开始接收数据
            {
                /* code */
                //start rev data
                isStartGetData = 1;
                memset(inDataBuffer, 0, inBufferLen);
            }
        }
        //GET IMSI
        if (gGprsPara.currentCmdId == AT_CIMI_CMD)
        {
            if (strstr(inDataBuffer, "\r\n") != NULL)
            {
                //检测到版本号
                if (strlen(inDataBuffer) > 10)
                {
                    memcpy(SendStatus.GPRS_IMSI, inDataBuffer, 15);
                    memset(inDataBuffer, 0, inBufferLen);
                }
            }
        }
        index = 0;
    }
}


//GRPS 串口发送数据
void GPRS_SendData(char *SendData, uint16_t SendLen)
{
    if(HAL_UART_Transmit(&huart2, (uint8_t*)SendData, SendLen,100)!= HAL_OK)
    {
        Error_Handler();
    }
}
//  static uint8_t kk = 0;
////串口分包发送
//int uart_Datapacket(char * outData, uint16_t len)
//{
//  uint8_t Packets = 0;
//  uint16_t Tail = 0;

//  Packets = len / 256;

//  if (kk < Packets)
//  {
////        printf("kk is:%d,Packet is %d\r\n",kk,Packets);
//      GPRS_SendData(outData + kk * GPRS_UART_TX_BUF_SIZE, GPRS_UART_TX_BUF_SIZE);
//  }
//  else
//  {
//      Tail = len - (Packets * GPRS_UART_TX_BUF_SIZE);
//      if (Tail > 0)
//      {
////            printf("Tail is:%d\r\n",Tail);
//          GPRS_SendData(outData + Packets * GPRS_UART_TX_BUF_SIZE, Tail);
//      }
//      return 1;
//  }
//  //
//  kk += 1;
//  //
//  return -1;
//}
//GPRS Parameter init
void GPRS_Parameter_init(void)
{
    _gprs_uart_send_timer_creat();

}

static void _gprs_stop_timer(void)
{
    if (HAL_TIM_Base_Stop_IT(&gprsTimerId) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

//关闭串口通信
void GPRS_uart_close(uint8_t isCloseGPRS)
{
    //事件开始
    gGprsPara.currentCmdId = AT_INVALID_CMD; //无效
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_RXNE);
    //100ms start
    _gprs_stop_timer();
    //osTimerStop(GPRS_timer_id);

    ofo_sleep_ms(50);
    //睡眠的状态下不关闭模块
    if (isGPRSSleep == 0 || isCloseGPRS == 1)
        GPRS_close();
    
    //释放接收缓冲区内存
    if (inDataBuffer != NULL)
    {
        free(inDataBuffer);
        inDataBuffer = NULL;
    }
    if (data_array != NULL)
    {
        free(data_array);
        data_array = NULL;
    }
    if (outDataBuffer != NULL)
    {
        free(outDataBuffer);
        outDataBuffer = NULL;
    }
}

/* USART1 init function */
static void MX_USART2_UART_Init(void)
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
    MX_USART2_UART_Init();
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
}

void gprs_power_on(void)
{
//for formal pcba, not for develop kit
/*
    nrf_gpio_cfg_output(UG_EN_PIN);
    nrf_gpio_cfg_output(UG_PWRKEY);
    //sleep pin
    nrf_gpio_cfg_output(UG_DTR_PIN);
    //
    nrf_gpio_pin_set(UG_EN_PIN);
    ofo_sleep_ms(200);
    nrf_gpio_pin_set(UG_PWRKEY);*/
}

static void _gprs_timer_start(void)
{
    inDataBuffer = (char *)malloc(inBufferLen);
    data_array = (uint8_t *)malloc(inBufferLen);
    if (inDataBuffer != NULL && data_array != NULL)
    {
        // start 100ms timer
        if (HAL_TIM_Base_Start_IT(&gprsTimerId) != HAL_OK)
          {
            /* Starting Error */
            Error_Handler();
          }
        //osTimerStart(GPRS_timer_id, 100);
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


//打开串口通信
/**@snippet [UART Initialization] */
static void _gprs_start(void)
{
    GPRS_uart_close(0);
    
    gprs_power_on();
    ofo_sleep_ms(100);
    _gprs_uart_enable();
    _gprs_timer_start();

    // init parameters
    gGprsPara.isGetServerData = 0;
    gGprsPara.currentCmdId = AT_LINK_CHECK_CMD;
    gGprsPara.gprsMaxCommTime = GPRS_MAX_COMM_TIME;
}


#if 0
static void _get_axes_raw(void)
{
    AxesRaw_t getdata;
    int_least8_t temp3D[3];

    
    LIS3DH_GetAccAxesRaw(&getdata);
    temp3D[0] = (getdata.AXIS_X) >> 4;
    temp3D[1] = (getdata.AXIS_Y) >> 4;
    temp3D[2] = (getdata.AXIS_Z) >> 4;
    sprintf(SendStatus.DEV_3D, "%d,%d,%d", temp3D[0], temp3D[1], temp3D[2]);
}
#endif

void gprs_start_communicate(void)
{
    _gprs_start();
    //_get_axes_raw();

    //开始监听通信，80s超时
    isGPRSPostData = 1;
    isGPRSSleep = 0;
    //睡眠启动参数置0
    isGPRSStartSleep = 0;
    GPRSSleepTime = 0;
}
//开始进入睡眠
void GPRS_StartSleep(void)
{
    //开始进入睡眠
    isGPRSSleep = 1;
    GPRSSleepTime = 0;
    //
    isGPRSStartSleep = 1;
    _gprs_start();
}

//GPRS 关
void GPRS_close(void)
{
/*
    nrf_gpio_pin_set(Green_LED_PIN);    //Green LED OFF
    nrf_gpio_cfg_output(UG_PWEDWN);
    nrf_gpio_cfg_output(UG_EN_PIN);
    nrf_gpio_pin_set(UG_PWEDWN);
    ofo_sleep_ms(100);
    nrf_gpio_pin_clear(UG_PWEDWN);
    ofo_sleep_ms(2);
    nrf_gpio_pin_clear(UG_EN_PIN);*/
}
/**
 * 比较版本号
 *
 * @param v1 第一个版本号
 * @param v2 第二个版本号
 *
 * @return 如果版本号相等，返回 0,
 *         如果第一个版本号低于第二个，返回 -1，否则返回 1.
 */
static int compareVersion(const char *v1, const char *v2)
{
    const char *p_v1 = v1;
    const char *p_v2 = v2;

    while (*p_v1 && *p_v2) {
        char buf_v1[32] = {0};
        char buf_v2[32] = {0};
        char *i_v1 = strchr(p_v1, '.');
        char *i_v2 = strchr(p_v2, '.');

        if (!i_v1 || !i_v2) break;

        if (i_v1 != p_v1) {
            strncpy(buf_v1, p_v1, i_v1 - p_v1);
            p_v1 = i_v1;
        }
        else
            p_v1++;

        if (i_v2 != p_v2) {
            strncpy(buf_v2, p_v2, i_v2 - p_v2);
            p_v2 = i_v2;
        }
        else
            p_v2++;
        int order = atoi(buf_v1) - atoi(buf_v2);
        if (order != 0)
            return order < 0 ? -1 : 1;
    }

    double res = atof(p_v1) - atof(p_v2);

    if (res < 0) return -1;
    if (res > 0) return 1;
    return 0;
}

//解析服务器数据
void ParsedData(char* indata)
{
    int keyValueIndex = 0;
    int keyLength = 0;
    int valueLength = 0;
    int i = 0;
    int startKeyValueFlag = 0;
    int mesureKeyLengthFlag = 0;
    char key[16] = {0};
    char key_value[32] = {0};
    char plus_value[32] = {0};
    char version_value[32] = {0};
    char time_value[32] = {0};
    int version_len = 0;
    //int key_len = 0;
    int parse_total = 1;
    uint8_t oldVer[10] = {0};
  uint8_t newVer[10] = {0};
    while(indata[i] != '\0'){
        if(indata[i] == '"' && startKeyValueFlag == 0) 
        {
            startKeyValueFlag = 1;
        }
        if(startKeyValueFlag){  //start to parse key-value pair
            mesureKeyLengthFlag = 1;
            keyValueIndex = 0;
            keyLength = 0;      
            valueLength = 0;
            while(1)
            {
                if(indata[i+keyValueIndex] == '"' && (indata[i+keyValueIndex+1] == ',' || indata[i+keyValueIndex+ 1] == '}'))
                    break;
                if(mesureKeyLengthFlag && indata[i+keyValueIndex] == '"')
                {  //measure length of key
                    keyValueIndex++;
                }
                if(indata[i+keyValueIndex] == ':')
                {  //measure length of value
                    mesureKeyLengthFlag = 0;
                }
                if(mesureKeyLengthFlag == 1)
                {
                    keyLength++;
                }
                else
                {
                    valueLength++;
                }
                keyValueIndex++;
            }
            copyString(key, &indata[i+1], keyLength);
            valueLength = valueLength -2;
            if(!strcmp(key, "SVTime"))
            {
                memset(time_value,0,32);
                memcpy(time_value,&indata[i+keyLength+4],valueLength);
                parse_total = parse_total << 1;
            } else if (!strcmp(key, "MCUVer")) {
                memset(version_value,0,32);
                memcpy(version_value,&indata[i+keyLength+4],valueLength);
                version_len = valueLength;
                parse_total = parse_total << 1;
            } else if (!strcmp(key, "KEY")) {
                memset(key_value,0,32);
                memcpy(key_value,&indata[i+keyLength+4],valueLength);
                //key_len = valueLength;
                parse_total = parse_total << 1;
            } else if (!strcmp(key, "PLUS")) {
                memset(plus_value,0,32);
                memcpy(plus_value,&indata[i+keyLength+4],valueLength);
                parse_total = parse_total << 1;
            }

            if(parse_total == 0x10)
            {
                //SetAppTimer(atoi(time_value));
                memcpy(RevStatus.MCUVer,version_value,version_len);
                RevStatus.PLUS = atoi(plus_value); //PLUS
                //SavePassWord(key_value, key_len);
                GPRS_uart_close(0);
                memcpy(oldVer,APP_VERSION+4,5);
                memcpy(newVer,RevStatus.MCUVer+4,5);

                if (compareVersion((char *)oldVer, (char *)newVer) < 0)
                {
                    printf("update ver ");
                    ofo_sleep_ms(1000);

                    SendStatus.DEV_ACTION = 10;  //20170612
                    gprs_start_communicate();  //GPRS

                }
                break;
            }
            i += keyValueIndex+2;
            startKeyValueFlag = 0;
        }
        else
        {
            i++;    
        }
    }
    if(!i)
    {
        GPRS_SendData("my_object got error as json str\r\n", strlen("my_object got error as json str\r\n"));
    } else if(parse_total != 0x10) {
        static char postTimes = 1;

        // if get wrong data retry the http communicating flow 10 times
        if (postTimes >= 10)
        {
            postTimes = 0;
            gGprsPara.currentCmdId = AT_INVALID_CMD;
            GPRS_uart_close(0);
        }
        else
        {   
            gGprsPara.currentCmdId = AT_HTTPPOST_PRE_CMD; 
            postTimes++;
        }
    }
}
//GPRS POll
//void
//
//加密信息
void Encrypted(void)
{
    unsigned char temp[50];
//
    SafeData data;
    //nrf_ecb_init();
    //nrf_ecb_set_key(dev_Parm.BLE_secretkey);

//  DBD9CFF511C6
	/*
    data.mac[0] = (((NRF_FICR->DEVICEADDR[1]) & 0xff00) | 0xc000) >> 8;
    data.mac[1] = (((NRF_FICR->DEVICEADDR[1]) & 0x00ff));
    data.mac[2] =  ((NRF_FICR->DEVICEADDR[0]) & 0xff000000) >> 24;
    data.mac[3] =  ((NRF_FICR->DEVICEADDR[0]) & 0xff0000) >> 16;
    data.mac[4] =  ((NRF_FICR->DEVICEADDR[0]) & 0xff00) >> 8;
    data.mac[5] =  ((NRF_FICR->DEVICEADDR[0]) & 0xff);*/

#ifdef ServerTest
    data.mac[0] = 0xDF;
    data.mac[1] = 0x95;
    data.mac[2] = 0x1C;
    data.mac[3] = 0xA1;
    data.mac[4] = 0x35;
    data.mac[5] = 0x91;
//  DF951CA13591
#endif

    data.ser[0] =  0;
    data.ser[1] =  0;
    data.time    =  SendStatus.DEV_STTime;
    memset(Encryptout, 0, 50);
    memcpy(Encryptout, &data, sizeof(data));
    Encryptout[12] = 0x31;
    Encryptout[13] = 0x31;
    Encryptout[14] = 0x31;
    Encryptout[15] = 0x31;
    memset(temp, 0, 50);
    //nrf_ecb_crypt(temp, Encryptout);
    memset(Encryptout, 0, 16);
    Base64encode((char *)Encryptout, temp, 16);
}

/**@snippet [UART Initialization] */
//GPRS Combination Data
void GPRS_CmbData(void)
{
    char temp[10] = {0};
    int datalen = 0;
    char dataBasePtr[320] = {0};
    OfoJsonCmbData(1, dataBasePtr, &datalen,"ID",  SendStatus.DEV_MacAddr);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"Token", (char *)dev_Parm.BLE_secretkey);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"CSQ", SendStatus.GPRS_CSQ);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"IMSI", SendStatus.GPRS_IMSI);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"SIMVer", SendStatus.GPRS_SIMVer);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"SPAddr", SendStatus.DEV_SPAddr);
    memset(temp, 0, 10);
    sprintf(temp, "%d", SendStatus.DEV_ACTION);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"ACTION", temp);

    OfoJsonCmbData(0, dataBasePtr, &datalen,"MCUVer",APP_VERSION);
    memset(temp, 0, 10);
    sprintf(temp, "%d", SendStatus.DEV_Vot);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"Vot",temp);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"3D", SendStatus.DEV_3D);
    Encrypted();
    OfoJsonCmbData(0, dataBasePtr, &datalen,"sa",(char *)Encryptout);
    memset(temp, 0, 10);
    sprintf(temp, "%d", SendStatus.DEV_STTime);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"STTime", temp);
    memset(temp, 0, 10);
    sprintf(temp, "%d", SendStatus.DEV_SPTime);
    OfoJsonCmbData(0, dataBasePtr, &datalen,"SPTime", temp);
    OfoJsonCmbData(2, dataBasePtr, &datalen,"PWD", SendStatus.DEV_PWD.PWD_Value);
    outDataBuffer = (char *)malloc(outBufferLen);

    if (outDataBuffer != NULL)
    {
        //just for test
        memset(dataBasePtr, 0, sizeof(dataBasePtr));
        strcpy(dataBasePtr, 
        "{\"ID\":\"FBDE94A7AB6C\",\"Token\":\"0123456789012345\",\"CSQ\":\"16\",\"IMSI\":\"\",\"SIMVer\":\"418B03SIM868M32\",\"SPAddr\":\"\",\"ACTION\":\"0\",\"MCUVer\":\"ofoV1.0.8\",\"Vot\":\"7393\",\"3D\":\"0,0,0\",\"sa\":\"wljeFnnIzYSOpDACLsUzFw==\",\"STTime\":\"1483228800\",\"SPTime\":\"1483228800\",\"PWD\":\"\"}");
        
        #if 0
        datalen = strlen(dataBasePtr);
        sprintf(outDataBuffer, "POST %s HTTP/1.1\r\nHost: %s\r\nConnection: keep-alive\r\nContent-Length: %d\r\n", Remote_Post, Remote_Host, datalen);
        //combine Cache-Control: no-cache
        strcat(outDataBuffer, "Cache-Control: no-cache\r\n");
        strcat(outDataBuffer, "Content-Type: application/json\r\n");
        strcat(outDataBuffer, "Accept: */*\r\n");
//          strcat(outDataBuffer, "Accept-Encoding: gzip, deflate\r\n");
        strcat(outDataBuffer, "Accept-Language: zh-CN,zh;q=0.8\r\n");
        strcat(outDataBuffer, "\r\n");
        #endif
        
        strcat(outDataBuffer, dataBasePtr);
    }

}


