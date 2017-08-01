#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"
#include "ofo_porting.h"
#include <string.h>

typedef void (*uartRcvDeal)(uint8_t);
typedef void (*timerHandler)(void);

typedef struct{
    uartRcvDeal gpsUartRcvDeal;
    uartRcvDeal gprsUartRcvDeal;
    timerHandler gprsTimerHandler;
}ofo_porting_para_t;

static UART_HandleTypeDef huart2;
static TIM_HandleTypeDef gprsTimerId;

ofo_porting_para_t gOfoPortingPara;

void ofoP_sleep_ms(int timeMs)
{
    HAL_Delay(timeMs);
}

// TODO: in this test the gprs and gps used the same uart
void ofoP_gprs_uart_enable(void)
{
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
}

void ofoP_gprs_uart_disable(void)
{
    __HAL_UART_DISABLE_IT(&huart2,UART_IT_RXNE);
}

void ofoP_gprs_uart_init(void *pUartRcvDeal)
{
    gOfoPortingPara.gprsUartRcvDeal = (uartRcvDeal)pUartRcvDeal;

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

void ofoP_gprs_uart_send(char *SendData, uint16_t SendLen)
{
    if(HAL_UART_Transmit(&huart2, (uint8_t*)SendData, SendLen,100)!= HAL_OK)
    {
        Error_Handler();
    }
}

void ofoP_gps_uart_enable(void)
{
    //actually, in the reality pcb, gps and gprs use different uart
    ofoP_gprs_uart_enable();
}

void ofoP_gps_uart_disable(void)
{
    ofoP_gprs_uart_disable();
}

void ofoP_gps_uart_init(void *pUartRcvDeal)
{
    gOfoPortingPara.gpsUartRcvDeal = (uartRcvDeal)pUartRcvDeal;

    ofoP_gprs_uart_init(NULL);
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART2_IRQHandler(void)
{
    static uint8_t RxBuf;
    
  /* USER CODE BEGIN USART1_IRQn 0 */
   if((USART2->ISR&UART_FLAG_ORE) != 0)
    { 
        USART2->ICR = UART_CLEAR_OREF;
        /* do something */
        
    }
    if((USART2->ISR&UART_FLAG_RXNE) != 0)
    { 
            RxBuf = USART2->RDR;

            // TODO:only one is not null, the code just for test
            if (gOfoPortingPara.gprsUartRcvDeal != NULL)
            {
                gOfoPortingPara.gprsUartRcvDeal(RxBuf);
            }

            if (gOfoPortingPara.gpsUartRcvDeal != NULL)
            {
                gOfoPortingPara.gpsUartRcvDeal(RxBuf);
            }
    }
}

void ofoP_gprs_uart_send_timer_creat(void *pTimerHandle, uint16_t IntervalMs)
{
    uint32_t uwPrescalerValue = 0;

    gOfoPortingPara.gprsTimerHandler = (timerHandler)pTimerHandle;

  __HAL_RCC_TIM3_CLK_ENABLE();
  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
      HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);

      /* Enable the TIMx global Interrupt */
      HAL_NVIC_EnableIRQ(TIM3_IRQn);
    
    uwPrescalerValue = 6400 - 1; // 64m / 6400 = 10k

    gprsTimerId.Instance = TIM3;

    gprsTimerId.Init.Period            = IntervalMs*10 - 1; //IntervalMs*10k/1000 - 1
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

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&gprsTimerId);

    if (gOfoPortingPara.gprsTimerHandler != NULL)
    {
        gOfoPortingPara.gprsTimerHandler();
    }
}

void ofoP_gprs_timer_start(void)
{
    if (HAL_TIM_Base_Start_IT(&gprsTimerId) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }

}

void ofoP_gprs_stop_timer(void)
{
    if (HAL_TIM_Base_Stop_IT(&gprsTimerId) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}


void ofoP_gps_power_on(void)
{
    // TODO:just for testing on develop kit
    ofoP_gprs_uart_send("AT+CGNSPWR=1\r\n", strlen("AT+CGNSPWR=1\r\n"));
    ofoP_sleep_ms(1000);
    ofoP_gprs_uart_send("AT+CGPSINF=32\r\n", strlen("AT+CGPSINF=32\r\n"));
}

void ofoP_gps_power_off(void)
{
    
}

void ofoP_gprs_power_on(void)
{
    
}

void ofoP_gprs_power_off(void)
{
    
}


