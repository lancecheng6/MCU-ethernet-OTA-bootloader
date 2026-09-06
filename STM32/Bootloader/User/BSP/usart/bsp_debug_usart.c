/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   USART1 debug port driver, printf redirect, interrupt receive mode
  ******************************************************************************
  * @attention
  *
  * Platform: Wildfire STM32F746 Development Board
  * Forum:    http://www.firebbs.cn
  * Shop:     http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
  
#include "bsp_debug_usart.h"

UART_HandleTypeDef UartHandle;
extern uint8_t ucTemp;  
 /**
  * @brief  DEBUG_USART GPIO configuration, mode: 115200 8-N-1
  * @param  None
  * @retval None
  */  
void DEBUG_USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;
        
    DEBUG_USART_RX_GPIO_CLK_ENABLE();
    DEBUG_USART_TX_GPIO_CLK_ENABLE();
    
    /* Configure USART1 clock source */
		RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
		RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);
    /* Enable USART1 clock */
    DEBUG_USART_CLK_ENABLE();

    /* Configure Tx pin as alternate function push-pull */
    GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = DEBUG_USART_TX_AF;
    HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);
    
    /* Configure Rx pin as alternate function push-pull */
    GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
    GPIO_InitStruct.Alternate = DEBUG_USART_RX_AF;
    HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct); 
    
    /* Configure DEBUG_USART mode */
    UartHandle.Instance = DEBUG_USART;
    UartHandle.Init.BaudRate = 115200;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.Mode = UART_MODE_TX_RX;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    UartHandle.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&UartHandle);

}


/*****************  Send string **********************/
void Usart_SendString(uint8_t *str)
{
	unsigned int k=0;
  do 
  {
      HAL_UART_Transmit( &UartHandle,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
  
}
/// Redirect C library function printf to DEBUG_USART for printf output
int fputc(int ch, FILE *f)
{
	/* Send one byte of data to DEBUG_USART */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	
	return (ch);
}

/// Redirect C library function scanf to DEBUG_USART for input using scanf, getchar, etc.
int fgetc(FILE *f)
{
		
	int ch;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}

/**
  * @brief  Non-blocking UART key read. Returns ASCII key or 0 if none.
  *         Polls RXNE flag (no interrupt, does not block LwIP loop).
  */
int UART_ReadKey_NonBlock(void)
{
    uint8_t ch;

    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET)
    {
        if (HAL_UART_Receive(&UartHandle, &ch, 1, 0) == HAL_OK)
            return (int)ch;
    }
    return 0;
}

/**
  * @brief  Blocking UART key read (waits forever for one char).
  *         Used for user confirmation interaction (y/n/ESC) -- no timeout, waits for user input.
  */
int UART_ReadKey_Blocking(void)
{
    uint8_t ch;
    if (HAL_UART_Receive(&UartHandle, &ch, 1, HAL_MAX_DELAY) == HAL_OK)
        return (int)ch;
    return 0;   /* HAL error (rare) */
}
/*********************************************END OF FILE**********************/
