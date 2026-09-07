/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   ��V1.2.0�汾�⽨�Ĺ���ģ��
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32H743������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************
  */  
#include "stm32h7xx.h"
#include "main.h"
#include "bsp_led.h" 
#include "bsp_debug_usart.h"
#include "./menu/menu.h"
#include "./ota/ota.h"
#include "./key/bsp_key.h"
#include "./flash/bsp_qspi_flash.h"
#include "lwip/init.h"
#include "app_ethernet.h"
#include "ethernetif.h"
#include "lwip/timeouts.h"
#include "./delay/core_delay.h"  
//#include "./delay/core_delay.h"


int flag = 0;
int eth_inited = 0;   /* Ethernet/LwIP inited (set 1 only in [2] Firmware Update) */
int qspi_inited = 0;  /* QSPI inited (once, on first entry to interactive mode) */

/**
  * @brief  Interactive mode: Ethernet main loop + UART menu (0-6).
  *         LwIP polling continues every iteration (non-blocking).
  */
void interactive_mode(void)
{
    /* QSPI init: once, on first entry to interactive mode (init-before-use) */
    if (!qspi_inited) {
        if (BSP_QSPI_Init() == QSPI_OK) {
            printf("[QSPI] ID: 0x%06X\r\n", (unsigned)BSP_QSPI_ReadJedecID());
        } else {
            printf("[QSPI] init FAILED\r\n");
        }
        qspi_inited = 1;
    }

    Display_Menu();

    while (1)
    {
        /* LwIP RX: process received frames when ETH IRQ set flag (only if ETH inited) */
        if (eth_inited && flag)
        {
            flag = 0;
            LED2_TOGGLE;
            ethernetif_input(&gnetif);
        }
        /* LwIP timers (ARP, TCP retransmit...) (only if ETH inited) */
        if (eth_inited)
            sys_check_timeouts();

        /* OTA receive progress (non-blocking) */
        if (OTA_IsReceiving())
        {
            uint32_t exp = OTA_GetExpected();
            uint32_t got = OTA_GetReceived();
            uint32_t pct = (exp) ? ((got * 100) / exp) : 0;

            static uint32_t last_pct = 0xFFFFFFFF;
            static uint32_t last_tick = 0;
            /* update ~every 200ms or on progress change */
            if (pct != last_pct || (HAL_GetTick() - last_tick) > 200)
            {
                last_pct = pct;
                last_tick = HAL_GetTick();
                printf("\r[OTA] [");
                for (uint32_t i = 0; i < 30; i++)
                    printf("%c", (i < (pct * 30 / 100)) ? '#' : '-');
                printf("] %3lu%% (%lu/%lu)",
                       (unsigned long)pct, (unsigned long)got, (unsigned long)exp);
                if (pct == 100) printf("\r\n");
            }
        }

        /* OTA state machine: CRC verify / write Slot A / reset */
        OTA_Poll();

        /* OTA CONFIRM state: staged, waiting for user decision to write App */
        if (OTA_IsConfirm())
        {
            static int confirm_prompted = 0;
            if (!confirm_prompted) {
                confirm_prompted = 1;
                printf("\r\n[OTA] Firmware staged done (length %lu bytes)\r\n",
                       (unsigned long)OTA_GetReceived());
                printf("[OTA] Write to STM32 App address (0x08100000)? (y/n): ");
            }
            int c = UART_ReadKey_Blocking();
            if (c == 'y' || c == 'Y') {
                printf(" y\r\n");
                if (OTA_Flash_Write() == 0) {
                    printf("[OTA] Write done -> rebooting...\r\n");
                    HAL_Delay(50);
                    NVIC_SystemReset();
                } else {
                    printf("[OTA] Write FAILED\r\n");
                }
                OTA_ResetState();
                confirm_prompted = 0;
                printf("\r\nSelect (0-6): ");
            } else if (c == 'n' || c == 'N' || c == 27) {   /* n / ESC */
                printf(" n\r\n[OTA] Aborted, back to menu (staged kept)\r\n");
                OTA_ResetState();
                confirm_prompted = 0;
                printf("\r\nSelect (0-6): ");
            }
            /* c==0 timeout or ignored -> keep waiting for key */
        }

        /* UART key input (non-blocking) */
        int key = UART_ReadKey_NonBlock();
        if (key != 0)
        {
            Menu_HandleKey(key);   /* prints Select prompt internally (unless menu reprinted) */
        }
    }
}

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
    /* ��ETHʹ�õ��ڴ濪������*/
    MPU_Config(); 
  
    /* Enable I-Cache */
    SCB_EnableICache();

    /* Enable D-Cache */
    SCB_EnableDCache();  
    //��Cache����write-through��ʽ
    SCB->CACR|=1<<2;
  
    /* ����ϵͳʱ��Ϊ400 MHz */
    SystemClock_Config();

    /* ��ʼ��RGB�ʵ� */
    LED_GPIO_Config();

    /* ��ʼ��USART1 ����ģʽΪ 115200 8-N-1 */
    DEBUG_USART_Config();

    /* Key GPIO (KEY1=PA0, KEY2=PC13) */
    Key_GPIO_Config();

    /* Boot branch: KEY1 held (read GPIO directly, no wait for release) -> interactive menu; else normal_boot
       QSPI / Ethernet(LwIP) both deferred: QSPI at interactive_mode() entry, Ethernet at [2] */

    /* Metadata state step + boot decision (before KEY1 check)
       returns: 0=enter menu, 1=need watchdog jump, 2=jump directly */
    int boot_req = Boot_Metadata_Step();

    if (HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON) {
        printf("\r\n[BOOT] KEY1 pressed -> interactive menu\r\n");
        interactive_mode();
    } else if (boot_req == 0) {
        printf("\r\n[BOOT] No valid firmware -> interactive menu\r\n");
        interactive_mode();
    } else {
        if (boot_req == 1) {
            /* TESTING: enable IWDG(5s) then jump to App (health-check start) */
            printf("[BOOT] Enable IWDG(5s) -> jump to App\r\n");
            Iwdg_Enable(5000);
        }
        JumpToApp();
    }
}/* main end */
/**
  * @brief  System Clock ����
  *         system Clock ��������: 
	*            System Clock source  = PLL (HSE)
	*            SYSCLK(Hz)           = 480000000 (CPU Clock)
	*            HCLK(Hz)             = 240000000 (AXI and AHBs Clock)
	*            AHB Prescaler        = 2
	*            D1 APB3 Prescaler    = 2 (APB3 Clock  120MHz)
	*            D2 APB1 Prescaler    = 2 (APB1 Clock  120MHz)
	*            D2 APB2 Prescaler    = 2 (APB2 Clock  120MHz)
	*            D3 APB4 Prescaler    = 2 (APB4 Clock  120MHz)
	*            HSE Frequency(Hz)    = 25000000
	*            PLL_M                = 5
	*            PLL_N                = 192
	*            PLL_P                = 2
	*            PLL_Q                = 4
	*            PLL_R                = 2
	*            VDD(V)               = 3.3
	*            Flash Latency(WS)    = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;
  
  /*ʹ�ܹ������ø��� */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

  /* ��������ʱ��Ƶ�ʵ������ϵͳƵ��ʱ����ѹ���ڿ����Ż����ģ�
		 ����ϵͳƵ�ʵĵ�ѹ����ֵ�ĸ��¿��Բο���Ʒ�����ֲᡣ  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
 
  /* ����HSE������ʹ��HSE��ΪԴ����PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
 
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
	/* ѡ��PLL��Ϊϵͳʱ��Դ����������ʱ�ӷ�Ƶ�� */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK  | \
																 RCC_CLOCKTYPE_HCLK    | \
																 RCC_CLOCKTYPE_D1PCLK1 | \
																 RCC_CLOCKTYPE_PCLK1   | \
                                 RCC_CLOCKTYPE_PCLK2   | \
																 RCC_CLOCKTYPE_D3PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

/**
  * @brief  ����MPU���� 
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as Device not cacheable 
     for ETH DMA descriptors */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Configure the MPU attributes as Cacheable write through 
     for LwIP RAM heap which contains the Tx buffers */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
/****************************END OF FILE***************************/
