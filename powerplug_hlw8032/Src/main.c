
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId sys_main_taskHandle;
osThreadId wifi_recv_taskHandle;
osThreadId power_recv_taskHandle;
osSemaphoreId sys_main_can_send_wifiHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
void start_sys_main(void const * argument);
void start_wifi_recv(void const * argument);
void start_power_recv(void const * argument);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//net_state_machine
typedef enum
{
  NET_STATE_MACHINE_CWMODE_DEF_1 = 0,
  NET_STATE_MACHINE_CWJAP_DEF_1 = 1,
  NET_STATE_MACHINE_CIPSTART_TCP_IP = 2,
  NET_STATE_MACHINE_CIPSEND_7 = 3,
  NET_STATE_MACHINE_CIPSEND_CONTENT_1 = 4,
  NET_STATE_MACHINE_CIPSEND_11_1 = 5,
  NET_STATE_MACHINE_CIPSEND_CONTENT_2 = 6,
  NET_STATE_MACHINE_CIPSEND_11_2 = 7,
  NET_STATE_MACHINE_CIPSEND_CONTENT_3 = 8,
  NET_STATE_MACHINE_CWJAP_QUERY = 9,
  NET_STATE_MACHINE_ = 255
} net_state_machine_status;

#define UART1_RX_LEN  24

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

uint32_t   voltage_par_value;
uint32_t   voltage_reg_value;
uint32_t   current_par_value;
uint32_t   current_reg_value;
uint32_t   power_param_value;
uint32_t   power_reges_value;
float      voltage;
uint32_t   voltage_int;
float      current;
uint32_t   current_int;

uint8_t    usart3_rx_flag = 0;
uint8_t    usart3_rx_buffer[128];
uint8_t    usart3_tx_buffer[128];
uint16_t   usart3_tx_len = 0;

uint8_t    usart1_rx_flag = 0;
uint8_t    usart1_rx_buffer[48];
uint8_t    usart1_tx_buffer[48];
uint16_t   usart1_tx_len = 0;

char thousand_char;
char hundred_char;
char deci_char;
char last_char;
char value2str[11];
char value2str_vol[11];
char value2str_cur[11];
int net_state_machine = 0;

_Bool net_sendcmd_noblock(char *cmd)
{
  printf("Sent:%s\r\n", cmd);
  dma_send((unsigned char *)cmd, strlen((const char *)cmd));
  //printf("Sent:%s\r\n", cmd);
}

void value2str_in_out(int value, char *str)
{
  int temp_value;

  if (value / 10000 != 0)
  {
    return ;
  }
  else if (value / 10000 == 0) //below 10000
  {
    thousand_char = value / 1000 + 0x30; //thousand
    temp_value = value % 1000; // hundred deci etc

    if (temp_value / 100 != 0)
    {
      hundred_char = temp_value / 100 + 0x30;
    }
    else if (temp_value / 100 == 0)
    {
      hundred_char = 0x30;
    }
    temp_value = value % 100; //deci etc;

    if (temp_value / 10 != 0)
    {
      deci_char = temp_value / 10 + 0x30;
    }
    else if (temp_value / 10 == 0)
    {
      deci_char = 0x30;
    }
    last_char = value % 10 + 0x30;
  }


  str[0] = 'v';
  str[1] = 'o';
  str[2] = 'l';
  str[3]  = ':';

  str[4] = thousand_char;
  str[5] = hundred_char;
  str[6] = deci_char;
  str[7] = last_char;
  str[8] = ' ';
  str[9] = ' ';
  str[10] = '\0';
}
void hlw8032_value_adjust(uint8_t *rx_src, uint8_t *rx_adjust)
{
  int rx_bf_index;
  uint8_t rx_bf_temp_pre;
  uint8_t rx_bf_temp_cur;
  for (rx_bf_index = 0; rx_bf_index < 24; rx_bf_index++)
  {
    rx_bf_temp_cur = rx_src[rx_bf_index];
    if (rx_bf_temp_cur == 0x55)
    {
      rx_bf_temp_pre = rx_bf_temp_cur;
    }
    else if (rx_bf_temp_cur == 0x5a)
    {
      if (rx_bf_temp_pre == 0x55)
      {
        if (rx_bf_index == 1)
        {
          memcpy(rx_adjust + 2, &rx_src[rx_bf_temp_cur + 2], 24 - (rx_bf_temp_cur - 1));
          memcpy(rx_adjust + 2 + (24 - (rx_bf_temp_cur - 1)), &rx_src[0], rx_bf_temp_cur - 1);
        }
      }
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_IWDG_Init(&hiwdg);
  //uart1 hlw8032
  //uart2 printf
  //uart3 esp8266

  printf("sichuan university wsn lab\r\n");
  printf("project powerplug v1.0\r\n");
  printf("compile date: %s %s\r\n", __DATE__, __TIME__);

  //uart3 receive dma wifi at command
  if (HAL_UART_Receive_DMA(&huart3, (uint8_t *)&usart3_rx_buffer, 128) != HAL_OK)
  {
    Error_Handler();
  }

  //enable uart3 idle interrupt
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

  //uart1 receive dma hlw8032 data 24byte
  HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, UART1_RX_LEN);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of sys_main_can_send_wifi */
  osSemaphoreDef(sys_main_can_send_wifi);
  sys_main_can_send_wifiHandle = osSemaphoreCreate(osSemaphore(sys_main_can_send_wifi), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of sys_main_task */
  osThreadDef(sys_main_task, start_sys_main, osPriorityHigh, 0, 128);
  sys_main_taskHandle = osThreadCreate(osThread(sys_main_task), NULL);

  /* definition and creation of wifi_recv_task */
  osThreadDef(wifi_recv_task, start_wifi_recv, osPriorityAboveNormal, 0, 128);
  wifi_recv_taskHandle = osThreadCreate(osThread(wifi_recv_task), NULL);

  /* definition and creation of power_recv_task */
  osThreadDef(power_recv_task, start_power_recv, osPriorityNormal, 0, 128);
  power_recv_taskHandle = osThreadCreate(osThread(power_recv_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */


  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 4800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CTL_Pin_GPIO_Port, CTL_Pin_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CTL_Pin_Pin */
  GPIO_InitStruct.Pin = CTL_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CTL_Pin_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    usart1_rx_flag = 1;
    //HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, UART1_RX_LEN);
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, UART1_RX_LEN);
  }
}

/* USER CODE END 4 */

/* start_sys_main function */
void start_sys_main(void const *argument)
{

  /* USER CODE BEGIN 5 */
  osSemaphoreRelease(sys_main_can_send_wifiHandle);
  /* Infinite loop */
  for (;;)
  {
    osDelay(50);
    HAL_IWDG_Refresh(&hiwdg);
    //wifi send function part
    osSemaphoreWait(sys_main_can_send_wifiHandle, osWaitForever);
    //HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, UART1_RX_LEN);
    if (net_state_machine == NET_STATE_MACHINE_CWMODE_DEF_1)
    {
      net_sendcmd_noblock("AT+CWMODE_DEF=1\r\n");
    }
    if (net_state_machine == NET_STATE_MACHINE_CWJAP_DEF_1)
    {
      net_sendcmd_noblock("AT+CWJAP_DEF=\"wsn405\",\"wsn405405\"\r\n");
      //net_sendcmd_noblock("AT+CWJAP_DEF=\"davwang\",\"15908106107\"\r\n");

    }
    if (net_state_machine == NET_STATE_MACHINE_CIPSTART_TCP_IP)
    {
      net_sendcmd_noblock("AT+CIPSTART=\"TCP\",\"120.78.149.124\",9999\r\n");
    }
    if (net_state_machine == NET_STATE_MACHINE_CIPSEND_7)
    {
      net_sendcmd_noblock("AT+CIPSEND=7\r\n");
    }
    if (net_state_machine == NET_STATE_MACHINE_CIPSEND_CONTENT_1)
    {
      net_sendcmd_noblock("hello\r\n");
    }
    if (net_state_machine == NET_STATE_MACHINE_CIPSEND_11_1)
    {
      net_sendcmd_noblock("AT+CIPSEND=11\r\n");
    }
    if (net_state_machine == NET_STATE_MACHINE_CIPSEND_CONTENT_2)
    {
      printf("Send Data\r\n");
      dma_send(value2str_vol, 11);
    }
    if (net_state_machine == NET_STATE_MACHINE_CIPSEND_11_2)
    {
      net_sendcmd_noblock("AT+CIPSEND=11\r\n");
    }
    if (net_state_machine == NET_STATE_MACHINE_CIPSEND_CONTENT_3)
    {
      memcpy(value2str_cur, "cur", 3);
      printf("Send Data\r\n");
      dma_send(value2str_cur, 11);
    }
    if (net_state_machine == NET_STATE_MACHINE_CWJAP_QUERY)
    {
      net_sendcmd_noblock("AT+CWJAP?\r\n");
    }
  }
  /* USER CODE END 5 */
}

/* start_wifi_recv function */
void start_wifi_recv(void const *argument)
{
  /* USER CODE BEGIN start_wifi_recv */
  /* Infinite loop */
  for (;;)
  {
    osDelay(50);
    if (usart3_rx_flag == 1)
    {
      usart3_rx_flag = 0;
      #if 0
      printf("Init Received:%s\r\n", usart3_tx_buffer);
      // control the relay via the python socket
      if (strstr(usart3_tx_buffer, "action:on"))
      {
        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(CTL_Pin_GPIO_Port, CTL_Pin_Pin, GPIO_PIN_SET);
      }
      else if (strstr(usart3_tx_buffer, "action:off"))
      {
        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(CTL_Pin_GPIO_Port, CTL_Pin_Pin, GPIO_PIN_RESET);

      }
      #endif
      //all the usart3 buffer should be checked firstly.
      if (strstr(usart3_tx_buffer, "ERROR"))
      {
        net_state_machine = NET_STATE_MACHINE_CWMODE_DEF_1;
        memset(usart3_tx_buffer, 0x00, 128);
        osDelay(2000);
        osSemaphoreRelease(sys_main_can_send_wifiHandle);
      }
      //net state machine switch
      if (net_state_machine == NET_STATE_MACHINE_CWMODE_DEF_1)
      {
        net_state_machine = 1;
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
        osDelay(2000);
        osSemaphoreRelease(sys_main_can_send_wifiHandle);
      }
      else if (net_state_machine == NET_STATE_MACHINE_CWJAP_DEF_1)
      {
        net_state_machine = 1;

        if ((strstr(usart3_tx_buffer, "OK")) || (strstr(usart3_tx_buffer, "WIFI")))
        {
          net_state_machine = NET_STATE_MACHINE_CIPSTART_TCP_IP;
          memset(usart3_tx_buffer, 0x00, 128);
          osDelay(4000);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }

        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == NET_STATE_MACHINE_CIPSTART_TCP_IP)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = NET_STATE_MACHINE_CIPSEND_7;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }
        else  if ((strstr(usart3_tx_buffer, "ERROR")) || (strstr(usart3_tx_buffer, "link is not valid")))
        {
          printf("tcp connect error\r\n");
          net_state_machine = NET_STATE_MACHINE_CWMODE_DEF_1;
          memset(usart3_tx_buffer, 0x00, 128);
          osDelay(2000);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == NET_STATE_MACHINE_CIPSEND_7)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = NET_STATE_MACHINE_CIPSEND_CONTENT_1;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == NET_STATE_MACHINE_CIPSEND_CONTENT_1)
      {
        if ((strstr(usart3_tx_buffer, "OK")) || (strstr(usart3_tx_buffer, "Recv")))
        {
          net_state_machine = NET_STATE_MACHINE_CIPSEND_11_1;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == NET_STATE_MACHINE_CIPSEND_11_1)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = NET_STATE_MACHINE_CIPSEND_CONTENT_2;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);

        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);

      }
      else if (net_state_machine == NET_STATE_MACHINE_CIPSEND_CONTENT_2)
      {
        if ((strstr(usart3_tx_buffer, "OK")) || (strstr(usart3_tx_buffer, "Recv")))
        {
          net_state_machine = NET_STATE_MACHINE_CIPSEND_11_2;
          memset(usart3_tx_buffer, 0x00, 128);
          osDelay(2000);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);

        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == NET_STATE_MACHINE_CIPSEND_11_2)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = NET_STATE_MACHINE_CIPSEND_CONTENT_3;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);

        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == NET_STATE_MACHINE_CIPSEND_CONTENT_3)
      {
        if ((strstr(usart3_tx_buffer, "OK")) || (strstr(usart3_tx_buffer, "Recv")))
        {
          net_state_machine = NET_STATE_MACHINE_CIPSEND_11_1;
          memset(usart3_tx_buffer, 0x00, 128);
          osDelay(2000);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);

        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == NET_STATE_MACHINE_CWJAP_QUERY)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = NET_STATE_MACHINE_CIPSEND_11_1;
          memset(usart3_tx_buffer, 0x00, 128);
          //osDelay(2000);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);

        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }

    }
  }
  /* USER CODE END start_wifi_recv */
}

/* start_power_recv function */
void start_power_recv(void const *argument)
{
  /* USER CODE BEGIN start_power_recv */
  uint16_t count;
  uint8_t rx_buff_temp[24];
  uint8_t a[24];
  uint8_t b[24];
  int pre_net_state_machine;
  int cur_net_state_machine;
  int net_state_unchanged_count;
  int i;
  int start_index;
  /* Infinite loop */
  for (;;)
  {
    //osDelay(50);
    HAL_IWDG_Refresh(&hiwdg);


    if (usart1_rx_flag == 1)
    {

      usart1_rx_flag = 0;
      taskENTER_CRITICAL();
      // copy the data to the temp buffer
      memcpy(rx_buff_temp, usart1_rx_buffer, 24);

      memcpy(a, rx_buff_temp, 24);
      //printf("%x %x %x %x %x %x %x %x %x %x   %x %x %x %x %x %x %x %x %x %x   %x %x %x %x \r\n", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15], a[16], a[17], a[18], a[19], a[20], a[21], a[22], a[23]);
      for (i = 0; i < 24; i++)
      {
        if ((a[i] == 0x55) && (a[i] == 0x5a))
        {
          start_index = i;
          printf("i=%d\r\n", i);
        }
      }
      memcpy(b, a + start_index, 24 - start_index);
      memcpy(b + start_index, a, start_index);
      memcpy(a, b, 24);
      memcpy(rx_buff_temp, a, 24);
      //printf("a[0]=%x\r\n", a[0]);
      osDelay(50);
      //printf("convert:%x %x %x %x %x %x %x %x %x %x   %x %x %x %x %x %x %x %x %x %x   %x %x %x %x \r\n", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15], a[16], a[17], a[18], a[19], a[20], a[21], a[22], a[23]);
      taskEXIT_CRITICAL();
	  //osDelay(100);
      HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, UART1_RX_LEN);

      count++;
      if (count > 20)
      {
        count = 0;
        printf("voltage=%f\r\n", voltage);
        printf("current=%f\r\n", current);
        printf("net_state_machine=%d\r\n", net_state_machine);
        cur_net_state_machine = net_state_machine;
        if (cur_net_state_machine == pre_net_state_machine)
        {
          net_state_unchanged_count++;
        }
        else
        {
          net_state_unchanged_count = 0;
        }
        if (net_state_unchanged_count > 5)
        {
          net_state_machine++;
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }
        pre_net_state_machine = cur_net_state_machine;
        printf("cycle:%x %x %x %x %x %x %x %x %x %x	 %x %x %x %x %x %x %x %x %x %x	 %x %x %x %x \r\n", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15], a[16], a[17], a[18], a[19], a[20], a[21], a[22], a[23]);
      }

      // 20*50=1000ms, 1s send the voltage and current data to usart
      voltage_par_value = rx_buff_temp[2] * 0xFFFF + rx_buff_temp[3] * 0xFF + rx_buff_temp[4];
      voltage_reg_value = rx_buff_temp[5] * 0xFFFF + rx_buff_temp[6] * 0xFF + rx_buff_temp[7];
      current_par_value = rx_buff_temp[8] * 0xFFFF + rx_buff_temp[9] * 0xFF + rx_buff_temp[10];
      current_reg_value = rx_buff_temp[11] * 0xFFFF + rx_buff_temp[12] * 0xFF + rx_buff_temp[13];
      power_param_value = rx_buff_temp[14] * 0xFFFF + rx_buff_temp[15] * 0xFF + rx_buff_temp[16];
      power_reges_value = rx_buff_temp[17] * 0xFFFF + rx_buff_temp[18] * 0xFF + rx_buff_temp[19];
      voltage = voltage_par_value / voltage_reg_value * 1.88;
      voltage_int = (int)voltage ;
      current = current_par_value / current_reg_value * 0.5;
      current_int = (int)current;

      value2str_in_out((int)voltage_int, &value2str_vol[0]);
      value2str_in_out((int)current_int, &value2str_cur[0]);

    }
  }
  /* USER CODE END start_power_recv */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
