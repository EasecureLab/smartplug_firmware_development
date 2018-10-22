
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
void start_sys_main(void const *argument);
void start_wifi_recv(void const *argument);
void start_power_recv(void const *argument);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern uint8_t send_data_to_esp8266(char *fmt, ...);
#define UART1_RX_LEN  24

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture = 0;

uint32_t   voltage_parameter_value;
uint32_t   voltage_reg_value;
uint32_t   current_parameter_value;
uint32_t   current_reg_value;
uint32_t   power_parameter_value;
uint32_t   power_reg_value;
float voltage;
uint32_t voltage_int;
float current;
uint32_t current_int;



uint8_t  usart3_rx_flag = 0;
uint8_t  usart3_rx_buffer[128];
uint8_t  usart3_tx_buffer[128];
uint16_t usart3_tx_len = 0;


uint8_t  usart1_rx_flag = 0;
uint8_t  usart1_rx_buffer[48];
uint8_t  usart1_tx_buffer[48];
uint16_t usart1_tx_len = 0;

uint8_t usart_oneshot_send = 1;

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
  dma_send((unsigned char *)cmd, strlen((const char *)cmd));
  //printf("Sent:%s\r\n", cmd);
}

_Bool NET_DEVICE_SendCmd(char *cmd, char *res, _Bool mode)
{
  unsigned char timeOut = 300;
  //dma_send(ptr, len1);
  //NET_IO_Send((unsigned char *)cmd, strlen((const char *)cmd)); //Ð´ÃüÁîµ½ÍøÂçÉè±¸
  dma_send((unsigned char *)cmd, strlen((const char *)cmd));
  printf("Sent:%s\r\n", cmd);
  while (timeOut--)
  {
    //block way receive handle
    if (usart3_rx_flag == 1)
    {
      usart3_rx_flag = 0;
      printf("Received:%s\r\n", usart3_tx_buffer);
      memset(usart3_tx_buffer, 0x00, 128);
    }
    if (strstr(usart3_tx_buffer, "ERROR") != NULL)
    {
      //if there is some error, lock in this function
      timeOut = 300;
    }
    if (strstr(usart3_tx_buffer, res) != NULL)
    {

      return 0;
    }

    HAL_Delay(10);
  }
  return 1;
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

void value2string(int value)
{
  //char temp_str[128];
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


  value2str[0] = 'v';
  value2str[1] = 'o';
  value2str[2] = 'l';
  value2str[3] = ':';

  value2str[4] = thousand_char;
  value2str[5] = hundred_char;
  value2str[6] = deci_char;
  value2str[7] = last_char;
  value2str[8] = ' ';
  value2str[9] = ' ';
  //value2str[4]='\r';
  //value2str[5]='\n';
  value2str[10] = '\0';
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

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
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
    HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, UART1_RX_LEN);
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
    //wifi send function part
    osSemaphoreWait(sys_main_can_send_wifiHandle, osWaitForever);
    if (net_state_machine == 0)
    {
      net_sendcmd_noblock("AT+CWMODE_DEF=1\r\n");
    }
    if (net_state_machine == 1)
    {
      net_sendcmd_noblock("AT+CWJAP_DEF=\"wsn405\",\"wsn405405\"\r\n");
    }
    if (net_state_machine == 2)
    {
      net_sendcmd_noblock("AT+CIPSTART=\"TCP\",\"132.232.89.145\",9999\r\n");
    }
    if (net_state_machine == 3)
    {
      net_sendcmd_noblock("AT+CIPSEND=7\r\n");
    }
    if (net_state_machine == 4)
    {
      net_sendcmd_noblock("hello\r\n");
    }
    if (net_state_machine == 5)
    {
      net_sendcmd_noblock("AT+CIPSEND=11\r\n");
    }
    if (net_state_machine == 6)
    {
      dma_send(value2str_vol, 11);
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
      if (net_state_machine == 0)
      {
        net_state_machine = 1;
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
        osDelay(2000);
        osSemaphoreRelease(sys_main_can_send_wifiHandle);
      }
      else if (net_state_machine == 1)
      {
        net_state_machine = 1;

        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = 2;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }

        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == 2)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = 3;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == 3)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = 4;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == 4)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = 5;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);
        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);
      }
      else if (net_state_machine == 5)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = 6;
          memset(usart3_tx_buffer, 0x00, 128);
          osSemaphoreRelease(sys_main_can_send_wifiHandle);

        }
        printf("Received:%s\r\n", usart3_tx_buffer);
        memset(usart3_tx_buffer, 0x00, 128);

      }
      else if (net_state_machine == 6)
      {
        if (strstr(usart3_tx_buffer, "OK"))
        {
          net_state_machine = 5;
          memset(usart3_tx_buffer, 0x00, 128);
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
  /* Infinite loop */
  for (;;)
  {
    if (usart1_rx_flag == 1)
    {
      count++;
      usart1_rx_flag = 0;
      if (count > 20)
      {
        count = 0;
        printf("voltage=%f\r\n", voltage);
        printf("current=%f\r\n", current);
      }
      #if 1
      //printf("PowerPlug:%x %x %x %x %x %x \r\n", usart1_rx_buffer[0], usart1_rx_buffer[1], usart1_rx_buffer[2], usart1_rx_buffer[3], usart1_rx_buffer[4], usart1_rx_buffer[5]);
      //printf("PowerPlug:%x %x %x %x %x %x \r\n", usart1_rx_buffer[6], usart1_rx_buffer[7], usart1_rx_buffer[8], usart1_rx_buffer[9], usart1_rx_buffer[10], usart1_rx_buffer[11]);
      //printf("PowerPlug:%x %x %x %x %x %x \r\n", usart1_rx_buffer[12], usart1_rx_buffer[13], usart1_rx_buffer[14], usart1_rx_buffer[15], usart1_rx_buffer[16], usart1_rx_buffer[17]);
      //printf("PowerPlug:%x %x %x %x %x %x \r\n", usart1_rx_buffer[18], usart1_rx_buffer[19], usart1_rx_buffer[20], usart1_rx_buffer[21], usart1_rx_buffer[22], usart1_rx_buffer[23]);

      voltage_parameter_value = usart1_rx_buffer[2] * 65536 + usart1_rx_buffer[3] * 256 + usart1_rx_buffer[4];
      voltage_reg_value = usart1_rx_buffer[5] * 65536 + usart1_rx_buffer[6] * 256 + usart1_rx_buffer[7];
      current_parameter_value = usart1_rx_buffer[8] * 65536 + usart1_rx_buffer[9] * 256 + usart1_rx_buffer[10];
      current_reg_value = usart1_rx_buffer[11] * 65536 + usart1_rx_buffer[12] * 256 + usart1_rx_buffer[13];
      power_parameter_value = usart1_rx_buffer[14] * 65536 + usart1_rx_buffer[15] * 256 + usart1_rx_buffer[16];
      power_reg_value = usart1_rx_buffer[17] * 65536 + usart1_rx_buffer[18] * 256 + usart1_rx_buffer[19];

      //printf("voltage_parameter_value=%d,voltage_reg_value=%d\r\n", voltage_parameter_value, voltage_reg_value);
      voltage = voltage_parameter_value / voltage_reg_value * 1.88;
      //printf("voltage=%f\r\n", voltage);
      voltage_int = (int)voltage - 16;
      //printf("voltage_int=%d\r\n", voltage_int);

      //printf("current_parameter_value=%d,current_reg_value=%d\r\n", current_parameter_value, current_reg_value);
      current = current_parameter_value / current_reg_value * 0.5;
      //printf("current=%f\r\n", current);
      current_int = (int)current;
      //printf("current_int=%d\r\n", current_int);

      memset(usart1_tx_buffer, 0x00, 48);

      value2str_in_out((int)voltage_int, &value2str_vol[0]);
      value2str_in_out((int)current_int, &value2str_cur[0]);
      #endif
      //HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, UART1_RX_LEN);
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
