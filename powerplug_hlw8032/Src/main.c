
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

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
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern uint8_t send_data_to_esp8266(char *fmt, ...);

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




/* Private variables ---------------------------------------------------------*/
/* Captured Value */
uint16_t            uhIC2Value = 0;
/* Duty Cycle Value */
uint16_t            uhDutyCycle = 0;
/* Frequency Value */
uint32_t            uwFrequency = 0;
/* Duty Cycle Value */
uint16_t            uhDutyCycle2 = 0;
/* Frequency Value */
uint32_t            uwFrequency2 = 0;
uint8_t             temp_ch = 0x55;

uint8_t  usart3_rx_flag = 0;
uint8_t  usart3_rx_buffer[128];
uint8_t  usart3_tx_buffer[128];
uint16_t usart3_tx_len = 0;
uint16_t send_count = 0;

uint8_t  usart1_rx_flag = 0;
uint8_t  usart1_rx_buffer[48];
uint8_t  usart1_tx_buffer[48];
uint16_t usart1_tx_len = 0;

uint8_t usart_oneshot_send = 1;


#define UART1_RX_LEN  24
_Bool net_sendcmd_noblock(char *cmd)
{
  dma_send((unsigned char *)cmd, strlen((const char *)cmd));
  printf("Sent:%s\r\n", cmd);

}

_Bool NET_DEVICE_SendCmd(char *cmd, char *res, _Bool mode)
{
  unsigned char timeOut = 300;
  //dma_send(ptr, len1);
  //NET_IO_Send((unsigned char *)cmd, strlen((const char *)cmd)); //写命令到网络设备
  dma_send((unsigned char *)cmd, strlen((const char *)cmd));
  printf("Sent:%s\r\n", cmd);
  while (timeOut--)
  {
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

char thousand_char;
char hundred_char;
char deci_char;
char last_char;
char value2str[10];
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

  value2str[0] = thousand_char;
  value2str[1] = hundred_char;
  value2str[2] = deci_char;
  value2str[3] = last_char;
  value2str[4] = ' ';
  value2str[5] = ' ';
  //value2str[4]='\r';
  //value2str[5]='\n';
  value2str[6] = '\0';


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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (usart_oneshot_send == 1)
    {
      usart_oneshot_send = 0;
      //CWMODE_DEF =1
      NET_DEVICE_SendCmd("AT+CWMODE_DEF=1\r\n", "OK", 1);
      //CWJAP
      NET_DEVICE_SendCmd("AT+CWJAP_DEF=\"TP-LINK_B11273\",\"kaiyuan1028\"\r\n", "OK", 1);
      //NET_DEVICE_SendCmd("AT+CWJAP_DEF=\"WSN405\",\"wsn405405\"\r\n", "OK", 1);
      //NET_DEVICE_SendCmd("AT+CWJAP_DEF=\"WSN407\",\"wsn407407\"\r\n", "OK", 1);
      HAL_Delay(3000);
      NET_DEVICE_SendCmd("AT+CIPSTART=\"TCP\",\"192.168.1.100\",1234\r\n", "OK", 1);
      //NET_DEVICE_SendCmd("AT+CIPSTART=\"TCP\",\"192.168.199.102\",8080\r\n", "OK", 1);
      //NET_DEVICE_SendCmd("AT+CIPSTART=\"TCP\",\"192.168.1.144\",1234\r\n", "OK", 1);
      //the length of data which be sent
      NET_DEVICE_SendCmd("AT+CIPSEND=7\r\n", "OK", 1);
      HAL_Delay(2000);
      //the data content
      NET_DEVICE_SendCmd("abcde\r\n", "OK", 1);
    }

    //esp8266 data send to smart phone, two step including 1,length which will be sent;2,send the data;
    //send cmd with block
    NET_DEVICE_SendCmd("AT+CIPSEND=7\r\n", "OK", 1);
    //voltage integer value convert into the value2str
    if (voltage_int == 0)
    {
      //HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, UART1_RX_LEN);
      //printf("reinit the uart1 for the hlw8032\r\n");
    }
    value2string((int)voltage_int);
    //the data content
    dma_send(value2str, 7);
    HAL_Delay(10);


    //esp8266 data processing algorithm
    if (usart3_rx_flag == 1)
    {
      usart3_rx_flag = 0;
      printf("Received:%s\r\n", usart3_tx_buffer);
      memset(usart3_tx_buffer, 0x00, 128);
    }

    //hlw8032 data processing algorithm
    if (usart1_rx_flag == 1)
    {
      usart1_rx_flag = 0;
      //printf("Need to handle uart1 data\r\n");
      printf("PowerPlug:%x %x %x %x %x %x \r\n", usart1_rx_buffer[0], usart1_rx_buffer[1], usart1_rx_buffer[2], usart1_rx_buffer[3], usart1_rx_buffer[4], usart1_rx_buffer[5]);
      printf("PowerPlug:%x %x %x %x %x %x \r\n", usart1_rx_buffer[6], usart1_rx_buffer[7], usart1_rx_buffer[8], usart1_rx_buffer[9], usart1_rx_buffer[10], usart1_rx_buffer[11]);
      printf("PowerPlug:%x %x %x %x %x %x \r\n", usart1_rx_buffer[12], usart1_rx_buffer[13], usart1_rx_buffer[14], usart1_rx_buffer[15], usart1_rx_buffer[16], usart1_rx_buffer[17]);
      printf("PowerPlug:%x %x %x %x %x %x \r\n", usart1_rx_buffer[18], usart1_rx_buffer[19], usart1_rx_buffer[20], usart1_rx_buffer[21], usart1_rx_buffer[22], usart1_rx_buffer[23]);

      voltage_parameter_value = usart1_rx_buffer[2] * 65536 + usart1_rx_buffer[3] * 256 + usart1_rx_buffer[4];
      voltage_reg_value = usart1_rx_buffer[5] * 65536 + usart1_rx_buffer[6] * 256 + usart1_rx_buffer[7];
      current_parameter_value = usart1_rx_buffer[8] * 65536 + usart1_rx_buffer[9] * 256 + usart1_rx_buffer[10];
      current_reg_value = usart1_rx_buffer[11] * 65536 + usart1_rx_buffer[12] * 256 + usart1_rx_buffer[13];
      power_parameter_value = usart1_rx_buffer[14] * 65536 + usart1_rx_buffer[15] * 256 + usart1_rx_buffer[16];
      power_reg_value = usart1_rx_buffer[17] * 65536 + usart1_rx_buffer[18] * 256 + usart1_rx_buffer[19];

      printf("voltage_parameter_value=%d,voltage_reg_value=%d\r\n", voltage_parameter_value, voltage_reg_value);
      voltage = voltage_parameter_value / voltage_reg_value * 1.88;
      printf("voltage=%f\r\n", voltage);
      voltage_int = (int)voltage - 16;
      printf("voltage_int=%d\r\n", voltage_int);

      memset(usart1_tx_buffer, 0x00, 48);
    }
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
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
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(CTL_Pin_GPIO_Port, CTL_Pin_Pin, GPIO_PIN_RESET);

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
    //printf("recv power data\r\n");
    usart1_rx_flag = 1;
    HAL_UART_Receive_IT(&huart1, usart1_rx_buffer, UART1_RX_LEN);

    //HAL_UART_Transmit_DMA(&huart2,usart1_rx_buffer,128); // DM2发送出去
    // HAL_UART_Receive_DMA(&huart1,aRxBuffer1,1); // 重新DMA接收
    //HAL_UART_Receive_DMA(&huart1,(uint8_t *)&usart1_rx_buffer,128);
  }
}

#if 0
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if (uwIC2Value1 != 0)
    {
      /* Duty cycle computation */
      uhDutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2)) * 100) / uwIC2Value1;
      /* uwFrequency computation
      //TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */
      uwFrequency = (HAL_RCC_GetHCLKFreq()) / 2 / uwIC2Value1;
    }
    else
    {
      uhDutyCycle = 0;
      uwFrequency = 0;
    }
  }
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    if (uwIC2Value2 != 0)
    {
      uhDutyCycle2 = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2)) * 100) / uwIC2Value2;
      uwFrequency2 = (HAL_RCC_GetHCLKFreq()) / 2 / uwIC2Value2;
    }
    else
    {
      uhDutyCycle2 = 0;
      uwFrequency2 = 0;
    }
  }
}
#endif
/* USER CODE END 4 */

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
