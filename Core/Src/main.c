/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "float.h"
#include "nrf24.h"
#include "messages_stm.pb.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
#include "flash_f1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_PAGE_ADDR 0x0801FC00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t sample1 = 0, sample2 = 0, diff = 0, freq = 0;
float cons_inst = 0, cons_total = 0;
uint8_t is_sample1 = 1, enable_send = 0;
uint16_t samples_freq[1024], n_samples = 0;

float aux_cons_inst = 0;
int samples = 0;

uint8_t node_address[2][6] = {"HUB01", "WA101"};
char sensor_serial[6] = "WA101";
bool pairingMode = false;
uint8_t data[32];
uint8_t length;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static bool nrf_report();
static bool nrf_pairing();
static void nrf_pairing_report(char *_serial, int _channel);
/* USER CODE BEGIN PFP */
void r_PairingMessage(uint8_t *_data, int _data_len);
void w_message(double instant, int samples);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);

  return len;
}

float freq_to_consumption(float freq)
{
  return ((11.086 * freq - 9.357) / 3.6); /*Eq da reta conforme datasheet - conversão para ml/s*/
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /*Buffers flash*/
  //	uint8_t buff_write_flash[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  //	uint8_t buff_read_flash[16];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  NRF24_begin(GPIOB, NRF_CSN_Pin, NRF_CE_Pin, hspi1);

  NRF24_openWritingPipe(node_address[1], sizeof(node_address[1]) - 1);

  NRF24_openReadingPipe(1, node_address[0], sizeof(node_address[0]) - 1);
  //  NRF24_openReadingPipe(2, node_address[1], sizeof(node_address[1]) - 1);

  NRF24_stopListening();

  printRadioSettings();

  printf("END SETUP\n\n");

  /* Operações Flash*/
 //  Flash_Write_Data(FLASH_PAGE_ADDR , (uint32_t*)buff_write_flash, (sizeof(buff_write_flash)/sizeof(int)));
   Flash_Read_Data(FLASH_PAGE_ADDR , (uint32_t*)node_address[0], (sizeof(node_address[0])/sizeof(int)));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (pairingMode)
    {
      NRF24_startListening();
      HAL_Delay(500);
      unsigned long start = HAL_GetTick();
      bool aux = false;
      while (aux != true && (unsigned long)(HAL_GetTick()) - start <= 15000)
      {
        if (nrf_pairing())
        {
          printf("\n\n[PAIRING] report");
          nrf_pairing_report(sensor_serial, 69);
          aux = true;
        }
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
        HAL_Delay(50);
      }
      if ((unsigned long)(HAL_GetTick()) - start > 15000)
      {
        printf("\n\n[PAIRING] Timeout\n\n");
      }

      NRF24_stopListening();
      HAL_Delay(500);
      start = HAL_GetTick();
      while (aux == true && (unsigned long)(HAL_GetTick()) - start <= 15000)
      {
        if (nrf_report())
        {
          aux = false;
          printf("\n\n[PAIRING] Success\n\n");
        }
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
        HAL_Delay(250);
      }
      if ((unsigned long)(HAL_GetTick()) - start > 15000)
      {
        printf("\n\n[PAIRING] Timeout\n\n");
      }

      pairingMode = false;
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    }

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    if (freq && n_samples > 8)
    {
      cons_inst = freq_to_consumption((float)(freq / n_samples)); // mL/s
      cons_total += (cons_inst / 1000);
      printf("Amostras: %d	Instantaneo:%.1f mL/seg Acumulado:%.3f L\r\n", (int)n_samples, cons_inst, cons_total);
      memset(samples_freq, 0, n_samples * 2);
      n_samples = 0;
      freq = 0;

      if (samples == 0)
        w_message(cons_inst, samples);
      else
        w_message(aux_cons_inst, samples);

      unsigned long start = HAL_GetTick();
      bool aux = false;
      while (aux == false && (unsigned long)(HAL_GetTick()) - start <= 500)
      {
        if (nrf_report())
        {
          aux_cons_inst = 0;
          samples = 0;
          aux = true;
          HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
        }
      }
      if ((unsigned long)(HAL_GetTick()) - start > 500)
      {
        printf("\n[TX] Timeout\n");
        aux_cons_inst += cons_inst;
        samples++;
      }
    }

    HAL_Delay(1000);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF_CE_Pin | LED_2_Pin | LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin LED_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin | LED_2_Pin | LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW2_Pin | SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {

    if (is_sample1)
    {
      sample1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      is_sample1 = 0;
    }
    else
    {
      sample2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      diff = (sample2 > sample1) ? (sample2 - sample1) : ((0xFFFF - sample1) + sample2 + 1);

      samples_freq[n_samples] = (HAL_RCC_GetPCLK1Freq() / 24) / diff;

      __HAL_TIM_SetCounter(htim, 0);

      freq += samples_freq[n_samples++];
      is_sample1 = 1;
    }
  }
}

unsigned long last_micros;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  unsigned long debouncing_time = 200; // Debouncing Time in Milliseconds
  if (GPIO_Pin == SW1_Pin)
  {
    if ((HAL_GetTick() - last_micros >= debouncing_time))
    {
      printf("\n\n\nSW1\n\n\n");
      last_micros = HAL_GetTick();
    }
  }
  if (GPIO_Pin == SW2_Pin)
  {
    if ((HAL_GetTick() - last_micros >= debouncing_time))
    {
      printf("\n\n\nPAIRING MODE ON\n\n\n");
      pairingMode = true;
      last_micros = HAL_GetTick();
    }
  }
}

void w_message(double instant, int samples)
{
  uint8_t buffer[32];
  WaterSensorReport msg = WaterSensorReport_init_zero;

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  msg.instant = instant;
  msg.samples = samples;
  pb_encode(&stream, WaterSensorReport_fields, &msg);

  printf("MSG SERIALIZED : ");
  for (int i = 0; i < stream.bytes_written; i++)
  {
    data[i] = buffer[i];
    printf("%02X", data[i]);
  }
  printf("\n");

  data[31] = (uint8_t)stream.bytes_written;
}

bool nrf_report()
{
  unsigned long start_time = HAL_GetTick();
  bool reported = NRF24_write(&data, sizeof(data)); // transmit & save the report
  unsigned long end_time = HAL_GetTick();

  if (reported)
  {
    printf("Transmission successful!"); // payload was delivered
    printf("Tranmission time %lu ms\nSent: ", end_time - start_time);
    for (int i = 0; i < 32; i++)
    {
      printf("%02X", data[i]);
    }
    printf("\n");
  }
  return reported;
}

void nrf_pairing_report(char *_serial, int _channel)
{
  uint8_t buffer[32];
  PairingMessage msg = PairingMessage_init_zero;

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  strcpy(msg.serial, _serial);
  msg.channel = _channel;
  pb_encode(&stream, PairingMessage_fields, &msg);

  printf("\nPairing Msg : ");
  for (int i = 0; i < stream.bytes_written; i++)
  {
    data[i] = buffer[i];
    printf("%02X", data[i]);
  }
  printf("\n");

  data[31] = (uint8_t)stream.bytes_written;
}

bool nrf_pairing()
{
  uint8_t pipe;
  if (NRF24_availablePipe(&pipe))
  {
    uint8_t bytes = NRF24_getPayloadSize(); // get the size of the payload
    NRF24_read(&data, bytes);               // fetch payload from FIFO
    printf("Received %d \n", bytes);        // print the size of the payload
    printf(" bytes on pipe %d \n", pipe);
    for (int i = 0; i < bytes; i++)
    {
      printf("%02X", data[i]);
    }
    printf("\n");
    printf("\n");
    r_PairingMessage(data, bytes);
    printf("\n");
    return true;
  }
  return false;
}

void r_PairingMessage(uint8_t *_data, int _data_len)
{
  PairingMessage msg = PairingMessage_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(_data, _data_len);
  pb_decode(&stream, PairingMessage_fields, &msg);

  printf("DECODED: Serial: %s  Channel: %d\r\n", msg.serial, (int)msg.channel);

  Flash_Write_Data(FLASH_PAGE_ADDR , (uint32_t*)msg.serial, (sizeof(msg.serial)/sizeof(int)));
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
