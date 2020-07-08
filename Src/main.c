/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USE_GPIO 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void JV_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  JV_TIM1_Init();

  /* USER CODE BEGIN 2 */

  printf("\r\nPower On\r\n");
  printf("Compiled: " __DATE__ " " __TIME__ "\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#if USE_GPIO
  char bits[] = "00\r\n";
  while (1)
  {
      //bits[0] = HAL_GPIO_ReadPin(INPUT_A_GPIO_Port, INPUT_A_Pin) ? '1' : '0' ;
      //bits[1] = HAL_GPIO_ReadPin(INPUT_B_GPIO_Port, INPUT_B_Pin) ? '1' : '0' ;
      //printf(bits);
  }
#else
  while (1)
  {
      printf("%d\r\n", (int) (TIM1->CNT & 0xFF) );
  }
#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}
#if 0
/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
}
#endif


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GRN_LED_Pin|SPI2_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_A_Pin INPUT_B_Pin */
  GPIO_InitStruct.Pin = INPUT_A_Pin|INPUT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : GRN_LED_Pin SPI2_RST_Pin */
  GPIO_InitStruct.Pin = GRN_LED_Pin|SPI2_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_INT_Pin */
  GPIO_InitStruct.Pin = SPI2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief TIMER1 Initialization Function. Configure TIM1 for quadrature encoder on CH1 and CH2.
 * @param None
 * @retval None
 */
static void JV_TIM1_Init( void )
{
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE( );

    /* Config Register 1 (TIM1_CR1)
     * 0000 00.. .... .... : Reserved
     * .... ..10 .... .... : CKD: tDTS=4*tCK_INT
     * .... .... 0... .... : ARPE: TIMx_ARR register is not buffered
     * .... .... .00. .... : CMS: Edge-aligned mode
     * .... .... ...0 .... : DIR: Counter used as upcounter
     * .... .... .... 0... : OPM: Counter is not stopped at update event
     * .... .... .... .0.. : URS: Update generation events generate an update interrupt or DMA request
     * .... .... .... ..0. : UDIS:The Update (UEV) event generation is enabled
     * .... .... .... ...0 : CEN: Counter is disabled
     */
    TIM1->CR1 = 0x0200U;

    /*  Config Register 2 (TIM1_CR2)
     * 0... .... .... .... : Reserved
     * .0.. .... .... .... : OIS4:  OC4=0  after a dead-time when MOE=0
     * ..0. .... .... .... : OIS3N: OC3N=0 after a dead-time when MOE=0
     * ...0 .... .... .... : OIS3:  OC3=0  after a dead-time when MOE=0
     * .... 0... .... .... : OIS2N: OC2N=0 after a dead-time when MOE=0
     * .... .0.. .... .... : OIS2:  OC2=0  after a dead-time when MOE=0
     * .... ..0. .... .... : OIS1N: OC1N=0 after a dead-time when MOE=0
     * .... ...0 .... .... : OIS1:  OC1=0  after a dead-time when MOE=0
     * .... .... 0... .... : TI1S:  The TIMx_CH1 pin is connected to TI1 input
     * .... .... .000 .... : MMS:   Reset - the UG bit from the TIMx_EGR register is used as trigger output (TRGO).
     * .... .... .... 0... : CCDS:  CCx DMA request sent when CCx event occurs
     * .... .... .... .0.. : CCUS:  Capture/compare control update selection
     * .... .... .... ..0. : Reserved
     * .... .... .... ...0 : CCPC:  CCxE, CCxNE and OCxM bits are not preloaded
     */
    TIM1->CR2 = 0x0000U;

    /* Slave Mode Control Register (TIM1_SMCR)
     * 0... .... .... .... : ETP:  ETR is non-inverted, active at high level or rising edge
     * .0.. .... .... .... : ECE:  External clock mode 2 disabled
     * ..00 .... .... .... : ETPS: External Trigger Prescaler Off
     * .... 0000 .... .... : ETF:  No External Trigger Filter
     * .... .... 0... .... : MSM:  Master/Slave - no action
     * .... .... .000 .... : TS:   Internal Trigger 0 (ITR0)
     * .... .... .... 0... : OCCS: OCREF_CLR_INT is connected to the OCREF_CLR input
     * .... .... .... .001 : SMS:  Encoder mode 1 counts up/down on TI2FP1 edge depending on TI1FP2 level
     */
    TIM1->SMCR = 0x0001U;

    /* DMA/interrupt enable register (TIM1_DIER)
     * (enable none)
     */
    TIM1->DIER = 0x0000U;

    /* Status Register (TIM1_SR)
     * (clear all interrupt flags)
     */
    TIM1->SR = 0x0000U;

    /* Event Generation Register (TIM1_EGR)
     * (Not a config register)
     *TIM1->EGR   = 0x0000U;
     */

    /* Capture/Compare Mode Register 1 (TIM1_CCMR1)
     * 0101 .... .... .... : IC2F = fSAMPLING = fDTS / 2, N = 8
     * .... 00.. .... .... : IC2PSC = no prescaler, capture is done each time an edge
     * .... ..01 .... .... : CC2S = CC2 channel is configured as input, IC2 is mapped on TI2
     * .... .... 0101 .... : IC1F = fSAMPLING = fDTS / 2, N = 8
     * .... .... .... 00.. : IC1PSC = no prescaler, capture is done each time an edge
     * .... .... .... ..01 : CC1S = CC1 channel is configured as input, IC1 is mapped on TI1
     */
    TIM1->CCMR1 = 0x5151U;

    /* Capture/Compare Mode Register 2 (TIM1_CCMR2)
     * 0000 .... .... .... : IC4F = No filter, sampling is done at fDTS
     * .... 00.. .... .... : IC4PSC = no prescaler, capture is done each time an edge
     * .... ..01 .... .... : CC4S = CC4 channel is configured as input, IC4 is mapped on TI4
     * .... .... 0000 .... : IC3F = No filter, sampling is done at fDTS
     * .... .... .... 00.. : IC3PSC = no prescaler, capture is done each time an edge
     * .... .... .... ..01 : CC3S = CC3 channel is configured as input, IC3 is mapped on TI3
     */
    TIM1->CCMR2 = 0x0101U;

    /* Capture/Compare Enable Register (TIM1_CCER)
     * 00.. .... .... .... : Reserved
     * ..0. .... .... .... : CC4P:  input non-inverted/rising edge
     * ...0 .... .... .... : CC4E:  input capture disabled
     * .... 0... .... .... : CC3NP: input non-inverted/rising edge
     * .... .0.. .... .... : CC3NE: off; OC3N is not active
     * .... ..0. .... .... : CC3P:  input non-inverted/rising edge
     * .... ...0 .... .... : CC3E:  input capture disabled
     * .... .... 0... .... : CC2NP: input non-inverted/rising edge
     * .... .... .0.. .... : CC2NE: off; OC2N is not active
     * .... .... ..0. .... : CC2P:  input non-inverted/rising edge
     * .... .... ...0 .... : CC2E:  input capture disabled
     * .... .... .... 0... : CC1NP: input non-inverted/rising edge
     * .... .... .... .0.. : CC1NE: off; OC1N is not active
     * .... .... .... ..0. : CC1P:  input non-inverted/rising edge
     * .... .... .... ...0 : CC1E:  input capture disabled
     */
    TIM1->CCER = 0x0000U;

    /* Counter (TIM1_CNT)
     * (clear counter)
     */
    TIM1->CNT = 0x0000;

    /* Prescaler (TIM1_PSC)
     * counter clock frequency (CK_CNT) is equal to fCK_PSC / (PSC[15:0] + 1)
     */
    TIM1->PSC = 1U;

    /* Auto-Reload Register (TIM1_ARR)
     * (sets timer period)
     */
    TIM1->ARR = 0xFFFFU;

    /* Repetition Counter Register (TIM1_RCR)
     * (reset value)
     */
    TIM1->RCR = 0x0000U;

    /* Capture/Compare Registers 1-4 (TIM1_CCRx)
     * (rest value)
     */
    TIM1->CCR1 = 0x0000U;
    TIM1->CCR2 = 0x0000U;
    TIM1->CCR3 = 0x0000U;
    TIM1->CCR4 = 0x0000U;

    /* Break and Dead-Time Register (TIM1_BDTR)
     * 0... .... .... .... : MOE: OC and OCN outputs are disabled or forced to idle state.
     * .0.. .... .... .... : AOE: MOE can be set only by software
     * ..0. .... .... .... : BKP: Break input BRK is active low
     * ...0 .... .... .... : BKE: Break inputs (BRK and CCS clock failure event) disabled
     * .... 0... .... .... : OSSR: When inactive, OC/OCN outputs are disabled
     * .... .0.. .... .... : OSSI: When inactive, OC/OCN outputs are disabled
     * .... ..00 .... .... : LOCK: LOCK OFF - No bit is write protected
     * .... .... 0000 0000 : DTG: Dead Time = 0 * tDTS
     */
    TIM1->BDTR = 0x0000;

    /* DMA Control Register (TIM1_DCR)
     * (reset value)
     */
    TIM1->DCR = 0x0000;

    /* DMA Address for full transfer (TIM1_DMAR)
     * (reset value)
     */
    TIM1->DMAR = 0x0000;

    /*
     * Configure GPIOs for TIM1
     *   PA8 --> TIM1_CH1
     *   PA9 --> TIM1_CH2
     */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Enable Timer16 */
    TIM1->CR1 = 0x0201U;
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

  while(1)
  {
  }
  /* USER CODE END Erro_Handler_Debug */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
