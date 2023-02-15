/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NS  128
#define BUFFER_SIZE  128
#define FLOAT_TO_INT16 32768.0f
#define FLOAT_TO_INT12 2047.5f
//#define FLOAT_TO_INT12 1024.0f
//1024.0f//
#define INT12_TO_FLOAT 1.0f / (2047.5f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t Wave_LUT[NS] = { 2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837,
		2929, 3020, 3108, 3193, 3275, 3355, 3431, 3504, 3574, 3639, 3701, 3759,
		3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076, 4087, 4094, 4095,
		4091, 4082, 4069, 4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730,
		3671, 3607, 3539, 3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790,
		2695, 2598, 2500, 2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595,
		1497, 1400, 1305, 1212, 1120, 1031, 944, 860, 779, 701, 627, 556, 488,
		424, 365, 309, 258, 211, 168, 130, 97, 69, 45, 26, 13, 4, 0, 1, 8, 19,
		35, 56, 82, 113, 149, 189, 234, 283, 336, 394, 456, 521, 591, 664, 740,
		820, 902, 987, 1075, 1166, 1258, 1353, 1449, 1546, 1645, 1745, 1845,
		1946, 2047 };
uint32_t dacData[NS] = { 2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837,
		2929, 3020, 3108, 3193, 3275, 3355, 3431, 3504, 3574, 3639, 3701, 3759,
		3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076, 4087, 4094, 4095,
		4091, 4082, 4069, 4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730,
		3671, 3607, 3539, 3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790,
		2695, 2598, 2500, 2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595,
		1497, 1400, 1305, 1212, 1120, 1031, 944, 860, 779, 701, 627, 556, 488,
		424, 365, 309, 258, 211, 168, 130, 97, 69, 45, 26, 13, 4, 0, 1, 8, 19,
		35, 56, 82, 113, 149, 189, 234, 283, 336, 394, 456, 521, 591, 664, 740,
		820, 902, 987, 1075, 1166, 1258, 1353, 1449, 1546, 1645, 1745, 1845,
		1946, 2047 };
static volatile uint32_t *outBufPtr = &dacData[0];
uint8_t dataReadyFlag;
double deltaOmega = 0.0f;
double omega = 0.0f;
uint8_t timesCalled = 0;
uint16_t adcControlData[1];
float potValue;
float frequency;
double period;
float (*signalFunc)(float);
char resetButton = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_DACEx_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
//	outBufPtr = &dacData[0];
//	dataReadyFlag = 1;
}
void HAL_DACEx_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
//		outBufPtr = &dacData[0];
//		dataReadyFlag = 1;
//	outBufPtr = &dacData[BUFFER_SIZE / 2];
//	dataReadyFlag = 1;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	outBufPtr = &dacData[0];
	dataReadyFlag = 1;
}
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
//		outBufPtr = &dacData[0];
//		dataReadyFlag = 1;
	outBufPtr = &dacData[BUFFER_SIZE / 2];
	dataReadyFlag = 1;
}

//void HAL_DAC_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
//	outBufPtr = &dacData[BUFFER_SIZE / 2];
//	dataReadyFlag = 1;
//}

//void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
////	updateCh1Part2 = 1;
//	outBufPtr = &dacData[0];
//	dataReadyFlag = 1;
//	timesCalled++;
//}
//void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
////	updateCh1Part2 = 1;
////	uint16_t output = sin(M_TWOPI * 440.0f * ms);
//	outBufPtr = &dacData[BUFFER_SIZE / 2];
//	dataReadyFlag = 1;
//	timesCalled++;
//}
// Called when buffer is completely filled
float freqRange = 10.0f - 0.0f;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	potValue = ((float) (4095 - adcControlData[0])) / 4095.0f;
	frequency = potValue * freqRange;
	setFrequency(frequency);
}

//void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
//	timesCalled++;
//}
//void HAL_DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef *hdac) {
//	timesCalled++;
//}
//void HAL_DACEx_ErrorCallbackCh2(DAC_HandleTypeDef *hdac) {
//	timesCalled++;
//}
float sine(float phase) {
//	return 0.0f;
	return sinf(phase);
}

float square(float phase) {
	if (phase <= M_PI) {
		return 1.0;
	} else {
		return -1.0f;
	}
}

float sawtoothUp(float phase) {
	return 1.0 - 2.0 * (phase * (1.0 / M_TWOPI));
}

float sawtoothDown(float phase) {
	return (2.0 * (phase * (1.0 / M_TWOPI))) - 1.0;
}

float triangle(float phase) {
	float value = (2.0 * (phase * (1.0 / M_TWOPI))) - 1.0;
	if (value < 0.0) {
		value = -value;
	}
	return 2.0 * (value - 0.5);
}

void setFrequency(double frequency) {
	deltaOmega = frequency / 48000.0f;
}

void processData() {
//	static float potValue;


	for (uint8_t n = 0; n < BUFFER_SIZE / 2; n++) {

//		HAL_GetTick()
//		float output = sin(M_TWOPI * freq * ((float) HAL_GetTick()) / 1000.0f);
//		potValue = ((float) adcControlData[0]) / 4095.0f;
//
//		freq = potValue * freqRange + 0.0f;
//		period = 1 / freq;

//		float output = sin(
//		M_TWOPI * freq * (float) HAL_GetTick() / 1000.0f);
//		double seconds = tick / 24000.0f;

		float output = signalFunc(M_TWOPI * omega);
//		float output = sin(M_TWOPI * (float) n / ((float) BUFFER_SIZE / 2.0f));

		output += 1.0f;
//		if (output > 2.0f) {
//			output = 2.0f;
//		}
//		if (output < 0.0f) {
//			output = 0.0f;
//		}

		outBufPtr[n] = (uint32_t) (FLOAT_TO_INT12 * output);
//		outBufPtr[n] = 4095;

//		tick += 1.0f;
//		if (seconds >= period) {
//			tick = 0.0f;
//		}
		omega += deltaOmega;
		if (omega >= 1.0) { omega -= 1.0; }
	}

	dataReadyFlag = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char uart_buf[50];
	int uart_buf_len;
	uint16_t timer_val;

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
//  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState)
//	HAL_TIM_Base_Start(&htim1);
//	HAL_TIM_Base_Stop_IT(&htim1);
// Say something
//	uart_buf_len = sprintf(uart_buf, "Timer Test\r\n");
//	HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, uart_buf_len, 100);
//	HAL_TIM_Base_Start(&htim2);
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_Base_Start(&htim6);
	signalFunc = sine;
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) dacData,
	BUFFER_SIZE,
	DAC_ALIGN_12B_R);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcControlData, 1);
	HAL_TIM_Base_Start(&htim2);

	uint32_t last_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (dataReadyFlag) {
			processData();
		}
		int stateOfPushButton = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		if (stateOfPushButton == 1) {
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			resetButton = 0;
		} else if (resetButton == 0) {
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			if (signalFunc == sine) {
				signalFunc = square;
			} else if (signalFunc == square) {
				signalFunc = sawtoothUp;
			} else if (signalFunc == sawtoothUp) {
				signalFunc = sawtoothDown;
			} else if (signalFunc == sawtoothDown) {
				signalFunc = triangle;
			} else if (signalFunc == triangle) {
				signalFunc = sine;
			}

			resetButton = 1;
		}
//		if (HAL_GetTick() - last_time > 1000) {
//			uart_buf_len = sprintf(uart_buf, "%f pot data\r\n", freq);
//			HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, uart_buf_len, 100);
////			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//			last_time = HAL_GetTick();
//		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 28800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1500-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
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
	while (1) {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
