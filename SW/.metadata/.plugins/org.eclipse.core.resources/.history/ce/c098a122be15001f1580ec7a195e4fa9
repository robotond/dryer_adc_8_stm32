/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utility.h"
#include <stdio.h>
#include <math.h>
#include "crc.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256	//Transmit buffer size
#define MAXBUFFER 256
#define ADC_SET_TIME 1

#define NUMSENSORS 8	//Number of sensors in a probe
#define RANGE_L -40                 // °C
#define RANGE_H 200                 // °C
#define BETA 3470                   // °K (Beta25/85)
#define NOMINAL_TEMPERATURE 298.15  // °K
#define INVALID_VAL 0xFF
#define AD_CORRECTION 0
//------------------------------------------------
#define KSMOOTH 10

#define NUM_RAW_DATA		((SCOLUMNS) * (SROWS))
#define NUM_TO_COMBINE		(1)
#define NUM_TEMPERATURES	(NUM_RAW_DATA / NUM_TO_COMBINE)
#define TEMP_OFFSET 		(31)
//------------------------------------------------

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

int adc_values[NUMSENSORS]; // Array to store values for NUMSENSORS channels
uint16_t temperatures_data[NUMSENSORS];
double NOMINAL_RESISTANCE = 10000.0;       // Nominal resistance at 25°C in ohms
double DIVIDER_RESISTANCE = 2200.0;       // Series resistor in ohms
double Vmeasured = 2.6;       // Voltage measured across the NTC in volts
double VREF = 3.335;             // ADC reference voltage in volts (assumed)
char tx_buffer[MAXBUFFER];
int rx_available;
char rx_buffer[RX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void calctemp() {
	// gets °C data from raw adc data
	for (int i = 0; i < NUMSENSORS; i++) {
		uint16_t value = adc_values[i] - AD_CORRECTION;
		double voltage = VREF / (double) 4096 * value;
		double ntc_resistance = voltage / (VREF - voltage) * DIVIDER_RESISTANCE;
		double temperature = (double) ntc_resistance
				/ (double) NOMINAL_RESISTANCE;
		temperature = log(temperature);
		temperature /= BETA;
		temperature += 1.0 / NOMINAL_TEMPERATURE;
		temperature = 1.0 / temperature;
		temperature -= 273.15;

		if (temperature < RANGE_L || temperature > RANGE_H) {
			temperature = INVALID_VAL;
		}

		temperatures_data[i] = temperature;
	}
}

int echo_u1(uint8_t escape_char) {
	uint8_t karcsi = ' ';
	int counter = 0;
	while (!(karcsi == escape_char)) {
		if (HAL_UART_Receive(&huart1, (uint8_t*) &karcsi, 1, SHORT_TIMEOUT)
				== HAL_OK) {
			// Echo the received character back
			HAL_UART_Transmit(&huart1, (uint8_t*) &karcsi, 1, HAL_MAX_DELAY);
			TOGGLE(BLUE1);
			counter++;
		}
	}
	return counter;
}
int echo_u3(uint8_t escape_char) {
	uint8_t karcsi = ' ';
	int counter = 0;
	while (!(karcsi == escape_char)) {
		if (HAL_UART_Receive(&huart3, (uint8_t*) &karcsi, 1, SHORT_TIMEOUT)
				== HAL_OK) {
			// Echo the received character back
			HAL_UART_Transmit(&huart3, (uint8_t*) &karcsi, 1, HAL_MAX_DELAY);
			TOGGLE(BLUE2);
			counter++;
		}
	}
	return counter;
}
//-------------------------------------------------------
void select_adc_channel(int channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
//////////    sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
	switch (channel) {
	case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_4;
		break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		break;
	case 6:
		sConfig.Channel = ADC_CHANNEL_6;
		break;
	case 7:
		sConfig.Channel = ADC_CHANNEL_7;
		break;
	default:
		sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	}
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
//----------------------------------------------

void convert() {
	int p;

	for (p = 0; p < NUMSENSORS; p++) {
		select_adc_channel(p);
		if (HAL_ADC_Start(&hadc1) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
			Error_Handler();
		}
		adc_values[p] = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

	}
}

//--------------------------------------------------------
void sendDataCal() {
	uint8_t packet_len = NUMSENSORS * 2;
	uint8_t packet[packet_len];
	for (int s = 0; s < NUMSENSORS; s++) {
		packet[s * 2 + 1] = temperatures_data[s] & 0xFF;
		packet[s * 2] = temperatures_data[s] >> 8;
	}

	uint16_t crc = calculateCRC16(packet, NUMSENSORS * 2);
	packet[packet_len - 2] = crc >> 8;
	packet[packet_len - 1] = crc & 0xFF;
	HAL_UART_Transmit(&huart3, packet, packet_len, 10);
}

//---------------------------------------------------
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
int length=0;
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
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	test_leds();
	ON(ENSPWR);
	HAL_Delay(100);
	convert();
	calctemp();
	convert();
	calctemp();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		convert();
//	  msglength=sprintf(tx_buffer,"ADC %04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d \r\n",adc_values[0],adc_values[1],adc_values[2],adc_values[3],adc_values[4],adc_values[5],adc_values[6],adc_values[7]);
//	  HAL_UART_Transmit(&huart3, (uint8_t *) tx_buffer,msglength, 100);
		calctemp();
//	  msglength=sprintf(tx_buffer,"TMP %04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d \r\n",temperatures_data[0],temperatures_data[1],temperatures_data[2],temperatures_data[3],temperatures_data[4],temperatures_data[5],temperatures_data[6],temperatures_data[7]);
//	  HAL_UART_Transmit(&huart3, (uint8_t *) tx_buffer,msglength, 100);
//------------------------------------------------------------------------
		// wait for rising edge
		//while (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin))
			;
		ON(GREEN);
//		read_all_sensors();	//ezt megteszi a convert()
		/* ez mar mukodott
		 for (int i = 0; i < NUM_RAW_DATA; i++){

		 adc_cal[i*2] = raw_adc_data[i] >> 8;

		 adc_cal[i*2 + 1] = raw_adc_data[i] & 0xFF;

		 }*//*
		 for (int i = 0; i < NUM_RAW_DATA; i++){

		 adc_cal[i*2] = calculated_temperatures[i] >> 8;

		 adc_cal[i*2 + 1] = calculated_temperatures[i] & 0xFF;

		 }
		 //memcpy(adc_cal,raw_adc_data,NUM_RAW_DATA*2);
		 // fill temperatures buffer
		 //fillTemp();

		 // send
		 //sendData();
		 */
		//sendDataCal();
		length=sprintf(tx_buffer,"%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d \r\n", temperatures_data[0], temperatures_data[1], temperatures_data[2], temperatures_data[3], temperatures_data[4], temperatures_data[5], temperatures_data[6], temperatures_data[7]);
		HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, length, 20);
		OFF(GREEN);
		// wait for falling edge
		//while (!HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin));
		HAL_Delay(5000);


//------------------------------------------------------------------------
		HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENSPWR_GPIO_Port, ENSPWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RED_Pin|GREEN_Pin|BLUE1_Pin|BLUE2_Pin
                          |BLUE3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : START_Pin */
  GPIO_InitStruct.Pin = START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(START_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENSPWR_Pin */
  GPIO_InitStruct.Pin = ENSPWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENSPWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_Pin GREEN_Pin BLUE1_Pin BLUE2_Pin
                           BLUE3_Pin */
  GPIO_InitStruct.Pin = RED_Pin|GREEN_Pin|BLUE1_Pin|BLUE2_Pin
                          |BLUE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
