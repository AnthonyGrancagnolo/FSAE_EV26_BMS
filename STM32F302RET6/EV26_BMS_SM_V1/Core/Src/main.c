/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "can_encode.h"
#include "can_decode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUGUart

enum states{
	INITIAL,
	IDLE_NEUTRAL,
	FAULT,
	CHARGING,
	DISCHARGING,

} state;

//enum events{

//};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool SHUTDOWN;
bool VOLTAGE_NSAFE;
bool THERM_NSAFE;
bool CURRENT_NSAFE;

CAN_TxHeaderTypeDef   TxHeader; /* Header containing the information of the transmitted frame */
CAN_RxHeaderTypeDef   RxHeader; ; /* Header containing the information of the received frame */
uint8_t               TxData[8] = {0};  /* Buffer of the data to send */
uint8_t               RxData[8]; /* Buffer of the received data */
uint32_t              TxMailbox;  /* The number of the mail box that transmitted the Tx message */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //CAN Transmission settings
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  // TODO Make sure Voltages and thermals can be read.

  // TODO Check if Systems are ready to discharge
  // SET shutdown loop to
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);
  SHUTDOWN = false;
  THERM_NSAFE = false;
  VOLTAGE_NSAFE = false;
  CURRENT_NSAFE = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // TODO ADC

	  // TODO DMA

	  // TODO SPI
	  // Send burst command to get all voltage and thermal levels

	  // Store the values in a structure


	  // TODO If Voltages or thermistors are out of line
	  if(VOLTAGE_NSAFE || THERM_NSAFE){ // Voltages are above X or below Y
		  SHUTDOWN = true;
	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);
	  }
	  // TODO SHUTDOWN

	  while(SHUTDOWN){

		  // TODO send CAN Messages to diag the problem only faults
		  // TODO Check if fault is clear
		  // TODO send SPI Burst command to check voltages
		  // TODO if pack returns to a safe state
		  if(VOLTAGE_NSAFE && THERM_NSAFE){
			  SHUTDOWN = false;
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);
		  }
	  }

	  // TODO CAN send Voltages and Thermal Measurements over CAN (Pack Voltage, current draw,
		// Encode BMS_STATUS
		/* =================== BMS_STATUS =================== */
		Encode_BMS_STATUS(
			350.5f,    // PACKVOLTS
			-120.2f,   // PACKAMPS
			76.0f,     // STATEOFCHARGE
			0, 0, 0, 0, 0, 1, 1, 0, // status bits
			TxData
			);
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
		if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) Error_Handler();

		/* =================== BMS_QUADVOLTS =================== */
		Encode_BMS_QUADVOLTS(
			12.34f, 12.45f, 12.56f, 12.67f, // QUAD1-4
			TxData
			);
		TxHeader.StdId = 1543; // Update CAN ID
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
		if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) Error_Handler();

		/* =================== BMS_CELLVOLTS =================== */
		Encode_BMS_CELLVOLTS(
			3.456f, 3.467f, 3.478f, 3.489f, // AVGVOLT1-4
			TxData
			);
		TxHeader.StdId = 1544;
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
		if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) Error_Handler();

		/* =================== BMS_TEMPS =================== */
		Encode_BMS_TEMPS(
			25.0f, 26.0f, 24.5f, 25.5f, // TEMP1-4
			TxData
			);
		TxHeader.StdId = 1545;
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
		if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) Error_Handler();

		/* Wait 100 ms → ~10 Hz overall loop */
		HAL_Delay(100);
	  // TODO EEPROM
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;


    // Get the incoming message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, RxData) != HAL_OK)
    {
        Error_Handler();
    }

    // Decode based on StdId
    switch(RxHeader.StdId)
    {
        case 1542: // BMS_STATUS
        {
            BMS_STATUS_t bmsStatus;
            Decode_BMS_STATUS(RxData, &bmsStatus);

            // Example usage:
            // float volts = bmsStatus.PACKVOLTS;
            // uint8_t fault = bmsStatus.FAULTED;
            break;
        }

        case 1543: // BMS_QUADVOLTS
        {
            BMS_QUADVOLTS_t quadVolts;
            Decode_BMS_QUADVOLTS(RxData, &quadVolts);

            // Example usage:
            // float q1 = quadVolts.QUAD1;
            break;
        }

        case 1544: // BMS_CELLVOLTS
        {
            BMS_CELLVOLTS_t cellVolts;
            Decode_BMS_CELLVOLTS(RxData, &cellVolts);
            break;
        }

        case 1545: // BMS_TEMPS
        {
            BMS_TEMPS_t bmsTemps;
            Decode_BMS_TEMPS(RxData, &bmsTemps);
            break;
        }

        case 516: // INVERTER_CTRL
        {
            INVERTER_CTRL_t inverterCtrl;
            Decode_INVERTER_CTRL(RxData, &inverterCtrl);
            break;
        }

        case 521: // SYS_STATUS
        {
            SYS_STATUS_t sysStatus;
            Decode_SYS_STATUS(RxData, &sysStatus);
            break;
        }

        default:
            // Unknown CAN ID — ignore or log
            break;
    }

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
