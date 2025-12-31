/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_encode.h"
#include "can_decode.h"
#include "DHAB124.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t adc_buffer[ADC_BUF_LEN];
float current;


// BMS Status Variables
float packVolts, packAmps,soc;
uint8_t lowVolt, highVolt, lowTemp, highTemp, imbalance, dischargeOK, chargeOK, faulted, data;


//CAN Variables
CAN_TxHeaderTypeDef   TxHeader; /* Header containing the information of the transmitted frame */
CAN_RxHeaderTypeDef   RxHeader; ; /* Header containing the information of the received frame */
uint8_t               TxData[8] = {0};  /* Buffer of the data to send */
uint8_t               RxData[8]; /* Buffer of the received data */
uint32_t              TxMailbox;  /* The number of the mail box that transmitted the Tx message */
/* USER CODE END Variables */
/* Definitions for BMSPrimaryMSG */
osThreadId_t BMSPrimaryMSGHandle;
const osThreadAttr_t BMSPrimaryMSG_attributes = {
  .name = "BMSPrimaryMSG",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Fault */
osThreadId_t FaultHandle;
const osThreadAttr_t Fault_attributes = {
  .name = "Fault",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CANRXMSG */
osThreadId_t CANRXMSGHandle;
const osThreadAttr_t CANRXMSG_attributes = {
  .name = "CANRXMSG",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for BMSSecondaryMSG */
osThreadId_t BMSSecondaryMSGHandle;
const osThreadAttr_t BMSSecondaryMSG_attributes = {
  .name = "BMSSecondaryMSG",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBMSPrimaryMSG(void *argument);
void StartFault(void *argument);
void StartCANRXMSG(void *argument);
void StartBMSSecondaryMSG(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BMSPrimaryMSG */
  BMSPrimaryMSGHandle = osThreadNew(StartBMSPrimaryMSG, NULL, &BMSPrimaryMSG_attributes);

  /* creation of Fault */
  FaultHandle = osThreadNew(StartFault, NULL, &Fault_attributes);

  /* creation of CANRXMSG */
  CANRXMSGHandle = osThreadNew(StartCANRXMSG, NULL, &CANRXMSG_attributes);

  /* creation of BMSSecondaryMSG */
  BMSSecondaryMSGHandle = osThreadNew(StartBMSSecondaryMSG, NULL, &BMSSecondaryMSG_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBMSPrimaryMSG */
/**
  * @brief  Function implementing the BMSPrimaryMSG thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBMSPrimaryMSG */
void StartBMSPrimaryMSG(void *argument)
{
  /* USER CODE BEGIN StartBMSPrimaryMSG */
  /* Infinite loop */
  for(;;)
  {
	//TODO Read Modules
	//TODO Read Current
	//TODO SEND STATUS Message
		// BMS Status Variables
		//float packVolts, packAmps,soc;
		//uint8_t lowVolt, highVolt, lowTemp, highTemp, imbalance, dischargeOK, chargeOK, faulted, data;
	//TODO PACK FAULTED
	  if(){//Voltage reads too high or too low, Thermistor reads to high, or not reading a module or thermisor
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET);
	  faulted = 1;
	  }
	  Encode_BMS_STATUS(packVolts, packAmps, soc,
					  lowVolt, highVolt, lowTemp,
					  highTemp, imbalance, dischargeOK,
					  chargeOK, faulted, TxData);
	  TxHeader.StdId = 1542; // Update CAN ID
	  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) Error_Handler();
  }
  /* USER CODE END StartBMSPrimaryMSG */
}

/* USER CODE BEGIN Header_StartFault */
/**
* @brief Function implementing the Fault thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFault */
void StartFault(void *argument)
{
  /* USER CODE BEGIN StartFault */
  /* Infinite loop */
  for(;;)
  {
	//TODO DEFINE FALUT

	//TODO Read Modules
	//TODO Read Current
	//TODO SEND BMS STATUS MESSAGE
		// BMS Status Variables
		//float packVolts, packAmps,soc;
		//uint8_t lowVolt, highVolt, lowTemp, highTemp, imbalance, dischargeOK, chargeOK, faulted, data;
	//TODO FAULT CLEARED
	  if(){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET);
	  	  faulted = 0;
	  }
	//

  }
  /* USER CODE END StartFault */
}

/* USER CODE BEGIN Header_StartCANRXMSG */
/**
* @brief Function implementing the CANRXMSG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANRXMSG */
void StartCANRXMSG(void *argument)
{
  /* USER CODE BEGIN StartCANRXMSG */
  /* Infinite loop */
  for(;;)
  {
	//TODO RX message
	//TODO INVERTER_CTRL
	//TODO SYS_STATUS
    osDelay(1);
  }
  /* USER CODE END StartCANRXMSG */
}

/* USER CODE BEGIN Header_StartBMSSecondaryMSG */
/**
* @brief Function implementing the BMSSecondaryMSG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBMSSecondaryMSG */
void StartBMSSecondaryMSG(void *argument)
{
  /* USER CODE BEGIN StartBMSSecondaryMSG */
  /* Infinite loop */
  for(;;)
  {
	//TODO BMS_QUADVOLTS
	  Encode_BMS_QUADVOLTS();
	//TODO BMS_CELLVOLTS
	  Encode_BMS_CELLVOLTS();
	//TODO BMS_TEMPS
	  Encode_BMS_TEMPS();


    osDelay(1);
  }
  /* USER CODE END StartBMSSecondaryMSG */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

