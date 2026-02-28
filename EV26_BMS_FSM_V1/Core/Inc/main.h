/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DHAB_HIGH_ADC_IN_Pin GPIO_PIN_0
#define DHAB_HIGH_ADC_IN_GPIO_Port GPIOA
#define DHAB_LOW_ADC_IN_Pin GPIO_PIN_2
#define DHAB_LOW_ADC_IN_GPIO_Port GPIOA
#define L9963T_NCS_GPIO_OUT_Pin GPIO_PIN_12
#define L9963T_NCS_GPIO_OUT_GPIO_Port GPIOB
#define L9963T_SCK_GPIO_OUT_Pin GPIO_PIN_13
#define L9963T_SCK_GPIO_OUT_GPIO_Port GPIOB
#define L9963T_MISO_GPIO_OUT_Pin GPIO_PIN_14
#define L9963T_MISO_GPIO_OUT_GPIO_Port GPIOB
#define L9963T_MOSI_GPIO_OUT_Pin GPIO_PIN_15
#define L9963T_MOSI_GPIO_OUT_GPIO_Port GPIOB
#define L9963T_TXAMP_GPIO_OUT_Pin GPIO_PIN_6
#define L9963T_TXAMP_GPIO_OUT_GPIO_Port GPIOC
#define L9963T_CLKFREQ_GPIO_OUT_Pin GPIO_PIN_7
#define L9963T_CLKFREQ_GPIO_OUT_GPIO_Port GPIOC
#define L9963T_TXEN_GPIO_OUT_Pin GPIO_PIN_8
#define L9963T_TXEN_GPIO_OUT_GPIO_Port GPIOC
#define L9963T_NSLAVE_GPIO_OUT_Pin GPIO_PIN_9
#define L9963T_NSLAVE_GPIO_OUT_GPIO_Port GPIOC
#define L9963T_BNE_GPIO_INPUT_Pin GPIO_PIN_8
#define L9963T_BNE_GPIO_INPUT_GPIO_Port GPIOA
#define L9963T_ISOFREQ_GPIO_OUT_Pin GPIO_PIN_9
#define L9963T_ISOFREQ_GPIO_OUT_GPIO_Port GPIOA
#define L9963T_DIS_GPIO_INOUT_Pin GPIO_PIN_10
#define L9963T_DIS_GPIO_INOUT_GPIO_Port GPIOA
#define EEPROM_SCK_GPIO_OUT_Pin GPIO_PIN_10
#define EEPROM_SCK_GPIO_OUT_GPIO_Port GPIOC
#define EEPROM_MISO_GPIO_IN_Pin GPIO_PIN_11
#define EEPROM_MISO_GPIO_IN_GPIO_Port GPIOC
#define EEPROM_MOSI_GPIO_OUT_Pin GPIO_PIN_12
#define EEPROM_MOSI_GPIO_OUT_GPIO_Port GPIOC
#define EEPROM_NCS_GPIO_OUT_Pin GPIO_PIN_2
#define EEPROM_NCS_GPIO_OUT_GPIO_Port GPIOD
#define EEPROM_NHold_GPIO_OUT_Pin GPIO_PIN_4
#define EEPROM_NHold_GPIO_OUT_GPIO_Port GPIOB
#define Shutdown_Ctl_Pin GPIO_PIN_6
#define Shutdown_Ctl_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
