/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define L9963TL_NCS_GPIO_OUT_Pin GPIO_PIN_4
#define L9963TL_NCS_GPIO_OUT_GPIO_Port GPIOA
#define L9963TL_DIS_GPIO_INOUT_Pin GPIO_PIN_4
#define L9963TL_DIS_GPIO_INOUT_GPIO_Port GPIOC
#define L9963TL_ISOFREQ_GPIO_OUT_Pin GPIO_PIN_5
#define L9963TL_ISOFREQ_GPIO_OUT_GPIO_Port GPIOC
#define L9963TL_BNE_GPIO_IN_Pin GPIO_PIN_0
#define L9963TL_BNE_GPIO_IN_GPIO_Port GPIOB
#define NSLAVE1_Pin GPIO_PIN_1
#define NSLAVE1_GPIO_Port GPIOB
#define L9963TL_TXEN_GPIO_OUT_Pin GPIO_PIN_2
#define L9963TL_TXEN_GPIO_OUT_GPIO_Port GPIOB
#define CLKFREQ1_Pin GPIO_PIN_10
#define CLKFREQ1_GPIO_Port GPIOB
#define TXAMP1_Pin GPIO_PIN_11
#define TXAMP1_GPIO_Port GPIOB
#define L9963TH_NCS_GPIO_OUT_Pin GPIO_PIN_12
#define L9963TH_NCS_GPIO_OUT_GPIO_Port GPIOB
#define TXAMP2_Pin GPIO_PIN_6
#define TXAMP2_GPIO_Port GPIOC
#define CLKFREQ2_Pin GPIO_PIN_7
#define CLKFREQ2_GPIO_Port GPIOC
#define L9963TH_TXEN_GPIO_OUT_Pin GPIO_PIN_8
#define L9963TH_TXEN_GPIO_OUT_GPIO_Port GPIOC
#define NSLAVE2_Pin GPIO_PIN_9
#define NSLAVE2_GPIO_Port GPIOC
#define L9963TH_BNE_GPIO_IN_Pin GPIO_PIN_8
#define L9963TH_BNE_GPIO_IN_GPIO_Port GPIOA
#define L9963TH_ISOFREQ_GPIO_OUT_Pin GPIO_PIN_9
#define L9963TH_ISOFREQ_GPIO_OUT_GPIO_Port GPIOA
#define L9963TH_DIS_GPIO_INOUT_Pin GPIO_PIN_10
#define L9963TH_DIS_GPIO_INOUT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
