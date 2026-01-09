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
#define SPI1_DIS_Pin GPIO_PIN_4
#define SPI1_DIS_GPIO_Port GPIOC
#define SPIl_ISOFREO_Pin GPIO_PIN_5
#define SPIl_ISOFREO_GPIO_Port GPIOC
#define SPIl_BNE_CPOL_Pin GPIO_PIN_0
#define SPIl_BNE_CPOL_GPIO_Port GPIOB
#define SPIl_NSLAVE_Pin GPIO_PIN_1
#define SPIl_NSLAVE_GPIO_Port GPIOB
#define SPIl_TXEN_CPHA_Pin GPIO_PIN_2
#define SPIl_TXEN_CPHA_GPIO_Port GPIOB
#define SPIl_CLKFREQ_Pin GPIO_PIN_10
#define SPIl_CLKFREQ_GPIO_Port GPIOB
#define SPIl_TXAMP_Pin GPIO_PIN_11
#define SPIl_TXAMP_GPIO_Port GPIOB
#define SPI2_TXAMP_Pin GPIO_PIN_6
#define SPI2_TXAMP_GPIO_Port GPIOC
#define SPI2_CLKFREQ_Pin GPIO_PIN_7
#define SPI2_CLKFREQ_GPIO_Port GPIOC
#define SPI2_TXEN_CPHA_Pin GPIO_PIN_8
#define SPI2_TXEN_CPHA_GPIO_Port GPIOC
#define SPI2_NSLAVE_Pin GPIO_PIN_9
#define SPI2_NSLAVE_GPIO_Port GPIOC
#define SPI2_BNE_CPOL_Pin GPIO_PIN_8
#define SPI2_BNE_CPOL_GPIO_Port GPIOA
#define SPI2_ISOFREQ_Pin GPIO_PIN_9
#define SPI2_ISOFREQ_GPIO_Port GPIOA
#define SPI2_DIS_Pin GPIO_PIN_10
#define SPI2_DIS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
