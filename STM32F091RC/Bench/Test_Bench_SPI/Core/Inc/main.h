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
#include "stm32f0xx_hal.h"

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

// Functions Prototypes
void GPIOInit(void);
void setAF(void);
void SPI1Init(void);
void L9963ETXX(uint8_t TX);
void L9963ERXX(uint8_t RX);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

// Pin Defines
#define SPI1_PORT GPIOA

#define SPI1_CS_PORT GPIOA
#define SPI1_CS_PIN 4

#define SPI1_SCK_PORT GPIOA
#define SPI1_SCK_PIN 5

#define SPI1_MISO_PORT GPIOA
#define SPI1_MISO_PIN 6

#define SPI1_MOSI_PORT GPIOA
#define SPI1_MOSI_PIN 7

// --- DEVICE CONSTANTS ---
#define L9963E_FRAME_BITS   20      // The L9963E uses a 20-bit frame
#define L9963E_WRITE        0       // R/W bit for a write operation (bit 19)
#define L9963E_READ         1       // R/W bit for a read operation (bit 19)

// --- FRAME STRUCTURE MACROS (Conceptual: R/W(1)|ADDR(8)|DATA(11) ) ---
// Note: This assumes the L9963E uses a fixed 4-bit CRC/parity within the 20-bit frame
// or a simplified structure. The actual frame structure must be verified from the datasheet.
#define L9963E_ADDRESS_MASK 0xFF
#define L9963E_DATA_MASK    0x7FF   // 11 bits of data
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
