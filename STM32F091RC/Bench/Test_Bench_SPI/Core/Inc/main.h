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
void GPIOInit(void);
void setAF(void);
void SPI1Init(void);
void L9963ETXX(uint8_t TX);
void L9963ERXX(uint8_t RX);

/* USER CODE BEGIN EFP */
void GPIOInit(void){
	// Enable GPIOS
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;


	// Enable Extras
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

}

void setAF(void){
	// TODO Configure PA4 (NCS) PA5 (SCK) PA6 (MISO) and PA7 (MOSI) as GPIO alternate function
	GPIOA->MODER &= ~((3 << (4 * 2)) | (3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2))); // Clear mode bits
	GPIOA->MODER |= (2 << (4 * 2)) | (2 << (5 * 2)) | (2 << (6 * 2)) | (2 << (7 * 2));  // Set to Alternate Function mode
	// Set alternate function to AF0 (SPI1)
	GPIOA->AFR[0] &= ~((0xF << (4 * 4)) | (0xF << (5 * 4)) | (0xF << (6 * 4)) | (0xF << (7 * 4))); // Clear AFRL bits and set to AF0

}

void SPI1Init(){
	// TODO Set configure CR1 and CR2
	// Configure SPI1
	    SPI1->CR1 |= (0u << 15) // MIDIMODE 0 full duplex 1 half duplex
	               | (0u << 14) // Bidirectional mode disabled 0 recive only 1 transmit only
	               | (0u << 13) // CRC Calcualtion disabled 0 disabled 1 enabled
	               | (0u << 12) // CRCNEXT 0 next transmit vlaue is from the Tx buffer 1 next transmit value is from the CRC register
	               | (0u << 11) // CRC Length 0 8-bit 1 16-bit
	               | (0u << 10) // RX Only 0 FULL DUPLEX 1 RX ONLY
	               | (1u << 9)  // SSM Software slave management enabled
	               | (1u << 8)  // SSI interal Slave Select
	               | (0u << 7)  // LSB FIRST 0 MSB 1 LSB
	               | (4u << 3)  // Baud rate control: fPCLK/X
	               | (1u << 2)  // Master Selection
	               | (1u << 1) // Clock Polarity
	               | (1u << 0); // Clock Phase

	    SPI1->CR2 |= (0u << 14) // LDMA_TX 0 number of data to transfer is even 1 odd
	               | (0u << 13) // LEMA)RX 0 number of data to transfer is even 1 odd
	               | (0u << 12) // FRXTH 0 RXNE event is generated if the FIFO level is greater than or equal to 1/2 1 RXNE event is generated if the FIFO level is greater than or equal to 1/4
	               | (0u << 8) // DATA Size PG 817
	               | (0u << 7) // TXEIE 0 TXE interupt Masked 1 TXE interrupt not masked
	               | (0u << 6) // RXNEIE 0 RXNE interrupt Masked 1 RXNE interrupt not masked
	               | (0u << 5) // ERRIE 0 Error interrupt Masked 1 Error interrupt is enabled
	               | (0u << 4) // FRF 0 Motorola 1 TI
	               | (0u << 3) // NSSP 0 no NSS Pulse 1 NSS pulse generated
	               | (0u << 2) // SSOE 0  SS output disabled 1 SS output enabled
	               | (0u << 1) // TXDMAEN 0 TX DMA disabled 1 TX DMA enabled
	               | (0u << 0); // RXDMAEN 0 RX DMA disabled 1 RX DMA enabled
	// Enable SPI peripheral
	SPI1->CR1 |= (1u << 6);  // SPI enabled

}

void L9963ETXX(uint8_t TX){
	// TODO Figure out the the Data size and change the second X in the function name and the uint size

	// wait till buffer is empty
	while (!(SPI1->SR & SPI_SR_TXE));
	// TODO TX

	// wait for transmission to complete
	while (SPI1->SR & SPI_SR_BSY);
}

void L9963ERXX(uint8_t RX){
	// TODO Figure out the the Data size and change the second X in the function name and the uint size

	// wait till buffer is empty
	while (!(SPI1->SR & SPI_SR_RXE));
	// TODO TX

	// wait for transmission to complete
	while (SPI1->SR & SPI_SR_BSY);



}

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
