/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI1_DIS_Pin|SPIl_ISOFREO_Pin|SPI2_TXAMP_Pin|SPI2_CLKFREQ_Pin
                          |SPI2_TXEN_CPHA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPIl_BNE_CPOL_Pin|SPIl_TXEN_CPHA_Pin|SPIl_CLKFREQ_Pin|SPIl_TXAMP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI2_BNE_CPOL_Pin|SPI2_ISOFREQ_Pin|SPI2_DIS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_DIS_Pin SPIl_ISOFREO_Pin SPI2_TXAMP_Pin SPI2_CLKFREQ_Pin
                           SPI2_TXEN_CPHA_Pin */
  GPIO_InitStruct.Pin = SPI1_DIS_Pin|SPIl_ISOFREO_Pin|SPI2_TXAMP_Pin|SPI2_CLKFREQ_Pin
                          |SPI2_TXEN_CPHA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPIl_BNE_CPOL_Pin SPIl_TXEN_CPHA_Pin SPIl_CLKFREQ_Pin SPIl_TXAMP_Pin */
  GPIO_InitStruct.Pin = SPIl_BNE_CPOL_Pin|SPIl_TXEN_CPHA_Pin|SPIl_CLKFREQ_Pin|SPIl_TXAMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPIl_NSLAVE_Pin */
  GPIO_InitStruct.Pin = SPIl_NSLAVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPIl_NSLAVE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_NSLAVE_Pin */
  GPIO_InitStruct.Pin = SPI2_NSLAVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_NSLAVE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_BNE_CPOL_Pin SPI2_ISOFREQ_Pin SPI2_DIS_Pin */
  GPIO_InitStruct.Pin = SPI2_BNE_CPOL_Pin|SPI2_ISOFREQ_Pin|SPI2_DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
