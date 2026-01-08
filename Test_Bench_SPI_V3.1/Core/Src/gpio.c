/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L9963TL_NCS_GPIO_OUT_Pin|L9963TH_ISOFREQ_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L9963TL_DIS_GPIO_INOUT_Pin|L9963TL_ISOFREQ_GPIO_OUT_Pin|TXAMP2_Pin|CLKFREQ2_Pin
                          |L9963TH_TXEN_GPIO_OUT_Pin|NSLAVE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSLAVE1_Pin|L9963TL_TXEN_GPIO_OUT_Pin|CLKFREQ1_Pin|TXAMP1_Pin
                          |L9963TH_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L9963TL_NCS_GPIO_OUT_Pin L9963TH_ISOFREQ_GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = L9963TL_NCS_GPIO_OUT_Pin|L9963TH_ISOFREQ_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L9963TL_DIS_GPIO_INOUT_Pin L9963TL_ISOFREQ_GPIO_OUT_Pin TXAMP2_Pin CLKFREQ2_Pin
                           L9963TH_TXEN_GPIO_OUT_Pin NSLAVE2_Pin */
  GPIO_InitStruct.Pin = L9963TL_DIS_GPIO_INOUT_Pin|L9963TL_ISOFREQ_GPIO_OUT_Pin|TXAMP2_Pin|CLKFREQ2_Pin
                          |L9963TH_TXEN_GPIO_OUT_Pin|NSLAVE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : L9963TL_BNE_GPIO_IN_Pin */
  GPIO_InitStruct.Pin = L9963TL_BNE_GPIO_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(L9963TL_BNE_GPIO_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSLAVE1_Pin L9963TL_TXEN_GPIO_OUT_Pin CLKFREQ1_Pin TXAMP1_Pin
                           L9963TH_NCS_GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = NSLAVE1_Pin|L9963TL_TXEN_GPIO_OUT_Pin|CLKFREQ1_Pin|TXAMP1_Pin
                          |L9963TH_NCS_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : L9963TH_BNE_GPIO_IN_Pin */
  GPIO_InitStruct.Pin = L9963TH_BNE_GPIO_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(L9963TH_BNE_GPIO_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L9963TH_DIS_GPIO_INOUT_Pin */
  GPIO_InitStruct.Pin = L9963TH_DIS_GPIO_INOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(L9963TH_DIS_GPIO_INOUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
