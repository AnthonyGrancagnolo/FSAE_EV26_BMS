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
/* PRECHARGE COMMAND MACROS */
#define Close_Precharge() \
    HAL_GPIO_WritePin(    \
        uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_GPIO_Port, uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_Pin, GPIO_PIN_SET)
#define Open_Precharge() \
    HAL_GPIO_WritePin(   \
        uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_GPIO_Port, uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_Pin, GPIO_PIN_RESET)
/* AIRs COMMAND MACROS */
#define Close_Air_Pos() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_AIR_POS_COMMAND_GPIO_Port, uC_GPIO_OUT_AIR_POS_COMMAND_Pin, GPIO_PIN_SET)
#define Open_Air_Pos() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_AIR_POS_COMMAND_GPIO_Port, uC_GPIO_OUT_AIR_POS_COMMAND_Pin, GPIO_PIN_RESET)
#define Close_Air_Neg() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_AIR_NEG_COMMAND_GPIO_Port, uC_GPIO_OUT_AIR_NEG_COMMAND_Pin, GPIO_PIN_SET)
#define Open_Air_Neg() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_AIR_NEG_COMMAND_GPIO_Port, uC_GPIO_OUT_AIR_NEG_COMMAND_Pin, GPIO_PIN_RESET)
/* AMS ERROR MACROS */
#define Set_AMS_Error()   HAL_GPIO_WritePin(AMS_ERROR_GPIO_Port, AMS_ERROR_Pin, GPIO_PIN_SET)
#define Reset_AMS_Error() HAL_GPIO_WritePin(AMS_ERROR_GPIO_Port, AMS_ERROR_Pin, GPIO_PIN_RESET)
/* LED MACROS */
#define Warn_LED_On()   HAL_GPIO_WritePin(WARN_LED_GPIO_OUT_GPIO_Port, WARN_LED_GPIO_OUT_Pin, GPIO_PIN_SET)
#define Warn_LED_Off()  HAL_GPIO_WritePin(WARN_LED_GPIO_OUT_GPIO_Port, WARN_LED_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define Stat1_LED_On()  HAL_GPIO_WritePin(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin, GPIO_PIN_SET)
#define Stat1_LED_Off() HAL_GPIO_WritePin(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define Stat2_LED_On()  HAL_GPIO_WritePin(STAT2_LED_GPIO_OUT_GPIO_Port, STAT2_LED_GPIO_OUT_Pin, GPIO_PIN_SET)
#define Stat2_LED_Off() HAL_GPIO_WritePin(STAT2_LED_GPIO_OUT_GPIO_Port, STAT2_LED_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define Err_LED_On()    HAL_GPIO_WritePin(ERR_LED_GPIO_OUT_GPIO_Port, ERR_LED_GPIO_OUT_Pin, GPIO_PIN_SET)
#define Err_LED_Off()   HAL_GPIO_WritePin(ERR_LED_GPIO_OUT_GPIO_Port, ERR_LED_GPIO_OUT_Pin, GPIO_PIN_RESET)
/* NCS of L9963 H an L */
#define L9963TH_NCS_High() HAL_GPIO_WritePin(L9963TH_NCS_GPIO_OUT_GPIO_Port, L9963TH_NCS_GPIO_OUT_Pin, GPIO_PIN_SET)
#define L9963TH_NCS_Low()  HAL_GPIO_WritePin(L9963TH_NCS_GPIO_OUT_GPIO_Port, L9963TH_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define L9963TL_NCS_High() HAL_GPIO_WritePin(L9963TL_NCS_GPIO_OUT_GPIO_Port, L9963TL_NCS_GPIO_OUT_Pin, GPIO_PIN_SET)
#define L9963TL_NCS_Low()  HAL_GPIO_WritePin(L9963TL_NCS_GPIO_OUT_GPIO_Port, L9963TL_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET)
/* TXEN of L9963 H an L */
#define L9963TH_TXEN_En()  HAL_GPIO_WritePin(L9963TH_TXEN_GPIO_OUT_GPIO_Port, L9963TH_TXEN_GPIO_OUT_Pin, GPIO_PIN_SET)
#define L9963TH_TXEN_Dis() HAL_GPIO_WritePin(L9963TH_TXEN_GPIO_OUT_GPIO_Port, L9963TH_TXEN_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define L9963TL_TXEN_En()  HAL_GPIO_WritePin(L9963TL_TXEN_GPIO_OUT_GPIO_Port, L9963TL_TXEN_GPIO_OUT_Pin, GPIO_PIN_SET)
#define L9963TL_TXEN_Dis() HAL_GPIO_WritePin(L9963TL_TXEN_GPIO_OUT_GPIO_Port, L9963TL_TXEN_GPIO_OUT_Pin, GPIO_PIN_RESET)
/* BNE of L9963 H and L (Bus Not Empty Pin)*/
#define L9963TH_BNE() ((uint8_t)HAL_GPIO_ReadPin(L9963TH_BNE_GPIO_IN_GPIO_Port, L9963TH_BNE_GPIO_IN_Pin))
#define L9963TL_BNE() ((uint8_t)HAL_GPIO_ReadPin(L9963TL_BNE_GPIO_IN_GPIO_Port, L9963TL_BNE_GPIO_IN_Pin))
/* DIS pin of L9963T H and L*/
#define L9963TH_DIS_SET() HAL_GPIO_WritePin(L9963TH_DIS_GPIO_INOUT_GPIO_Port, L9963TH_DIS_GPIO_INOUT_Pin, GPIO_PIN_SET)
#define L9963TH_DIS_RESET() \
    HAL_GPIO_WritePin(L9963TH_DIS_GPIO_INOUT_GPIO_Port, L9963TH_DIS_GPIO_INOUT_Pin, GPIO_PIN_RESET)
#define L9963TL_DIS_SET() HAL_GPIO_WritePin(L9963TL_DIS_GPIO_INOUT_GPIO_Port, L9963TL_DIS_GPIO_INOUT_Pin, GPIO_PIN_SET)
#define L9963TL_DIS_RESET() \
    HAL_GPIO_WritePin(L9963TL_DIS_GPIO_INOUT_GPIO_Port, L9963TL_DIS_GPIO_INOUT_Pin, GPIO_PIN_RESET)
/* Shutdown Circuit Activation (SDC)*/
#define SDC_On() HAL_GPIO_WritePin(SDC_GENERIC_SWITCH_GPIO_OUT_GPIO_Port, SDC_GENERIC_SWITCH_GPIO_OUT_Pin, GPIO_PIN_SET)

#define SDC_Off() \
    HAL_GPIO_WritePin(SDC_GENERIC_SWITCH_GPIO_OUT_GPIO_Port, SDC_GENERIC_SWITCH_GPIO_OUT_Pin, GPIO_PIN_RESET)
/* AIRs signals */
#define AIRs_Pos_Int_Closed() \
    ((uint8_t)HAL_GPIO_ReadPin(AIR_POS_INT_STATE_CLOSED_3V3_GPIO_Port, AIR_POS_INT_STATE_CLOSED_3V3_Pin))
#define AIRs_Neg_Int_Closed() \
    ((uint8_t)HAL_GPIO_ReadPin(AIR_NEG_INT_STATE_CLOSED_3V3_GPIO_Port, AIR_NEG_INT_STATE_CLOSED_3V3_Pin))
#define AIRs_Pos_Mech_Open() \
    ((uint8_t)HAL_GPIO_ReadPin(AIR_POS_MECH_STATE_OPEN_3V3_GPIO_Port, AIR_POS_MECH_STATE_OPEN_3V3_Pin))
#define AIRs_Neg_Mech_Open() \
    ((uint8_t)HAL_GPIO_ReadPin(AIR_NEG_MECH_STATE_OPEN_3V3_GPIO_Port, AIR_NEG_MECH_STATE_OPEN_3V3_Pin))
/* Fuse Enable STEF*/
#define STEF01FTR_En() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_STEF01FTR_Enable_GPIO_Port, uC_GPIO_OUT_STEF01FTR_Enable_Pin, GPIO_PIN_SET)
#define STEF01FTR_Dis() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_STEF01FTR_Enable_GPIO_Port, uC_GPIO_OUT_STEF01FTR_Enable_Pin, GPIO_PIN_RESET)
/* DC BUS signals*/
#define DCBUS_Over60V()     ((uint8_t)HAL_GPIO_ReadPin(nDCBUS_OVER_60V_3V3_GPIO_Port, nDCBUS_OVER_60V_3V3_Pin))
#define STG_DCBUS_Over60V() ((uint8_t)HAL_GPIO_ReadPin(nSTG_DCBUS_OVER60_3V3_GPIO_Port, nSTG_DCBUS_OVER60_3V3_Pin))
#define PRCH_closed()       ((uint8_t)HAL_GPIO_ReadPin(DCBUS_PRCH_RLY_INT_STATE_CLOSED_3V3_GPIO_Port, DCBUS_PRCH_RLY_INT_STATE_CLOSED_3V3_Pin))
#define DCBUS_Rly_Implausibility() ((uint8_t)HAL_GPIO_ReadPin(FB_DCBUS_RLY_IMPLAUSIBILITY_3V3_GPIO_Port, FB_DCBUS_RLY_IMPLAUSIBILITY_3V3_Pin))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
