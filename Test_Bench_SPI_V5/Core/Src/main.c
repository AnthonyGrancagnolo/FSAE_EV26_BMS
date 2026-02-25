/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "L9963E_utils.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define L9963E_DEBUG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void L9963T_init(void){

	HAL_GPIO_WritePin(L9963T_CLKFREQ_GPIO_OUT_GPIO_Port,L0063T_CLKFREQ_GPIO_OUT_Pin,GPIO_Pin_Set);
}

// Transmit and recive frame, data ready to be transmitted
	  uint8_t Wake_Up[5] = {
			  0x55,  // Byte 0: P.A.=0, R/W=0, Dev ID=0000 (broadcast)
		      0x55,  // Byte 1: Address = 0x00
			  0x55,  // Byte 2: GSW + Data (upper bits)
			  0x55,  // Byte 3: Data (middle bits)
			  0x55,   // Byte 4: Data (lower bits) + CRC
	  };


	uint8_t TX_Buffer[5] = {
			0x00,
	   	    0x00,
		    0x08,
			0x04,
			0xC2};

 uint8_t RX_Request[5] = {
		   0x00,
			  0x00,
			  0x00,
			  0x04,
		      0x82};

	 uint8_t RX_Buffer[5] = {
	  	  0x00,  // Byte 0: P.A.=0, R/W=0, Dev ID=0000 (broadcast)
	  	  0x00,  // Byte 1: Address = 0x00
   	      0x00,  // Byte 2: GSW + Data (upper bits)
  	  	  0x00,  // Byte 3: Data (middle bits)
	  	  0x00   // Byte 4: Data (lower bits) + CRC
	  	  	  };
	 uint8_t TX_Dummy[5] = {
	 	  	  0x00,  // Byte 0: P.A.=0, R/W=0, Dev ID=0000 (broadcast)
	 	  	  0x00,  // Byte 1: Address = 0x00
	    	  0x00,  // Byte 2: GSW + Data (upper bits)
	   	  	  0x00,  // Byte 3: Data (middle bits)
	 	  	  0x00   // Byte 4: Data (lower bits) + CRC
	 	  	  	  };


void Daniel_init(void){




    HAL_GPIO_WritePin(NSLAVE_GPIO_Port, NSLAVE_Pin, GPIO_PIN_RESET); // 1 = on
    HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_SET); // 1 = on
    HAL_GPIO_WritePin(TXAMP_GPIO_Port, TXAMP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ISOFREQ_GPIO_Port, ISOFREQ_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // Start main thing


    uint8_t TX_Hold1[5] = {};
    uint8_t TX_Hold2[5] = {};
    uint8_t TX_Hold3[5] = {};

          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
          HAL_SPI_Transmit(&hspi3, Wake_Up,5,32); //Sending in DMA mode
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    	  HAL_Delay(2);

/*
			HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET); // 1 = on
			//while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1){}
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_Receive(&hspi3, RX_Buffer, 5, 32);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_SET); // 1 = on

*/

          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
          L9963E_Add_CRC_To_Word(TX_Buffer);
          switch_endianness(TX_Buffer, TX_Hold1,1);
          HAL_SPI_Transmit(&hspi3, TX_Hold1,5,32); //Sending in DMA mode
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


/*

          	  	  	HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET); // 1 = on
                    //while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1){}
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                    HAL_SPI_Receive(&hspi3, RX_Buffer, 5, 32);
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_SET); // 1 = on
*/


          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
          L9963E_Add_CRC_To_Word(RX_Request);
          switch_endianness(RX_Request, TX_Hold2,1);
          HAL_SPI_Transmit(&hspi3, TX_Hold2,5,32); //Sending in DMA mode
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

/*

          HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET); // 1 = on
          //while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1){}
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
          HAL_SPI_Receive(&hspi3, RX_Buffer, 5, 32);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
          HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_SET); // 1 = on

*/

                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                   L9963E_Add_CRC_To_Word(TX_Dummy);
                   switch_endianness(TX_Dummy, TX_Hold3,0);
                   HAL_SPI_Transmit(&hspi3, TX_Hold3,5,32); //Sending in DMA mode
                   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
               //    HAL_Delay(2);

         HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET); // 1 = on
                            //while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1){}
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                  HAL_SPI_Receive(&hspi3, RX_Buffer, 5, 32);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_SET); // 1 = on

                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                                     L9963E_Add_CRC_To_Word(TX_Dummy);
                                     switch_endianness(TX_Dummy, TX_Hold3,0);
                                     HAL_SPI_Transmit(&hspi3, TX_Hold3,5,32); //Sending in DMA mode
                                     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                                  //   HAL_Delay(2);

                           HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET); // 1 = on
                                              //while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1){}
                                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                                    HAL_SPI_Receive(&hspi3, RX_Buffer, 5, 32);
                                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                                    HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_SET); // 1 = on

                                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                                                       L9963E_Add_CRC_To_Word(TX_Dummy);
                                                       switch_endianness(TX_Dummy, TX_Hold3,0);
                                                       HAL_SPI_Transmit(&hspi3, TX_Hold3,5,32); //Sending in DMA mode
                                                       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                                                   //    HAL_Delay(2);

                                             HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_RESET); // 1 = on
                                                                //while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1){}
                                                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                                                      HAL_SPI_Receive(&hspi3, RX_Buffer, 5, 32);
                                                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                                                      HAL_GPIO_WritePin(TXEN_GPIO_Port, TXEN_Pin, GPIO_PIN_SET); // 1 = on

         // HAL_SPI_Transmit_DMA(&hspi3, TX_Buffer, 1); //


}
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
  void L9963E_Add_CRC_To_Word(uint8_t *word);
  uint8_t L9963E_Calculate_CRC(uint8_t *data, uint8_t length);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  while(HAL_GetTick() < 500)
    HAL_Delay(100);

  L9963E_utils_init();
  // init the L9963E for RX only
  //NSLAVE = 0
  // ISOFREQ = 0
  // TXAMP = 0
  //
  Daniel_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Makes a frame of Data from inputs

uint8_t Frame_Builder(uint8_t Data, uint8_t Address, uint8_t RW){

	uint8_t TX_Frame[5] = {};

	 TX_Frame[0] = Data >> 32;
	 TX_Frame[1] = (Address >> 24) & 0x4;
	 TX_Frame[2] = RW >> 8;


	return TX_Frame;
}


void switch_endianness(uint8_t *in, uint8_t *out, uint8_t state) {


if(state == 1){
    out[0] = in[4];
    out[1] = in[3];
    out[2] = in[2];
    out[3] = in[1];
    out[4] = in[0];
}else if(state == 0){
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
    out[3] = in[3];
    out[4] = in[4];
}
    return;
}

// Claude CRC code

/**
 * @brief Calculate 6-bit CRC for L9963E communication
 * @param data Pointer to data bytes (40 bits = 5 bytes)
 * @param length Number of bytes to process (typically 5 for 40-bit word)
 * @return 6-bit CRC value
 */
uint8_t L9963E_Calculate_CRC(uint8_t *data, uint8_t length)
{
    uint8_t crc = L9963E_CRC_INIT;
    uint8_t i, j;

    for (i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80)  // Check MSB
            {
                crc = (crc << 1) ^ L9963E_CRC_POLY;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }

    // Return only 6-bit CRC
    return (crc >> 2) & 0x3F;
}

/**
 * @brief Add CRC to a 40-bit word in the first 6 bits
 * @param word Pointer to 40-bit word (5 bytes array)
 * @note The CRC is placed in bits [39:34] (first 6 bits of first byte)
 *       The function assumes data is in bits [33:0]
 */
void L9963E_Add_CRC_To_Word(uint8_t *word)
{
    uint8_t crc;
    uint8_t temp_data[5];

    // Copy word data, clearing the CRC bits
    temp_data[0] = word[0] & 0x03;  // Clear upper 6 bits (CRC field)
    temp_data[1] = word[1];
    temp_data[2] = word[2];
    temp_data[3] = word[3];
    temp_data[4] = word[4];

    // Calculate CRC on the 34-bit data portion
    crc = L9963E_Calculate_CRC(temp_data, 5);

    // Insert CRC into the upper 6 bits of first byte
    word[0] = (word[0] & 0x03) | (crc << 2);
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
