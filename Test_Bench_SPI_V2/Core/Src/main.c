#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include "string.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

/* Pin macros */
#define CS_LOW()       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_HIGH()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define DIS_LOW()      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)
#define TXEN_HIGH()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
#define READ_BNE()     HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)

/* UART redirect for printf */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void delay_short(void) {
    for (volatile int i=0; i<500; i++); // ~ short software delay
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();

    uint8_t txData = 0xAA;
    uint8_t rxData;

    /* Keep working wake/enable behavior */
    DIS_LOW();       // Enable ISO-SPI
    TXEN_HIGH();     // Enable transmit

    char msg[50];

    while (1)
    {
        // CS pulse: slow single byte
        CS_LOW();
        HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, HAL_MAX_DELAY);
        delay_short();
        CS_HIGH();

        // Repeat a few dummy bytes to ensure wake
        for (int i = 0; i < 5; i++) {
            CS_LOW();
            HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, HAL_MAX_DELAY);
            CS_HIGH();
            delay_short();
        }

        // Read BNE to see if board responded
        uint8_t bne = READ_BNE();
        snprintf(msg, sizeof(msg), "SPI RX=0x%02X  BNE=%d\r\n", rxData, bne);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        HAL_Delay(500);
    }
}

/* ================= System Clock & Error Handler ================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

//changes


void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
