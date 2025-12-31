/**
 * @file L9963E_interface.c
 * @brief Hardware abstraction layer implementation for L9963E on STM32F091RC
 * @note Configured for 48 MHz system clock with SOFTWARE NSS control on PA4
 */

#include "L9963E_interface.h"
#include <string.h>
#include <stdio.h>

/* External handles from main.c */
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

/**
 * @brief Initialize the hardware interface
 */
int L9963E_IF_Init(void)
{
    /* Enable DWT for microsecond delays */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    /* Set CS high (deselected) initially */
    L9963E_IF_CS_High();

    /* Small delay to ensure CS is stable */
    L9963E_IF_Delay_ms(10);

    return 0;
}

/**
 * @brief Set CS pin low (select device)
 * @note Software control of PA4 - active LOW
 */
void L9963E_IF_CS_Low(void)
{
    HAL_GPIO_WritePin(L9963E_CS_PORT, L9963E_CS_PIN, GPIO_PIN_RESET);
    /* Small delay for CS setup time (adjust if needed based on L9963E timing) */
    L9963E_IF_Delay_us(1);
}

/**
 * @brief Set CS pin high (deselect device)
 * @note Software control of PA4 - inactive HIGH
 */
void L9963E_IF_CS_High(void)
{
    /* Small delay before releasing CS (adjust if needed) */
    L9963E_IF_Delay_us(1);
    HAL_GPIO_WritePin(L9963E_CS_PORT, L9963E_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Transmit data via SPI
 */
int L9963E_IF_SPI_Transmit(uint8_t *pData, uint16_t Size)
{
    if (HAL_SPI_Transmit(&hspi1, pData, Size, L9963E_SPI_TIMEOUT) != HAL_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief Receive data via SPI
 */
int L9963E_IF_SPI_Receive(uint8_t *pData, uint16_t Size)
{
    if (HAL_SPI_Receive(&hspi1, pData, Size, L9963E_SPI_TIMEOUT) != HAL_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief Transmit and receive data via SPI simultaneously
 */
int L9963E_IF_SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    if (HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, Size, L9963E_SPI_TIMEOUT) != HAL_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief Delay in microseconds using DWT cycle counter
 * @note System clock is 48 MHz, so 48 cycles = 1 microsecond
 */
void L9963E_IF_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = 48 * us;  // 48 MHz system clock

    while ((DWT->CYCCNT - start) < cycles) {
        /* Wait */
    }
}

/**
 * @brief Delay in milliseconds using HAL
 */
void L9963E_IF_Delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief Debug print via UART2 (optional helper function)
 */
void L9963E_IF_Debug_Print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}
