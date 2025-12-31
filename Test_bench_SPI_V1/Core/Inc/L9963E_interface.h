/**
 * @file L9963E_interface.h
 * @brief Hardware abstraction layer interface for L9963E library on STM32F091RC
 * @details Implements platform-specific functions required by L9963E_lib
 *          Configured for SOFTWARE NSS control on PA4
 */

#ifndef L9963E_INTERFACE_H
#define L9963E_INTERFACE_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* Configuration for your hardware - STM32F091RCTx LQFP64 */
#define L9963E_SPI_HANDLE       hspi1           // SPI1: PA5=SCK, PA6=MISO, PA7=MOSI
#define L9963E_CS_PORT          GPIOA           // CS on PA4 (software controlled)
#define L9963E_CS_PIN           GPIO_PIN_4      // PA4
#define L9963E_SPI_TIMEOUT      100             // SPI timeout in ms

/* External handles (defined in main.c) */
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;  // For debug output on PA2

/* Interface function prototypes required by L9963E_lib */

/**
 * @brief Initialize the hardware interface
 * @return 0 on success, -1 on failure
 */
int L9963E_IF_Init(void);

/**
 * @brief Set CS pin low (select device)
 * @note PA4 will be driven LOW to select L9963E
 */
void L9963E_IF_CS_Low(void);

/**
 * @brief Set CS pin high (deselect device)
 * @note PA4 will be driven HIGH to deselect L9963E
 */
void L9963E_IF_CS_High(void);

/**
 * @brief Transmit data via SPI
 * @param pData Pointer to data buffer
 * @param Size Number of bytes to transmit
 * @return 0 on success, -1 on failure
 */
int L9963E_IF_SPI_Transmit(uint8_t *pData, uint16_t Size);

/**
 * @brief Receive data via SPI
 * @param pData Pointer to receive buffer
 * @param Size Number of bytes to receive
 * @return 0 on success, -1 on failure
 */
int L9963E_IF_SPI_Receive(uint8_t *pData, uint16_t Size);

/**
 * @brief Transmit and receive data via SPI simultaneously
 * @param pTxData Pointer to transmit buffer
 * @param pRxData Pointer to receive buffer
 * @param Size Number of bytes to transfer
 * @return 0 on success, -1 on failure
 */
int L9963E_IF_SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

/**
 * @brief Delay in microseconds
 * @param us Microseconds to delay
 * @note Configured for 48 MHz system clock
 */
void L9963E_IF_Delay_us(uint32_t us);

/**
 * @brief Delay in milliseconds
 * @param ms Milliseconds to delay
 */
void L9963E_IF_Delay_ms(uint32_t ms);

/**
 * @brief Debug print via UART2 (optional helper function)
 * @param str String to print
 */
void L9963E_IF_Debug_Print(const char *str);

#endif /* L9963E_INTERFACE_H */
