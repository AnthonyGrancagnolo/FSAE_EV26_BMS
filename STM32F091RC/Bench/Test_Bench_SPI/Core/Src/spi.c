#include "stm32f0xx_hal.h"
#include "L9963E.h"
#include "L9963E_DRV.h"

extern SPI_HandleTypeDef hspi1; // SPI1 from CubeMX

void L9963E_SPI_Transmit(uint8_t *data, uint16_t size)
{
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
}

void L9963E_SPI_Receive(uint8_t *data, uint16_t size)
{
    HAL_SPI_Receive(&hspi1, data, size, HAL_MAX_DELAY);
}

void L9963E_CS_Enable(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // CS low
}

void L9963E_CS_Disable(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    // CS high
}

void L9963E_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}
