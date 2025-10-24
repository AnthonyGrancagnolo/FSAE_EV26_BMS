#include "main.h"
#include "L9963E.h"
#include "L9963E_DRV.h"
#include "spi.h"
#include "gpio.h"

// Declare interface
L9963E_Interface_t l9963e_if = {
    .Transmit = L9963E_SPI_Transmit,
    .Receive = L9963E_SPI_Receive,
    .CS_Enable = L9963E_CS_Enable,
    .CS_Disable = L9963E_CS_Disable,
    .Delay = L9963E_Delay
};

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    // Initialize the driver
    L9963E_Init(&l9963e_if);

    // Configure for 14 cells
    L9963E_SetCellsNumber(14);

    // Example: read all cell voltages
    uint16_t cellVoltages[14];
    L9963E_ReadAllVoltages(cellVoltages);

    while (1)
    {
        for (int i = 0; i < 14; i++)
        {
            printf("Cell %d: %.3f V\n", i + 1, cellVoltages[i] * 0.0001);
        }

        HAL_Delay(1000);
    }
}
