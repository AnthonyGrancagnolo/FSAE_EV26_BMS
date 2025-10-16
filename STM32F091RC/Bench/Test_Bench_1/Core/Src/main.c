// Test Bench SPI communication with Slave Devices

#define debug



#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>


void gpioInit(void);
void pinInit(void);
void setAF(void);
void spiInit(void);

int main(void)
{
    gpioInit();
    pinInit(); 
    setAF();
    spiInit();

    #ifdef debug
        fprintf(stdout, "SPI Initialized\n");
    #endif

    while(1) {

    }
}

void gpioInit(void)
{
    // Enable GPIOA clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

}

void pinInit(void)
{
    // Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) as Alternate Function
    GPIOA->MODER &= ~(3 << (5 * 2) | 3 << (6 * 2) | 3 << (7 * 2));
    GPIOA->MODER |= (2 << (5 * 2) | 2 << (6 * 2) | 2 << (7 * 2));

}

void setAF(void)
{
    // Set Alternate Function 0 for PA5, PA6, PA7
    GPIOA->AFR[0] &= ~(0xF << (5 * 4) | 0xF << (6 * 4) | 0xF << (7 * 4));
    GPIOA->AFR[0] |= (0 << (5 * 4) | 0 << (6 * 4) | 0 << (7 * 4));
}

void spiInit(void)
{
    // Enable SPI1 clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure SPI1 in Master mode, CPOL=0, CPHA=0, 8-bit data frame, Baud rate = fPCLK/16
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_SSM | SPI_CR1_SSI;

    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

