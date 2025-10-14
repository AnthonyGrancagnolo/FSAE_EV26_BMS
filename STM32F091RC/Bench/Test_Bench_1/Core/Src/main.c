// Test Bench SPI communication with Slave Devices

#include "stm32f0xx.h"
#include <stdint.h>

void gpioInit(void);
void pinInit(void);
void spiInit(void);

int main(void)
{
    gpioInit();
    pinInit();
    spiInit();

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
    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

    // Set Alternate Function to AF0 (SPI1)
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
    GPIOA->AFR[0] |= (0x0 << (5 * 4)) | (0x0 << (6 * 4)) | (0x0 << (7 * 4));

    // Configure PA5, PA6, PA7 as High Speed
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR5 | GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7);

    // Configure PA5, PA6, PA7 with Pull-Up resistors
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0);
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