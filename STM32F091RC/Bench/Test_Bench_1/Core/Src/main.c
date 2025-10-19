// Test Bench SPI communication with Slave Devices

//#define debug

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
    // Enable GPIO B clock
    RCC->AHBENR |= (1u << 18); // Enable GPIOB clock
    RCC->APB2ENR |= (1u << 12); // Enable SPI1 clock
}

void pinInit(void)
{
    // Set PB3(SCK), PB4(MISO), PB5(MOSI) to Alternate Function mode
    GPIOB->MODER &= ~(3u << (3 * 2) | 3u << (4 * 2) | 3u << (5 * 2)); // Clear mode bits for PB3, PB4, PB5
    GPIOB->MODER |= (2u << (3 * 2) | 2u << (4 * 2) | 2u << (5 * 2));  // Set to Alternate Function mode
    // Set PB6 (NSS) as output
    GPIOB->MODER &= ~(3u << (6 * 2)); // Clear mode bits for PB6
    GPIOB->MODER |= (1u << (6 * 2));  // Set to General Purpose Output mode

}

void setAF(void)
{
    // Set Alternate Function 0 (AF0) for PB3(SCK), PB4(MISO), PB5(MOSI)    
    //redo 
    GPIOB->AFR[0] &= ~(15u << (3 * 4) | 15u << (4 * 4) | 15u << (5 * 4)); // Clear AFRL bits for PB3, PB4, PB5
    GPIOB->AFR[0] |= (0u << (3 * 4) | 0u << (4 * 4) | 0u << (5 * 4));  // Set AF0 for SPI1

}

void spiInit(void)
{
    // Configure SPI1
    SPI1->CR1 |= (0u << 15) // MIDIMODE 0 full duplex 1 half duplex
               | (0u << 14) // Bidirectional mode disabled 0 recive only 1 transmit only
               | (0u << 13) // CRC Calcualtion disabled 0 disabled 1 enabled
               | (0u << 12) // CRCNEXT 0 next transmit vlaue is from the Tx buffer 1 next transmit value is from the CRC register
               | (0u << 11) // CRC Length 0 8-bit 1 16-bit
               | (0u << 10) // RX Only 0 FULL DUPLEX 1 RX ONLY
               | (1u << 9)  // SSM Software slave management enabled
               | (1u << 8)  // SSI interal Slave Select
               | (0u << 7)  // LSB FIRST 0 MSB 1 LSB
               | (1u << 6)  // SPI enabled
               | (4u << 3)  // Baud rate control: fPCLK/X
               | (1u << 2)  // Master Selection
               | (1u << 1); // Clock Polarity
               | (1u << 0); // Clock Phase
    
    SPI1->CR2 |= (0u << 14) // LDMA_TX 0 number of data to transfer is even 1 odd
               | (0u << 13) // LEMA)RX 0 number of data to transfer is even 1 odd
               | (0u << 12) // FRXTH 0 RXNE event is generated if the FIFO level is greater than or equal to 1/2 1 RXNE event is generated if the FIFO level is greater than or equal to 1/4
               | (0u << 8) // DATA Size PG 817
               | (0u << 7) // TXEIE 0 TXE interupt Masked 1 TXE interrupt not masked
               | (0u << 6) // RXNEIE 0 RXNE interrupt Masked 1 RXNE interrupt not masked
               | (0u << 5) // ERRIE 0 Error interrupt Masked 1 Error interrupt is enabled
               | (0u << 4) // FRF 0 Motorola 1 TI
               | (0u << 3) // NSSP 0 no NSS Pulse 1 NSS pulse generated
               | (0u << 2) // SSOE 0  SS output disabled 1 SS output enabled
               | (0u << 1) // TXDMAEN 0 TX DMA disabled 1 TX DMA enabled
               | (0u << 0); // RXDMAEN 0 RX DMA disabled 1 RX DMA enabled
   
}

