
#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"


// --- STATIC STORAGE FOR THE OUT-OF-FRAME RESPONSE ---
// The L9963E returns the answer to the N-th command in the (N+1)-th frame (out-of-frame).
// This variable stores the response received from the *previous* SPI transaction.
static uint32_t last_rx_frame = 0;


int main(){
	GPIOInit();
	setAF();
	SPI1Init();

	while(1){
		// TODO Send transmission

	}

	return 0;
}

void GPIOInit(void){
	// Enable GPIOS
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;


	// Enable SPI
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	// Clear mode bits
	SPI1_PORT->MODER &= ~((3 << (SPI1_CS_PIN * 2)) | (3 << (SPI1_SCK_PIN * 2)) | (3 << (SPI1_MISO_PIN * 2)) | (3 << (SPI1_MOSI_PIN * 2)));
	 // Set to Alternate Function mode
	SPI1_PORT->MODER |= (2 << (SPI1_CS_PIN * 2)) | (2 << (SPI1_SCK_PIN * 2)) | (2 << (SPI1_MISO_PIN * 2)) | (2 << (SPI1_MOSI_PIN * 2));
	// Clear AFRL bits and set to AF0
	SPI1_PORT->AFR[0] &= ~((0xF << (SPI1_CS_PIN * 4)) | (0xF << (SPI1_SCK_PIN * 4)) | (0xF << (SPI1_MISO_PIN * 4)) | (0xF << (SPI1_MOSI_PIN * 4)));

}


void SPI1Init(){
	// TODO Set configure CR1 and CR2
	// Configure SPI1
	    SPI1->CR1 |= (0u << 15) // MIDIMODE 0 full duplex 1 half duplex
	               | (0u << 10) // RX Only 0 FULL DUPLEX 1 RX ONLY
	               | (1u << 9)  // SSM Software slave management enabled
	               | (1u << 8)  // SSI interal Slave Select
	               | (1u << 7)  // LSB FIRST 0 MSB 1 LSB
	               | (7u << 3)  // Baud rate control: fPCLK/X
	               | (1u << 2)  // Master Selection
	               | (1u << 1) 	// Clock Polarity
	               | (1u << 0); // Clock Phase

	    SPI1->CR2 |= (0u << 14) // LDMA_TX 0 number of data to transfer is even 1 odd
	               | (0u << 13) // LEMA)RX 0 number of data to transfer is even 1 odd
	               | (0u << 12) // FRXTH 0 RXNE event is generated if the FIFO level is greater than or equal to 1/2 1 RXNE event is generated if the FIFO level is greater than or equal to 1/4
	               | (7u << 8) // DATA Size PG 817
	               | (0u << 7) // TXEIE 0 TXE interupt Masked 1 TXE interrupt not masked
	               | (0u << 6) // RXNEIE 0 RXNE interrupt Masked 1 RXNE interrupt not masked
	               | (0u << 5) // ERRIE 0 Error interrupt Masked 1 Error interrupt is enabled
	               | (0u << 4) // FRF 0 Motorola 1 TI
	               | (0u << 3) // NSSP 0 no NSS Pulse 1 NSS pulse generated
	               | (0u << 2) // SSOE 0  SS output disabled 1 SS output enabled
	               | (0u << 1) // TXDMAEN 0 TX DMA disabled 1 TX DMA enabled
	               | (0u << 0); // RXDMAEN 0 RX DMA disabled 1 RX DMA enabled
	// Enable SPI peripheral
	SPI1->CR1 |= (1u << 6);  // SPI enabled

}

void L9963ETXX(uint8_t TX){
	// TODO Figure out the the Data size and change the second X in the function name and the uint size

	// wait till buffer is empty
	while (!(SPI1->SR & SPI_SR_TXE));
	// TODO TX

	// wait for transmission to complete
	while (SPI1->SR & SPI_SR_BSY);
}

void L9963ERXX(uint8_t RX){
	// TODO Figure out the the Data size and change the second X in the function name and the uint size

	// wait till buffer is empty
	while (!(SPI1->SR & SPI_SR_RXE));
	// TODO RX

	// wait for transmission to complete
	while (SPI1->SR & SPI_SR_BSY);



}
