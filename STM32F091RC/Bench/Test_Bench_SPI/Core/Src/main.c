#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <stdint.h>

#define SPI_CS_PORT		GPIOA
#define SPI_CS_PIN		4

#define SPI_MOSI_PORT	GPIOA
#define SPI_MOSI_PIN	7

#define SPI_SCK_PORT	GPIOA
#define SPI_SCK_PIN		5

#define SPI_MISO_PORT	GPIOA
#define SPI_MISO_PIN	6

#define SPI_CS_HIGH()   (SPI_CS_PORT->BSRR = (1U << (SPI_CS_PIN)))
#define SPI_CS_LOW()    (SPI_CS_PORT->BSRR = (1U << (SPI_CS_PIN + 16)))

void InitSPI(void){
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER &= ~(3U << (4*2));
	GPIOA->MODER |=  (1U << (4*2));   // Output

	GPIOA->MODER &= ~((3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)));
	GPIOA->MODER |=  ((2U << (5*2)) | (2U << (6*2)) | (2U << (7*2)));
	GPIOA->AFR[0] &= ~((0xF << (5*4)) | (0xF << (6*4)) | (0xF << (7*4)));
	GPIOA->AFR[0] |=  ((0x0 << (5*4)) | (0x0 << (6*4)) | (0x0 << (7*4))); // AF0 = SPI1

	SPI1->CR1 &= ~(3u << 3);
	SPI1->CR1 |= (1u << 9)  // SSM Software slave management enabled
			   | (1u << 8)  // SSI interal Slave Select
			   | (7u << 3)  // Baud rate control: fPCLK/4
			   | (1u << 2);  // Master Selection


	SPI1->CR2 |= (7u << 8); // DATA Size PG 817
	// TODO 3: Enable SPI peripheral
	SPI1->CR1 |= (1u << 6);  // SPI enabled
}


uint8_t computeCRC6(uint64_t frame_no_crc)
{
    uint8_t crc = 0x38; // seed = 0b111000
    uint64_t data = frame_no_crc;

    // Shift through bits 39..6 (we process 34 bits)
    for (int i = 39; i >= 6; i--) {
        uint8_t bit = (data >> i) & 0x1;
        uint8_t crc_msb = (crc >> 5) & 0x1;
        crc <<= 1;
        if (bit ^ crc_msb)
            crc ^= 0x19;  // polynomial x^6 + x^4 + x^3 + 1 -> 0b011001
        crc &= 0x3F; // keep 6 bits
    }
    return crc & 0x3F;
}

void L9963EGenFrame(uint8_t rw, uint8_t devID, uint16_t addr, uint8_t gsw, uint32_t data){
	uint64_t frame = 0;
	frame |= ((uint64_t)1      << 39); // P.A.
	frame |= ((uint64_t)rw     << 38); // R/W
	frame |= ((uint64_t)devID  << 35); // 3-bit device ID
	frame |= ((uint64_t)addr   << 25); // 10-bit address
	frame |= ((uint64_t)gsw    << 24); // 1-bit GSW
	frame |= ((uint64_t)data   << 6);  // 18-bit data

	uint8_t crc6 = computeCRC6(frame);
	frame |= crc6; // place CRC in bits [5:0]
}

static void spi_send8(uint8_t data) {
	while (!(SPI1->SR & SPI_SR_TXE));
	*((__IO uint8_t*)&SPI1->DR) = data;
	while (SPI1->SR & SPI_SR_BSY);

}

void l9963ESend40(uint64_t data){

	SPI_CS_LOW();

	for(uint8_t i = 0; i < 4; i++){
		spi_send8(0x00);
	}

	SPI_CS_HIGH();

}

static void L9963EWakeUp(void){
	SPI_CS_LOW();
	for(uint8_t i = 0; i < 5; i++){
		spi_send8(0x52);
	}
	SPI_CS_HIGH();

}


int main(){
	InitSPI();


	while(1){
		L9963EWakeUp();


		uint8_t rw = 1;
		uint8_t devID = 1;
		uint16_t addr = 2;
		uint8_t gsw = 2;
		uint32_t data = 2;
		L9963EGenFrame(rw, devID, addr, gsw, data);


	}



	return 0;
}
