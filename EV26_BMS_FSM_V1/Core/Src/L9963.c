#include "L9963.h"
#include "main.h"
#include "spi.h"


void L9963T_Init(void){
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_TXEN_GPIO_OUT_GPIO_Port,L9963T_TXEN_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_TXAMP_GPIO_OUT_GPIO_Port,L9963T_TXAMP_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L9963T_ISOFREQ_GPIO_OUT_GPIO_Port,L9963T_ISOFREQ_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L9963T_DIS_GPIO_INOUT_GPIO_Port,L9963T_DIS_GPIO_INOUT_Pin, GPIO_PIN_RESET);
}
void l9963TL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout){

	HAL_GPIO_WritePin(L9963T_TXEN_GPIO_OUT_GPIO_Port,L9963T_TXEN_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, pData,Size,Timeout);
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_TXEN_GPIO_OUT_GPIO_Port,L9963T_TXEN_GPIO_OUT_Pin, GPIO_PIN_RESET);

}

void L9963TL_SPI_WaitAndRecieve(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout){
	while(HAL_GPIO_ReadPin(L9963T_DIS_GPIO_INOUT_GPIO_Port,L9963T_DIS_GPIO_INOUT_Pin) != 0);
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi2, pData,Size,Timeout);
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_SET);
}
void L9963E_Wakeup(void){

	uint8_t wakeup_frame[FRAME_SIZE] = {0x55, 0x55, 0x55, 0x55, 0x55};

	l9963TL_SPI_Transmit(&hspi2, wakeup_frame,FRAME_SIZE,SPI_TRANSMIT_TIMEOUT);

}


void L9963E_Init(uint8_t n_slaves){
	uint8_t x = 0;

	L9963E_Wakeup();
	HAL_Delay(2);
	L9963E_Wakeup();
	uint8_t init_frame[FRAME_SIZE] = {0x83, 0x84, 0x00, 0x00, 0xB7};
	uint8_t RX_frame[FRAME_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00};

	HAL_Delay(10);
	//while(x < n_slaves){
		l9963TL_SPI_Transmit(&hspi2, init_frame,FRAME_SIZE,SPI_TRANSMIT_TIMEOUT);
		//L9963TL_SPI_WaitAndRecieve(&hspi3, RX_frame,FRAME_SIZE,SPI_TRANSMIT_TIMEOUT);
		/*if(1){
			//read back

			// read back sucessfull increment x
			x++;
		}else{

		}
			//send cmd to dev gen reg with
		*/
	//}

}
