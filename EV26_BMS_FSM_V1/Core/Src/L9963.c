#include "L9963.h"

void L9963T_init(void){
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_TXEN_GPIO_OUT_GPIO_Port,L9963T_TXEN_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_TXAMP_GPIO_OUT_GPIO_Port,L9963T_TXAMP_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L9963T_ISOFREQ_GPIO_OUT_GPIO_Port,L9963T_ISOFREQ_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L9963T_DIS_GPIO_INOUT_GPIO_Port,L9963T_DIS_GPIO_INOUT_Pin, GPIO_PIN_RESET);
}

void L9963E_wakeup(void){

	uint8_t wakeup_frame[FRAME_SIZE] = {0x55, 0x55, 0x55, 0x55, 0x55};

	HAL_GPIO_WritePin(L9963T_TXEN_GPIO_OUT_GPIO_Port,L9963T_TXEN_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, wakeup_frame,FRAME_SIZE,SPI_TRANSMIT_TIMEOUT );
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_TXEN_GPIO_OUT_GPIO_Port,L9963T_TXEN_GPIO_OUT_Pin, GPIO_PIN_RESET);

}

void l9963TL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout){

	HAL_GPIO_WritePin(L9963T_TXEN_GPIO_OUT_GPIO_Port,L9963T_TXEN_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, pData,Size,Timeout);
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L9963T_TXEN_GPIO_OUT_GPIO_Port,L9963T_TXEN_GPIO_OUT_Pin, GPIO_PIN_RESET);

}
void L9963TL_SPI_WaitAndRecieve(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout){
	while(HAL_GPIO_ReadPin(L9963T_DIS_GPIO_INOUT_GPIO_Port,L9963T_DIS_GPIO_INOUT_Pin) != 0);
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(hspi, pData,Size,Timeout);
	HAL_GPIO_WritePin(L9963T_NCS_GPIO_OUT_GPIO_Port,L9963T_NCS_GPIO_OUT_Pin, GPIO_PIN_SET);
}

void L9963E_init(uint8_t n_slaves){
	uint8_t x = 0;

	L9963E_wakeup();
	HAL_Delay(2);
	L9963E_wakeup();
	uint8_t init_frame[FRAME_SIZE] = {0x83, 0x84, 0x00, 0x00, 0xB7};
	uint8_t RX_frame[FRAME_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00};

	HAL_Delay(10);
	//while(x < n_slaves){
		l9963TL_SPI_Transmit(&hspi3, init_frame,FRAME_SIZE,SPI_TRANSMIT_TIMEOUT);
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
