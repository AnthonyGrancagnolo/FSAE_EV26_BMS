#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"


int main(){
	GPIOInit();
	setAF();
	SPIInit();

	while(1){
		switch(vehicleState){
		case 0: // Fault

			break;
		case 1: // Ready

			break;
		case 2: // Charging

			break;
		case 3: // Driving

			break;

		}
	}

	return 0;
}

//test
