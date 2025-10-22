#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"


int main(){
	GPIOInit();
	setAF();
	SPI1Init();

	while(1){
		// TODO Send transmission


	}

	return 0;
}

//test
