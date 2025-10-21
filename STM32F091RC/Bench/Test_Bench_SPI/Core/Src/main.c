#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"


int main(){
	GPIOInit();
	setAF();
	SPIInit();

	return 0;
}

