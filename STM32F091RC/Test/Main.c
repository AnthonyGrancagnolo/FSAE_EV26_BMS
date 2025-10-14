#include <stdlib.h>
#include <stdio.h>
#include "stm32f091xc.h"
#include "stm32f0xx.h"


void initClocks(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    SYSCFG->EXTICR 
     
}

void setPinMode(void){


}


int main(){



    return 0;
}