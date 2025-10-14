#include <stdlib.h>
#include <stdio.h>
#include "stm32f091xc.h"
#include "stm32f0xx.h"


void initClocks(){
    RCC->AHB2ENR |= (1 << 4 * 3);

}


int main(){



    return 0;
}