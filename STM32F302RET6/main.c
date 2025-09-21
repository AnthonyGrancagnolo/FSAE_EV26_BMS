#include <stdlib.h>
#include <stdio.h>
#include "stm32f302xe.h"


void intiSPI(void);
void initClock(void);
void configSPIpins(void);
void setPinMode(void);
void setAF(void);
void configSPI(void);
uint16_t transferSPI(uint16_t txData);

void intiSPI(){
    initClock(); // enable clock for SPI and GPIO
    configSPIpins(); // configure GPIO pins for SPI
    configSPI(); // configure SPI parameters
    setPinMode(); // set pin mode to AF
    setAF(); // set alternate function for pins
    
}


int main(void)
{   

    


    return 0;
}