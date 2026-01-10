 /*
 * DHAB124.c
 *
 *  Created on: Dec 12, 2025
 *      Author: antgr
 */
#include "DHAB124.h"

/* ADC parameters */
#define VREF_ADC       3.3f
#define ADC_COUNTS     4095.0f

/* Resistor divider */
#define DIVIDER_RATIO  1.575f   //  R1 + R2 /R2

/* DHAB S/124 Channel 1 */
#define OFFSET         2.5f      // volts Tues
#define SENSITIVITYLOW    0.0267f   // V/A
#define SENSITIVITYHIGH    0.0267f   // V/A
#define MAXCURRENTLOW 75

float getCurrent(const uint16_t *buffer)
{
    uint32_t adc_avg = buffer[0];

    // ADC counts → ADC pin voltage
    float V_adc = (adc_avg * VREF_ADC) / ADC_COUNTS;

    // Undo resistor divider
    float V_sensor = (1/SENSITIVITYLOW)*((V_adc * DIVIDER_RATIO)-OFFSET);


    if(MAXCURRENTLOW < V_sensor) (1/SENSITIVITYHIGH)*((V_adc * DIVIDER_RATIO)-OFFSET);

    // Sensor voltage → current

    return V_sensor; //(V_sensor - OFFSET) / SENSITIVITY;



}

