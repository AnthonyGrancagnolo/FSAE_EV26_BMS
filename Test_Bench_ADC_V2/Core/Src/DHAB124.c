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
#define SENSITIVITYHIGH   0.004f   // V/A

float getLowCurrent(const uint16_t *buffer)
{
    uint32_t adc_avg = (uint32_t) buffer[0];
    // ADC counts → ADC pin voltage
    float V_adc = (adc_avg * VREF_ADC) / ADC_COUNTS;

    // Undo resistor divider
    float V_sensor = V_adc * DIVIDER_RATIO;

    // Sensor voltage → current
    return (1/SENSITIVITYLOW)*(V_sensor-OFFSET); //(V_sensor - OFFSET) / SENSITIVITY;

}
float getHighCurrent(const uint16_t *buffer)
{
    uint32_t adc_avg = (uint32_t) buffer[1];
    // ADC counts → ADC pin voltage
    float V_adc = (adc_avg * VREF_ADC) / ADC_COUNTS;

    // Undo resistor divider
    float V_sensor = V_adc * DIVIDER_RATIO;

    // Sensor voltage → current
    return (1/SENSITIVITYHIGH)*(V_sensor-OFFSET); //(V_sensor - OFFSET) / SENSITIVITY;

}

