/*
 * DHAB124.h
 *
 *  Created on: Dec 12, 2025
 *      Author: antgr
 */

#ifndef DHAB124_H_
#define DHAB124_H_

#include <stdint.h>

float getLowCurrent(const uint16_t *buffer);
float getHighCurrent(const uint16_t *buffer);

#endif /* DHAB124_H_ */
