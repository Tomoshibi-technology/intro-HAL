#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <stdio.h>
#include "tim.h"

void  NeoPixInit (TIM_HandleTypeDef* _htim, uint32_t channel);
void  NeoPixStart (uint8_t* data, int len, bool wait = true);

#endif /* NEOPIXEL_H */
