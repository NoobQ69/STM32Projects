#ifndef __HCSR04_H
#define __HCSR04_H

#include "main.h"

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HCSR04_Read (void);

#endif

