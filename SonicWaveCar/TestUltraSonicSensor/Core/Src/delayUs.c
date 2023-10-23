#include "delayUs.h"

extern TIM_HandleTypeDef htim1;

void delayUs(TIM_HandleTypeDef *htim, uint16_t us)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
	while (__HAL_TIM_GET_COUNTER(htim) < us);
}

