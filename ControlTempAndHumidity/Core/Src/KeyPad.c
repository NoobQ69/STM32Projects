#include "KeyPad.h"

void readColumn(uint8_t *key, uint8_t row)
{
	if ((GPIOA->IDR & GPIO_PIN_3) == 0)
	{
		*key = row;
		while((GPIOA->IDR & GPIO_PIN_3) == 0);
	}
	else if ((GPIOA->IDR & GPIO_PIN_2) == 0)
	{
		*key = row + 1;
		while((GPIOA->IDR & GPIO_PIN_2) == 0);
	}
	else if ((GPIOA->IDR & GPIO_PIN_1) == 0)
	{
		*key = row + 2;
		while((GPIOA->IDR & GPIO_PIN_1) == 0);
	}
}

uint8_t readKey(void)
{
	uint8_t key = 10;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	readColumn(&key, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	readColumn(&key, 4);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	readColumn(&key, 7);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
	return key;
}
