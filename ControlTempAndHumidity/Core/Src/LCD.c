#include "LCD.h"
#include "delay_timer.h"


extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_print_integer_number(int number)
{
	lcd_send_data (number%100/10 + 0x30);
	lcd_send_data (number%10 + 0x30);
}

void lcd_print_float_number(float number)
{
	int intergerPart = (int)number;
	int fractionalPart = (number - (int)number)*10;
	
	lcd_send_data (intergerPart%100/10 + 0x30);
	lcd_send_data (intergerPart%10 + 0x30);
	lcd_send_data('.');
	lcd_send_data (fractionalPart%10 + 0x30);
}

void lcd_gotoxy(uint8_t row, uint8_t column)
{
    char position;

    if (row < 2)
    {
        position = 0x80 | (row << 6);

        if (column < 16)
        {
            position = position + column;

            lcd_send_cmd(position);
        }
    }
}

void lcd_reset()
{
	lcd_send_cmd(0x01);
	DELAY_TIM_Ms(&htim2, 20);
}

void lcd_init (void)
{
	// 4 bit initialisation
	DELAY_TIM_Ms(&htim2,50);  // wait for >40ms
	lcd_send_cmd (0x30);
	DELAY_TIM_Ms(&htim2,5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	DELAY_TIM_Ms(&htim2,1);  // wait for >100us
	lcd_send_cmd (0x30);
	DELAY_TIM_Ms(&htim2,10);
	lcd_send_cmd (0x20);  // 4bit mode
	DELAY_TIM_Ms(&htim2,10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	DELAY_TIM_Ms(&htim2,1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	DELAY_TIM_Ms(&htim2,1);
	lcd_send_cmd (0x01);  // clear display
	DELAY_TIM_Ms(&htim2,1);
	DELAY_TIM_Ms(&htim2,1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	DELAY_TIM_Ms(&htim2,1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}


