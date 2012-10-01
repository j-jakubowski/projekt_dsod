#include <avr/io.h>
#include "uart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include <avr/interrupt.h>
#include <util/delay.h>

#define EN_SENS_A PA6
#define EN_SENS_B PA7
#define EN_SENS_C PC7
#define SW1	PC0
#define SW2 PC1

#define READ 0
#define WRITE 1

#define sensor_address

char str_temporary[20];
uint8_t temporary;

uint8_t sensor_count(void);

int main(void)
{

	sei();


	/* lcd config */
	lcd_init(LCD_DISP_ON);

	/* UART config */
	UART_vInit(115200,8,UART_NO_PARITY,UART_NO_ERROR);

	/* I2C config */
	i2c_init();

	/* SPI config */

	/* GPIO config */

	DDRA|= (1<<EN_SENS_A) | (1<<EN_SENS_B); //all enable lines as outputs
	DDRC|= (1<<EN_SENS_C);

	PORTC |= (1<<SW1) | (1<<SW2); // pull-up on switches


	lcd_puts("System pomiarowy");
	lcd_gotoxy(0,1);
	lcd_puts("wersja 0.1");


	temporary = sensor_count();

	itoa(temporary,str_temporary,10);
	_delay_ms(2000);
	lcd_puts (str_temporary);



	while(1)
	{

	}

	return 0;
}

/**
 *  Check if all connected temperature sensors can be reached.
 *
 *  @return uint8_t number of available sensors
 */
uint8_t sensor_count(void)
{
	uint8_t count,i;

	PORTA|=(1<<EN_SENS_A);

	for(i=0;i<4;i++)
	{
		 if( i2c_start(((sensor_address + i) << 1) | READ )) count++;
	}
	PORTA&=(~(1<<EN_SENS_A));

	PORTA|=(1<<EN_SENS_B);

	for(i=0;i<4;i++)
	{
		if( i2c_start(((sensor_address + i) << 1) | READ )) count++;
	}
	PORTA&=(~(1<<EN_SENS_B));

	PORTC|=(1<<EN_SENS_C);

	for(i=0;i<4;i++)
	{
		if( i2c_start(((sensor_address + i) << 1) | READ )) count++;
	}
	PORTC&=(~(1<<EN_SENS_C));


	return count;

}
