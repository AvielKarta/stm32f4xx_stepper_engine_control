/*
 * 		main.c
 *      Author:  Aviel karta
 */

#include "stm32f407xx.h"
#include <string.h>

/*
 * LED MAPPING:
 * =============
 * BLUE(GPIOD,15)
 * RED(GPIOD,14)
 * ORANGE(GPIOD,13)
 * GREEN(GPIOD,12)
 *
 * ENGINE MAPPING
 * ===============
 * IN1	GPIOD_PIN8
 * IN2	GPIO_E_PIN15
 * IN3	GPIO_D_PIN10
 * IN4	GPIO_D_PIN11
 * */

#define MOTOR_SINGLE_STEP 		5
#define MOTOR_EIGTH_ROUND 		13*MOTOR_SINGLE_STEP
#define MOTOR_QUARTER_ROUND 	2*MOTOR_EIGTH_ROUND
#define MOTOR_HALF_ROUND 		2*MOTOR_QUARTER_ROUND
#define MOTOR_FULL_ROUND 		2*MOTOR_HALF_ROUND
#define M_SEC					1000
#define CW						1
#define RCW						0

void delay(int timeout)
{
	for(uint32_t i = 0 ; i < timeout ; i ++);
}

//void en_gpio(GPIO_Handle_t gpio_handler, GPIO_RegDef_t* port, int pin)
//{
//	gpio_configure_pin(&gpio_handler, port, pin, GPIO_MODE_OUT, GPIO_SPPED_LOW, GPIO_OUT_MODE_PP, GPIO_DIS_PUPD, 0);
//	gpio_init(&gpio_handler);
//}





void gpio_driver_function(GPIO_Handle_t in1, GPIO_Handle_t in2, GPIO_Handle_t in3, GPIO_Handle_t in4, GPIO_Handle_t green, GPIO_Handle_t orange, GPIO_Handle_t red, GPIO_Handle_t blue)
{

	gpio_configure_pin(&in1, GPIOD, 8, GPIO_MODE_OUT, GPIO_SPPED_LOW, GPIO_OUT_MODE_PP, GPIO_DIS_PUPD, 0);
	gpio_init(&in1);
	gpio_configure_pin(&in2, GPIOE, 15, GPIO_MODE_OUT, GPIO_SPPED_LOW, GPIO_OUT_MODE_PP, GPIO_DIS_PUPD, 0);
	gpio_init(&in2);
	gpio_configure_pin(&in3, GPIOD, 10, GPIO_MODE_OUT, GPIO_SPPED_LOW, GPIO_OUT_MODE_PP, GPIO_DIS_PUPD, 0);
	gpio_init(&in3);
	gpio_configure_pin(&in4, GPIOD, 11, GPIO_MODE_OUT, GPIO_SPPED_LOW, GPIO_OUT_MODE_PP, GPIO_DIS_PUPD, 0);
	gpio_init(&in4);

}

void motor_run(int timeout, int cycles, int cw)
{

	int steps[4][4];
	int cw_values[4][4] = {{1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}};
	int rcw_values[4][4] = {{0,0,0,1}, {0,0,1,0}, {0,1,0,0}, {1,0,0,0}};

	for (int i=0; i<4; i++)
	{
		for (int j=0; j<4; j++)
		{
			if (cw == CW)
			{
				steps[i][j] = cw_values[i][j];
			}
			else if (cw == RCW)
			{
				steps[i][j] = rcw_values[i][j];
			}


		}
	}


	for (int cycle=0; cycle<cycles; cycle++)
	{
		for (int values_set = 0; values_set < 4; values_set++)
		{
			gpio_write_to_pin(GPIOD, 8, steps[values_set][0]);
			gpio_write_to_pin(GPIOE, 15, steps[values_set][1]);
			gpio_write_to_pin(GPIOD, 10, steps[values_set][2]);
			gpio_write_to_pin(GPIOD, 11, steps[values_set][3]);
			delay(timeout);
		}
	}
}

int main(void)
{
	GPIO_Handle_t in1,in2, in3, in4, green_led, orange_led, red_led, blue_led;
	gpio_driver_function(in1, in2, in3, in4, green_led, orange_led, red_led, blue_led);

	for (int i=0; i<10 ; i++)
	{
	motor_run(2*M_SEC, MOTOR_QUARTER_ROUND, CW);
	delay(5*M_SEC);
	motor_run(2*M_SEC, MOTOR_QUARTER_ROUND, RCW);
	}

return 0;
}


