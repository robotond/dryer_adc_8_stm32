/*
 * utility.h
 *
 *  Created on: Sep 1, 2023
 *      Author: tjp
 */

#ifndef INC_UTILITY_H_
#define INC_UTILITY_H_
#define ON(PIN) HAL_GPIO_WritePin(PIN ## _GPIO_Port, PIN ## _Pin, GPIO_PIN_SET)
#define OFF(PIN) HAL_GPIO_WritePin(PIN ## _GPIO_Port, PIN ## _Pin, GPIO_PIN_RESET)
#define TOGGLE(PIN) HAL_GPIO_TogglePin(PIN ## _GPIO_Port, PIN ## _Pin)
#define SHORT_TIMEOUT 10

void test_leds(void);

void test_leds(){
	ON(RED);
	HAL_Delay(10);
	ON(GREEN);
	HAL_Delay(10);
	ON(BLUE1);
	HAL_Delay(10);
	ON(BLUE2);
	HAL_Delay(10);
	ON(BLUE3);
	HAL_Delay(10);
	OFF(RED);
	HAL_Delay(10);
	OFF(GREEN);
	HAL_Delay(10);
	OFF(BLUE1);
	HAL_Delay(10);
	OFF(BLUE2);
	HAL_Delay(10);
	OFF(BLUE3);
	HAL_Delay(10);

}




#endif /* INC_UTILITY_H_ */
