/*
 * led_control.c
 *
 *  Created on: 6 May 2021
 *      Author: Quentin Delfosse
 */

#include "hal.h"
#include "led_control.h"

enum {MOVING = 0, BREAKING = 1, STOPPED = 2, LEFT_TURN = 3, RIGHT_TURN = 4};

void led_control(uint8_t state) {
	switch (state) {
	case MOVING:
		palSetPad(GPIOD, GPIOD_LED_FRONT);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
		break;
	case STOPPED:
		palClearPad(GPIOD, GPIOD_LED_FRONT);
		palSetPad(GPIOD, GPIOD_LED3);
		palClearPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
		break;
	case LEFT_TURN:
		palSetPad(GPIOD, GPIOD_LED_FRONT);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palTogglePad(GPIOD, GPIOD_LED7);
		break;
	case RIGHT_TURN:
		palSetPad(GPIOD, GPIOD_LED_FRONT);
		palTogglePad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
		break;
	default:
		palClearPad(GPIOD, GPIOD_LED_FRONT);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED7);
		break;
	}
}

