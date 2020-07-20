#include "neoPixel_handler.h"
#include "math.h"

extern TIM_HandleTypeDef htim4;

#define H_VAL 75
#define L_VAL 30

#define N_LEDS 32

#define BITS_PER_LED 3*8
#define BIT_BUF_SIZE (N_LEDS+1)*BITS_PER_LED

uint32_t neoPixelBuf[BIT_BUF_SIZE];

/*
 * headlight_state protocol
 *	0 - light 0
 *	1 - light 1
 *	2 - light 2
 *	3 - light 3
 *	4 - stop
 *	5 - reverse
 *	6 - warning
*/
void headlight_manager(uint8_t*headlight, uint8_t*reverse, uint16_t speed,
		uint8_t*warning, uint8_t*headlight_state) {
	if (*warning) {
		if (*headlight_state != 6) {
			neoPixel_warning();
			*headlight_state = 6;
		}
	} else if (speed == 0) {
		if (*headlight_state != 4) {
			neoPixel_stop();
			*headlight_state = 4;
		}
	} else if (*reverse) {
		if (*headlight_state != 5) {
			neoPixel_reverse();
			*headlight_state = 5;
		}
	} else if (*headlight_state != *headlight) {
		neoPixel_light(*headlight);
		*headlight_state = *headlight;
	}
}

void neoPixel_initLeds() {
	for (int i = 0; i < BIT_BUF_SIZE; i++)
		neoPixelBuf[i] = 0;

	neoPixel_turnOffLeds();
}

void neoPixel_turnOffLeds() {
	for (int i = 0; i < N_LEDS; i++)
		neoPixel_setColor(i, BLACK);

	neoPixel_applyColors();
}

void neoPixel_warning() {
	for (int i = 0; i < N_LEDS; i++)
		neoPixel_setColor(i, ORANGE);

	neoPixel_applyColors();
}

void neoPixel_turnOnLedsFullWhite() {
	for (int i = 0; i < N_LEDS; i++)
		neoPixel_setColor(i, WHITE);

	neoPixel_applyColors();
}

void neoPixel_light(int strength) {
	const uint8_t *color;

	if (strength == 0) {
		color = BLACK;
	}
	if (strength == 1) {
		color = SOFT_WHITE;
	}
	if (strength == 2) {
		color = MID_WHITE;
	}
	if (strength == 3) {
		color = WHITE;
	}

	for (int i = 0; i < 32; i++)
		neoPixel_setColor(i, color);

	neoPixel_applyColors();

}

void neoPixel_reverse() {
	for (int i = 0; i < 8; i++)
		neoPixel_setColor(i, SOFT_RED);
	neoPixel_setColor(8, SOFT_RED);
	neoPixel_setColor(15, SOFT_RED);
	neoPixel_setColor(16, SOFT_RED);
	neoPixel_setColor(23, SOFT_RED);

	for (int i = 24; i < 32; i++)
		neoPixel_setColor(i, SOFT_RED);

	for (int i = 9; i < 15; i++)
		neoPixel_setColor(i, SOFT_WHITE);

	for (int i = 17; i < 23; i++)
		neoPixel_setColor(i, SOFT_WHITE);

	neoPixel_applyColors();
}

void neoPixel_Welcome() {
	neoPixel_turnOffLeds();
	neoPixel_applyColors();
	neoPixel_setColor(0, GREEN);
	neoPixel_setColor(3, GREEN);
	neoPixel_setColor(4, GREEN);
	neoPixel_setColor(7, GREEN);
	neoPixel_setColor(9, GREEN);
	neoPixel_setColor(10, GREEN);
	neoPixel_setColor(13, GREEN);
	neoPixel_setColor(14, GREEN);
	neoPixel_setColor(17, GREEN);
	neoPixel_setColor(18, GREEN);
	neoPixel_setColor(21, GREEN);
	neoPixel_setColor(22, GREEN);
	neoPixel_setColor(24, GREEN);
	neoPixel_setColor(27, GREEN);
	neoPixel_setColor(28, GREEN);
	neoPixel_setColor(31, GREEN);

	neoPixel_applyColors();
}

void neoPixel_WelcomeOff() {
	neoPixel_turnOffLeds();
	neoPixel_applyColors();
}

void neoPixel_leftArrowTemplate(const uint8_t *color) {
	neoPixel_setColor(0, color);
	neoPixel_setColor(8, color);
	neoPixel_setColor(16, color);
	neoPixel_setColor(24, color);

	neoPixel_applyColors();
}

void neoPixel_leftArrow() {
	neoPixel_leftArrowTemplate(ORANGE);
}

void neoPixel_leftArrowOff() {
	neoPixel_leftArrowTemplate(BLACK);
}

void neoPixel_rightArrow() {
	neoPixel_rightArrowTemplate(ORANGE);
}

void neoPixel_rightArrowOff() {
	neoPixel_rightArrowTemplate(BLACK);
}

void neoPixel_rightArrowTemplate(const uint8_t *color) {
	neoPixel_setColor(7, color);
	neoPixel_setColor(15, color);
	neoPixel_setColor(23, color);
	neoPixel_setColor(31, color);
	neoPixel_applyColors();
}

void neoPixel_stop() {
	for (int i = 0; i < 32; i++)
		neoPixel_setColor(i, SOFT_RED);

	neoPixel_applyColors();
}

void neoPixel_setColor(int ledIdx, const uint8_t *color) {
	if (ledIdx >= N_LEDS)
		return;

	uint8_t r = color[0];
	uint8_t g = color[1];
	uint8_t b = color[2];

	int i = ledIdx * BITS_PER_LED;
	uint8_t mask;
	mask = 0x80;
	while (mask) {
		neoPixelBuf[i] = (mask & g) ? H_VAL : L_VAL;
		mask >>= 1;
		i++;
	}
	mask = 0x80;
	while (mask) {
		neoPixelBuf[i] = (mask & r) ? H_VAL : L_VAL;
		mask >>= 1;
		i++;
	}
	mask = 0x80;
	while (mask) {
		neoPixelBuf[i] = (mask & b) ? H_VAL : L_VAL;
		mask >>= 1;
		i++;
	}
}

void neoPixel_applyColors() {
	HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t*) neoPixelBuf,
			sizeof(neoPixelBuf) / sizeof(neoPixelBuf[0]));
}

