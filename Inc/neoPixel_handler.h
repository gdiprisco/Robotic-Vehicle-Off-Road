#include "stm32f4xx_hal.h"


#ifndef NEOPIXEL_HANDLER_H
#define NEOPIXEL_HANDLER_H

#define NEOPIXEL_DATA_Pin GPIO_PIN_7
#define NEOPIXEL_DATA_Port GPIOB

void neoPixel_initLeds();

void neoPixel_turnOffLeds();
void neoPixel_light(int strength);
void neoPixel_reverse();
void neoPixel_Welcome();
void neoPixel_WelcomeOff();
void neoPixel_leftArrow();
void neoPixel_leftArrowOff();
void neoPixel_rightArrow();
void neoPixel_rightArrowOff();
void neoPixel_stop();


void neoPixel_turnOnLedsFullWhite();

void neoPixel_reverse() ;

void neoPixel_Welcome() ;

void neoPixel_WelcomeOff();

void neoPixel_leftArrowTemplate(const uint8_t *color);

void neoPixel_rightArrowTemplate(const uint8_t *color) ;

void neoPixel_setColor(int ledIdx, const uint8_t *color);

void neoPixel_applyColors();

void neoPixel_fadeEffect( uint8_t maxLevel );

void neoPixel_warning();

void headlight_manager(uint8_t*headlight, uint8_t*reverse, uint16_t speed,
		uint8_t*warning, uint8_t*headlight_state);

// >>>>> Color definitions
#define	OFF		0
#define FULL	255
#define MID		120
#define SOFT	15

static const uint8_t BLACK[] =
{ OFF, OFF, OFF };
static const uint8_t RED[] =
{ FULL, OFF, OFF };
static const uint8_t GREEN[] =
{ OFF, FULL, OFF };
static const uint8_t BLUE[] =
{ OFF, OFF, FULL };
static const uint8_t ORANGE[] =
{ FULL, 40, OFF };
static const uint8_t WHITE[] =
{ FULL, FULL, FULL };
static const uint8_t SOFT_WHITE[] =
{ SOFT, SOFT, SOFT };
static const uint8_t MID_WHITE[] =
{ MID, MID, MID };
static const uint8_t MID_RED[] =
{ MID, OFF, OFF };
static const uint8_t MID_GREEN[] =
{ OFF, MID, OFF };
static const uint8_t MID_BLUE[] =
{ OFF, OFF, MID };
static const uint8_t SOFT_RED[] =
{ SOFT, OFF, OFF };
static const uint8_t SOFT_GREEN[] =
{ OFF, SOFT, OFF };
static const uint8_t SOFT_BLUE[] =
{ OFF, OFF, SOFT };
// <<<<< Color definitions


#endif
