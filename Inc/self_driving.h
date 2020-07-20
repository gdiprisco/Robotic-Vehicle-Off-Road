#include "stm32f4xx_hal.h"

#define LANE_THRESHOLD_SX 1000
#define LANE_THRESHOLD_CEN 2000
#define LANE_THRESHOLD_DX 1000
#define AUTONOMOUS_TIMEOUT 3000L
#define SPEED_CORRECTION_DIFF_WEAK 3
#define SPEED_CORRECTION_DIFF_HARD 2
#define SPEED_CORRECTION_WEAK 1.3
#define SPEED_CORRECTION_HARD 0.9
#define SPEED_LIMIT 90

extern uint32_t adc_buffer[3];
extern uint32_t auto_counter;

void autonomous_drive(int16_t* speed_a, int16_t* speed_b, int16_t* speed_c,
		int16_t* speed_d, uint8_t *left, uint8_t *right, uint8_t *old_left,
		uint8_t *old_right, uint8_t*warning);
int16_t weak_differential_correction(uint8_t speed, uint8_t high_speed,
		uint8_t upscale);
int16_t strong_differential_correction(uint8_t speed, uint8_t high_speed,
		uint8_t upscale);
