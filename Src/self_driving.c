#include "self_driving.h"

/*
 * Change direction and scale speed according to lane detection
 * - weak differential steering on low error
 * - strong differential steering on high error
 * - slower proportional tank-turn on highest error
 * - warning state on all sensor activation and no sensor activation
 * - protection mode on timeout
 * */
void autonomous_drive(int16_t* speed_a, int16_t* speed_b, int16_t* speed_c,
		int16_t* speed_d, uint8_t *left, uint8_t *right, uint8_t *old_left,
		uint8_t *old_right, uint8_t*warning) {
	if (adc_buffer[1] < LANE_THRESHOLD_CEN) {
		*warning = 0;
		if ((adc_buffer[0] < LANE_THRESHOLD_SX)
				&& (adc_buffer[2] < LANE_THRESHOLD_DX)) {
			*speed_a = 0;
			*speed_b = 0;
			*speed_c = 0;
			*speed_d = 0;
			*right = 0;
			*left = 0;
			*old_right = 0;
			*old_left = 0;
			*warning = 1;
		} else if (adc_buffer[0] < LANE_THRESHOLD_SX) {
			*speed_a = weak_differential_correction(*speed_a,
					*speed_a > SPEED_LIMIT, 0);
			*speed_b = weak_differential_correction(*speed_b,
					*speed_b > SPEED_LIMIT, 1);
			*speed_c = weak_differential_correction(*speed_c,
					*speed_c > SPEED_LIMIT, 0);
			*speed_d = weak_differential_correction(*speed_d,
					*speed_d > SPEED_LIMIT, 1);
			*old_left = 1;
			*old_right = 0;
			*left = 0;
			*right = 0;
			auto_counter = HAL_GetTick();
		} else if (adc_buffer[2] < LANE_THRESHOLD_DX) {
			*speed_a = weak_differential_correction(*speed_a,
					*speed_a > SPEED_LIMIT, 1);
			*speed_b = weak_differential_correction(*speed_b,
					*speed_b > SPEED_LIMIT, 0);
			*speed_c = weak_differential_correction(*speed_c,
					*speed_c > SPEED_LIMIT, 1);
			*speed_d = weak_differential_correction(*speed_d,
					*speed_d > SPEED_LIMIT, 0);
			*old_left = 0;
			*old_right = 1;
			*left = 0;
			*right = 0;
			auto_counter = HAL_GetTick();
		} else {
			*old_left = 0;
			*old_right = 0;
			*left = 0;
			*right = 0;
			auto_counter = HAL_GetTick();
		}
	} else if (adc_buffer[0] < LANE_THRESHOLD_SX) {
		*speed_a = strong_differential_correction(*speed_a, *speed_a > SPEED_LIMIT, 0);
		*speed_b = strong_differential_correction(*speed_b, *speed_b > SPEED_LIMIT, 1);
		*speed_c = strong_differential_correction(*speed_c, *speed_c > SPEED_LIMIT, 0);
		*speed_d = strong_differential_correction(*speed_d, *speed_d > SPEED_LIMIT, 1);
		*warning = 0;
		*old_left = 1;
		*old_right = 0;
		*right = 0;
		*left = 1;
		auto_counter = HAL_GetTick();
	} else if (adc_buffer[2] < LANE_THRESHOLD_DX) {
		*speed_a = strong_differential_correction(*speed_a, *speed_a > SPEED_LIMIT, 1);
		*speed_b = strong_differential_correction(*speed_b, *speed_b > SPEED_LIMIT, 0);
		*speed_c = strong_differential_correction(*speed_c, *speed_c > SPEED_LIMIT, 1);
		*speed_d = strong_differential_correction(*speed_d, *speed_d > SPEED_LIMIT, 0);
		*warning = 0;
		*old_left = 0;
		*old_right = 1;
		*right = 1;
		*left = 0;
		auto_counter = HAL_GetTick();
	} else if (*old_left) {
		*speed_a =
				(*speed_a >= 100) ?
						(*speed_a) * SPEED_CORRECTION_HARD : (*speed_a);
		*speed_b =
				(*speed_b >= 100) ?
						(*speed_b) * SPEED_CORRECTION_HARD : (*speed_b);
		*speed_c =
				(*speed_c >= 100) ?
						(*speed_c) * SPEED_CORRECTION_HARD : (*speed_c);
		*speed_d =
				(*speed_d >= 100) ?
						(*speed_d) * SPEED_CORRECTION_HARD : (*speed_d);
		*left = 1;
		*right = 0;
	} else if (*old_right) {
		*speed_a =
				(*speed_a >= 100) ?
						(*speed_a) * SPEED_CORRECTION_HARD : (*speed_a);
		*speed_b =
				(*speed_b >= 100) ?
						(*speed_b) * SPEED_CORRECTION_HARD : (*speed_b);
		*speed_c =
				(*speed_c >= 100) ?
						(*speed_c) * SPEED_CORRECTION_HARD : (*speed_c);
		*speed_d =
				(*speed_d >= 100) ?
						(*speed_d) * SPEED_CORRECTION_HARD : (*speed_d);
		*left = 0;
		*right = 1;
	} else {
		*warning = 1;
	}

	if ((HAL_GetTick() - auto_counter) >= AUTONOMOUS_TIMEOUT) {
		*speed_a = 0;
		*speed_b = 0;
		*speed_c = 0;
		*speed_d = 0;
		*right = 0;
		*left = 0;
		*old_right = 0;
		*old_left = 0;
		*warning = 1;
	}

}


int16_t weak_differential_correction(uint8_t speed, uint8_t high_speed,
		uint8_t upscale) {
	if (high_speed) {
		if (upscale) {
			return speed * SPEED_CORRECTION_WEAK;
		} else {
			return speed / SPEED_CORRECTION_WEAK;
		}
	} else {
		if (upscale) {
			return speed + speed / SPEED_CORRECTION_DIFF_WEAK;
		} else {
			return speed - speed / SPEED_CORRECTION_DIFF_WEAK;
		}
	}
}


int16_t strong_differential_correction(uint8_t speed, uint8_t high_speed,
		uint8_t upscale) {
	if (high_speed) {
		return speed;
	} else {
		if (upscale) {
			return speed + speed / SPEED_CORRECTION_DIFF_HARD;
		} else {
			return speed - speed / SPEED_CORRECTION_DIFF_HARD;
		}
	}
}
