//#include "main.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdlib.h"
#include "motor_control.h"

/*Enable motor drivers through GPIO Output*/
void Enable_drivers() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
}

/*Stop motors and disable motor drivers through GPIO Output*/
void Disable_drivers(UART_HandleTypeDef huart) {
	drive_motor(MOTOR_DRIVER_FRONT_ADDRESS, 4, 0, huart);
	drive_motor(MOTOR_DRIVER_FRONT_ADDRESS, 0, 0, huart);
	drive_motor(MOTOR_DRIVER_REAR_ADDRESS, 4, 0, huart);
	drive_motor(MOTOR_DRIVER_REAR_ADDRESS, 0, 0, huart);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
}

/*Convert 3-char-buffer to integer number*/
uint16_t calculate_numeral_speed(uint8_t *speed) {
	uint16_t inserted_speed = 0;
	uint8_t buffer_size = 3;
	for (int i = 0; i < buffer_size; i++) {
		inserted_speed = 10 * inserted_speed + speed[i] - 48;
	}
	return inserted_speed;
}

/*Convert RPM speed to 0-127 speed*/
int16_t calculate_code_speed(int16_t RPM_speed) {
	float speed_code = 127 * RPM_speed / 500;
	if (speed_code > 127)
		return 127;
	if (speed_code < -127) {
		return -127;
	}
	return (int16_t) speed_code;
}

/*Control motor driver in packetized mode with predefined checksum mask*/
void drive_motor(char address, char mode, char speed, UART_HandleTypeDef huart) {
	char checksum = (address + mode + speed) & CHECKSUM_MASK;
	HAL_UART_Transmit(&huart, (uint8_t*) &address, sizeof(char), 0xFFFF);
	HAL_UART_Transmit(&huart, (uint8_t*) &mode, sizeof(char), 0xFFFF);
	HAL_UART_Transmit(&huart, (uint8_t*) &speed, sizeof(char), 0xFFFF);
	HAL_UART_Transmit(&huart, (uint8_t*) &checksum, sizeof(char), 0xFFFF);
}

/*PID controller*/
int16_t PID(int16_t speed, int16_t*last_error, int16_t*integral,
		int16_t*derivative, int16_t desired_speed) {
	int16_t error = desired_speed - speed;

	if (((*integral) >= WINDUP) && (error < 0)) {
		*integral += error;
	} else if (((*integral) <= (-WINDUP)) && (error > 0)) {
		*integral += error;
	} else if (((*integral) >= (-WINDUP)) && ((*integral) <= WINDUP)) {
		*integral += error;
	}

	*derivative = error - *last_error;

	int16_t trigger = desired_speed + KP * error;
	if (abs(desired_speed) <= 60) {
		trigger += KI_L * (*integral) + KD_L * (*derivative);
	} else {
		trigger += KI_H * (*integral) + KD_H * (*derivative);
	}
	*last_error = error;
	return calculate_code_speed(trigger);

}

/*Encoding speed from two-channel-timer in encoder mode with
 * speed thresholding and direction handling
 * */
int16_t encoder(uint16_t cnt1, uint16_t cnt2, TIM_HandleTypeDef htim,
		uint8_t fast_timer) {
	uint16_t diff;
	float speed;
	uint8_t reverse_timer = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim);
	if (reverse_timer) {
		if (cnt2 < cnt1) /* Check for counter underflow */
			diff = cnt1 - cnt2;
		else
			diff = (65535 - cnt2) + cnt1;
		speed = (diff) / PULSES_PER_REVOLUTION;
	} else {
		if (cnt2 > cnt1) /* Check for counter overflow */
			diff = cnt2 - cnt1;
		else
			diff = (65535 - cnt1) + cnt2;
		speed = (diff) / PULSES_PER_REVOLUTION;
	}
	speed = (fast_timer) ? (speed / 2) : speed;
	speed = (speed > SPEED_THRESHOLD) ? 0 : speed;
	speed = (reverse_timer) ? (-speed) : speed;

	return (int16_t) speed;
}

/*Change speed sign according to the direction of the motors*/
void change_direction(int16_t*speed_a, int16_t*speed_b, int16_t*speed_c,
		int16_t*speed_d, uint8_t*left, uint8_t*right) {
	if (!(*left)) {
		*speed_a = -(*speed_a);
		*speed_c = -(*speed_c);
	}
	if (*right) {
		*speed_b = -(*speed_b);
		*speed_d = -(*speed_d);
	}
}
