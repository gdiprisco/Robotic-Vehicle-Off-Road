#include "stm32f4xx_hal.h"

#define SPEED_THRESHOLD 1000
#define MOTOR_DRIVER_FRONT_ADDRESS 135
#define MOTOR_DRIVER_REAR_ADDRESS 133
#define CHECKSUM_MASK 0b01111111
#define KU 0.3
#define KP 0.6*KU
#define KI_H 0.2
#define KD_H 0.08
#define KI_L 0.4
#define KD_L 0
#define WINDUP 250
#define PULSES_PER_REVOLUTION 1

void Enable_drivers();
void Disable_drivers(UART_HandleTypeDef huart);

int16_t calculate_code_speed(int16_t RPM_speed);
void drive_motor(char address, char mode, char speed, UART_HandleTypeDef huart);
uint16_t calculate_numeral_speed(uint8_t *speed);
int16_t PID(int16_t speed, int16_t*last_error, int16_t*integral,
		int16_t*derivative, int16_t desired_speed);
int16_t encoder(uint16_t cnt1, uint16_t cnt2, TIM_HandleTypeDef htim,
		uint8_t fast_timer);
void change_direction(int16_t*speed_a, int16_t*speed_b, int16_t*speed_c,
		int16_t*speed_d, uint8_t*left, uint8_t*right);
