/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "motor_control.h"
#include "self_driving.h"
#include "neoPixel_handler.h"
#define CONTROL_CYCLE_PERIOD 50L
#define DEBUG_COUNT 2
#define N_SPEED_SAMPLES 1//1

#define INDICATION_DELAY  1000 /*!< LED is ON for at least this period [ms] */
#define X_POWER_UP_DELAY    100 /*!< Delay after accelero power-up [ms] */
#define G_POWER_UP_DELAY    150 /*!< Delay after gyro power-up [ms] */
#define N_SAMPLES  5 /*!< Number of samples */
#define UART_TRANSMIT_TIMEOUT  5000
#define MAX_BUF_SIZE 256
static char dataOut[MAX_BUF_SIZE];
static void *LSM6DSL_X_0_handle = NULL;
static void *LSM6DSL_G_0_handle = NULL;
uint8_t reg_addr[] = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
		0x19 };
uint8_t x_st_reg_values[] = { 0x38, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00 };
uint8_t g_st_reg_values[] = { 0x00, 0x5C, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00 };
#define ST_REG_COUNT  (sizeof(reg_addr) / sizeof(uint8_t))
typedef enum {
	LED2 = 0
} Led_TypeDef;
typedef enum {
	STATUS_SELFTEST, STATUS_SLEEP
} DEMO_STATUS;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t buffer[4], speed_buffer[3];
uint32_t lane_sensor[3], adc_buffer[3];
uint8_t debug_cycle = 0;
uint8_t RIGHT = 0, LEFT = 0, AUTONOMOUS = 0, PRINT_INFO = 0, headlight = 0,
		rearlight = 0, OLD_LEFT = 0, OLD_RIGHT = 0, warning = 0;
uint32_t auto_counter = 0;
uint8_t headlight_state = 0, lightcycle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static DrvStatusTypeDef Init_All_Sensors(void);
static DrvStatusTypeDef Enable_All_Sensors(void);
static DrvStatusTypeDef LSM6DSL_X_SelfTest(UART_HandleTypeDef huart);
static DrvStatusTypeDef LSM6DSL_G_SelfTest(UART_HandleTypeDef huart);
static DrvStatusTypeDef LSM6DSL_X_Get_Data(SensorAxes_t *data);
static DrvStatusTypeDef LSM6DSL_G_Get_Data(SensorAxes_t *data);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint16_t cnt_a_1, cnt_b_1, cnt_c_1, cnt_d_1;
	uint16_t cnt_a_2, cnt_b_2, cnt_c_2, cnt_d_2;
	uint32_t tick = 0;
	char msg[2000];
	uint16_t user_desired_speed = 0;
	int16_t desired_speed_a = 0, desired_speed_b = 0, desired_speed_c = 0,
			desired_speed_d = 0;
	int16_t speed_a = 0, speed_b = 0, speed_c = 0, speed_d = 0;
	int16_t buffer_speed_a = 0, buffer_speed_b = 0, buffer_speed_c = 0,
			buffer_speed_d = 0;
	int16_t average_speed_a = 0, average_speed_b = 0, average_speed_c = 0,
			average_speed_d = 0;
	int16_t forced_speed_a = 0, forced_speed_b = 0, forced_speed_c = 0,
			forced_speed_d = 0;
	uint8_t speed_cycle_counter = 0;
	int16_t integral_a = 0, integral_b = 0, integral_c = 0, integral_d = 0;
	int16_t derivative_a = 0, derivative_b = 0, derivative_c = 0, derivative_d =
			0;
	int16_t last_error_a = 0, last_error_b = 0, last_error_c = 0, last_error_d =
			0;

	uint8_t fast_tim_a = ((TIM1->SMCR & 0x3) == 0x3) ? 1 : 0;
	uint8_t fast_tim_b = ((TIM2->SMCR & 0x3) == 0x3) ? 1 : 0;
	uint8_t fast_tim_c = ((TIM3->SMCR & 0x3) == 0x3) ? 1 : 0;
	uint8_t fast_tim_d = ((TIM5->SMCR & 0x3) == 0x3) ? 1 : 0;

	for (int i = 0; i < 4; i++) {
		speed_buffer[i] = 48;
		buffer[i] = 48;
	}
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM4_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, adc_buffer, 3);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	HAL_ADC_Start_IT(&hadc1);
	snprintf(dataOut, MAX_BUF_SIZE,
			"\r\n----ROVER----\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) dataOut, strlen(dataOut), 0xFFFF);

	HAL_Delay(200);

	if (Init_All_Sensors() == COMPONENT_ERROR) {
		snprintf(dataOut, MAX_BUF_SIZE, "\r\nINIT SENSORS ERROR\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) dataOut, strlen(dataOut),
		UART_TRANSMIT_TIMEOUT);

	}

	if (Enable_All_Sensors() == COMPONENT_ERROR) {
		snprintf(dataOut, MAX_BUF_SIZE, "\r\nENABLE SENSORS ERROR\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *) dataOut, strlen(dataOut),
		UART_TRANSMIT_TIMEOUT);

	}

	// >>>>> WS2812 RGB Led
	neoPixel_initLeds();
	Enable_drivers();

	drive_motor(MOTOR_DRIVER_FRONT_ADDRESS, 4, forced_speed_a, huart6);
	drive_motor(MOTOR_DRIVER_FRONT_ADDRESS, 0, forced_speed_b, huart6);
	drive_motor(MOTOR_DRIVER_REAR_ADDRESS, 4, forced_speed_c, huart6);
	drive_motor(MOTOR_DRIVER_REAR_ADDRESS, 0, forced_speed_d, huart6);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); //MOTOR A ENCODER
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //MOTOR B ENCODER
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //MOTOR C ENCODER
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); //MOTOR D ENCODER

	cnt_a_1 = __HAL_TIM_GET_COUNTER(&htim1);
	cnt_b_1 = __HAL_TIM_GET_COUNTER(&htim2);
	cnt_c_1 = __HAL_TIM_GET_COUNTER(&htim3);
	cnt_d_1 = __HAL_TIM_GET_COUNTER(&htim5);
	tick = HAL_GetTick();
	HAL_Delay(1000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (HAL_GetTick() - tick > CONTROL_CYCLE_PERIOD) {
			cnt_a_2 = __HAL_TIM_GET_COUNTER(&htim1);
			cnt_b_2 = __HAL_TIM_GET_COUNTER(&htim2);
			cnt_c_2 = __HAL_TIM_GET_COUNTER(&htim3);
			cnt_d_2 = __HAL_TIM_GET_COUNTER(&htim5);
			speed_a = encoder(cnt_a_1, cnt_a_2, htim1, fast_tim_a);
			speed_b = encoder(cnt_b_1, cnt_b_2, htim2, fast_tim_b);
			speed_c = encoder(cnt_c_1, cnt_c_2, htim3, fast_tim_c);
			speed_d = encoder(cnt_d_1, cnt_d_2, htim5, fast_tim_d);
			buffer_speed_a += speed_a;
			buffer_speed_b += speed_b;
			buffer_speed_c += speed_c;
			buffer_speed_d += speed_d;

			speed_cycle_counter += 1;
			if (speed_cycle_counter >= N_SPEED_SAMPLES) {
				average_speed_a = buffer_speed_a / N_SPEED_SAMPLES;
				average_speed_b = buffer_speed_b / N_SPEED_SAMPLES;
				average_speed_c = buffer_speed_c / N_SPEED_SAMPLES;
				average_speed_d = buffer_speed_d / N_SPEED_SAMPLES;
				user_desired_speed = calculate_numeral_speed(speed_buffer);

				desired_speed_a = user_desired_speed;
				desired_speed_b = user_desired_speed;
				desired_speed_c = user_desired_speed;
				desired_speed_d = user_desired_speed;

				if (AUTONOMOUS) {
					autonomous_drive(&desired_speed_a, &desired_speed_b,
							&desired_speed_c, &desired_speed_d, &LEFT, &RIGHT,
							&OLD_LEFT, &OLD_RIGHT, &warning);
				} else {
					OLD_RIGHT = 0;
					OLD_LEFT = 0;
				}

				change_direction(&desired_speed_a, &desired_speed_b,
						&desired_speed_c, &desired_speed_d, &LEFT, &RIGHT);

				forced_speed_a = PID(average_speed_a, &last_error_a,
						&integral_a, &derivative_a, desired_speed_a);
				forced_speed_b = PID(average_speed_b, &last_error_b,
						&integral_b, &derivative_b, desired_speed_b);

				forced_speed_c = PID(average_speed_c, &last_error_c,
						&integral_c, &derivative_c, desired_speed_c);
				forced_speed_d = PID(average_speed_d, &last_error_d,
						&integral_d, &derivative_d, desired_speed_d);

				/******************************DEBUG INTERFACE***************************/
//				if (debug_cycle >= DEBUG_COUNT) {
//					HAL_UART_Transmit(&huart2, (uint8_t*) buffer,
//							4 * sizeof(char), 0xFFFFFF);
//					sprintf(msg, "\r\nTICK: %lu\tSPEED INPUT: %d\r\n"
//							"LANE_SX:%lu\tLANE_CEN:%lu\tLANE_DX:%lu\r\n"
//							"LANE_SX:%d\tLANE_CEN:%d\tLANE_DX:%d\r\n",
//							(unsigned long) tick, user_desired_speed,
//							adc_buffer[0], adc_buffer[1], adc_buffer[2],
//							(adc_buffer[0] < LANE_THRESHOLD_SX),
//							(adc_buffer[1] < LANE_THRESHOLD_CEN),
//							(adc_buffer[2] < LANE_THRESHOLD_DX));
//					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
//							0xFFFF);
//					sprintf(msg,
//							"MOTOR_A: %d\tMOTOR_B: %d\tMOTOR_C: %d\tMOTOR_D: %d\r\n",
//							forced_speed_a, forced_speed_b, forced_speed_c,
//							forced_speed_d);
//					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
//							0xFFFF);
//					debug_cycle = 0;
//					if (LSM6DSL_X_SelfTest(huart2) == COMPONENT_ERROR) {
//						snprintf(dataOut, MAX_BUF_SIZE,
//								"\r\n LSM6DSL_X SENSOR ERROR\r\n");
//						HAL_UART_Transmit(&huart2, (uint8_t *) dataOut,
//								strlen(dataOut), UART_TRANSMIT_TIMEOUT);
//					}
//					if (LSM6DSL_G_SelfTest(huart2) == COMPONENT_ERROR) {
//						snprintf(dataOut, MAX_BUF_SIZE,
//								"\r\n LSM6DSL_G SENSOR ERROR\r\n");
//						HAL_UART_Transmit(&huart2, (uint8_t *) dataOut,
//								strlen(dataOut), UART_TRANSMIT_TIMEOUT);
//					}
//				} else {
//					debug_cycle++;
//				}
				/******************************END DEBUG INTERFACE***********************/
				drive_motor(MOTOR_DRIVER_FRONT_ADDRESS,
						((forced_speed_a < 0) ? 4 : 5), abs(forced_speed_a),
						huart6);
				drive_motor(MOTOR_DRIVER_FRONT_ADDRESS,
						((forced_speed_b < 0) ? 1 : 0), abs(forced_speed_b),
						huart6);
				drive_motor(MOTOR_DRIVER_REAR_ADDRESS,
						((forced_speed_c < 0) ? 4 : 5), abs(forced_speed_c),
						huart6);
				drive_motor(MOTOR_DRIVER_REAR_ADDRESS,
						((forced_speed_d < 0) ? 1 : 0), abs(forced_speed_d),
						huart6);

				if (lightcycle <= 0) {
					headlight_manager(&headlight, &rearlight,
							(average_speed_a || average_speed_b
									|| average_speed_c || average_speed_d),
							&warning, &headlight_state);
					lightcycle = 10;
				} else {
					lightcycle--;
				}

				buffer_speed_a = 0;
				buffer_speed_b = 0;
				buffer_speed_c = 0;
				buffer_speed_d = 0;
				speed_cycle_counter = 0;
			}

			switch (PRINT_INFO) {
			case 1:
				sprintf(msg, "L\t\tC\t\tR\r\n"
						" %lu(%d)\t%lu(%d)\t%lu(%d)\r\n"
						"OLD_LEFT LEFT RIGHT OLD_RIGHT: %d %d %d %d\r\n",
						adc_buffer[0], (adc_buffer[0] < LANE_THRESHOLD_SX),
						adc_buffer[1], (adc_buffer[1] < LANE_THRESHOLD_CEN),
						adc_buffer[2], (adc_buffer[2] < LANE_THRESHOLD_DX),
						OLD_LEFT, LEFT, RIGHT, OLD_RIGHT);
				HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), 0xFFFF);
				break;
			case 2:
				if (LSM6DSL_X_SelfTest(huart1) == COMPONENT_ERROR) {
					snprintf(dataOut, MAX_BUF_SIZE,
							"\r\n LSM6DSL_X SENSOR ERROR\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t *) dataOut,
							strlen(dataOut),
							UART_TRANSMIT_TIMEOUT);
				}

				if (LSM6DSL_G_SelfTest(huart1) == COMPONENT_ERROR) {
					snprintf(dataOut, MAX_BUF_SIZE,
							"\r\n LSM6DSL_G SENSOR ERROR\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t *) dataOut,
							strlen(dataOut),
							UART_TRANSMIT_TIMEOUT);
				}
				break;
			case 3:
				sprintf(msg, "Front-L: %d\r\n"
						"Front-R: %d\r\n"
						"Rear-L: %d\r\n"
						"Rear-R: %d\r\n", average_speed_a, average_speed_b,
						average_speed_c, average_speed_d);
				HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), 0xFFFF);
				break;
			}
			PRINT_INFO = 0;

			tick = HAL_GetTick();
			cnt_a_1 = __HAL_TIM_GET_COUNTER(&htim1);
			cnt_b_1 = __HAL_TIM_GET_COUNTER(&htim2);
			cnt_c_1 = __HAL_TIM_GET_COUNTER(&htim3);
			cnt_d_1 = __HAL_TIM_GET_COUNTER(&htim5);
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 104;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 65535;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART6 init function */
static void MX_USART6_UART_Init(void) {

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 38400;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;
	__HAL_RCC_DMA2_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC8 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	for (int i = 0; i < 3; i++) {
		lane_sensor[i] = buffer[i];
	}
}

static DrvStatusTypeDef Init_All_Sensors(void) {
	if (BSP_ACCELERO_Init(LSM6DSL_X_0, &LSM6DSL_X_0_handle)
			== COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	if (BSP_GYRO_Init(LSM6DSL_G_0, &LSM6DSL_G_0_handle) == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

static DrvStatusTypeDef Enable_All_Sensors(void) {
	if (BSP_ACCELERO_Sensor_Enable(LSM6DSL_X_0_handle) == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	if (BSP_GYRO_Sensor_Enable(LSM6DSL_G_0_handle) == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

static DrvStatusTypeDef LSM6DSL_X_SelfTest(UART_HandleTypeDef huart) {
	int i = 0;
	SensorAxes_t data_nost;
	SensorAxes_t data;
	/* Wait defined time for stable output */
	HAL_Delay(X_POWER_UP_DELAY);

	/* Read first data and discard it */
	if (LSM6DSL_X_Get_Data(&data) == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	data_nost.AXIS_X = 0;
	data_nost.AXIS_Y = 0;
	data_nost.AXIS_Z = 0;

	/* Read valid data multiple times and average it */
	for (i = 0; i < N_SAMPLES; i++) {
		if (LSM6DSL_X_Get_Data(&data) == COMPONENT_ERROR) {
			return COMPONENT_ERROR;
		}
		data_nost.AXIS_X += data.AXIS_X;
		data_nost.AXIS_Y += data.AXIS_Y;
		data_nost.AXIS_Z += data.AXIS_Z;
	}
	data_nost.AXIS_X /= N_SAMPLES;
	data_nost.AXIS_Y /= N_SAMPLES;
	data_nost.AXIS_Z /= N_SAMPLES;

	/* Print measured data */
	snprintf(dataOut, MAX_BUF_SIZE, "\r\nMeasured acceleration [mg]:\r\n");
	HAL_UART_Transmit(&huart, (uint8_t *) dataOut, strlen(dataOut),
	UART_TRANSMIT_TIMEOUT);
	snprintf(dataOut, MAX_BUF_SIZE, "       X      | %8ld\r\n",
			data_nost.AXIS_X);
	HAL_UART_Transmit(&huart, (uint8_t *) dataOut, strlen(dataOut),
	UART_TRANSMIT_TIMEOUT);
	snprintf(dataOut, MAX_BUF_SIZE, "       Y      | %8ld\r\n",
			data_nost.AXIS_Y);
	HAL_UART_Transmit(&huart, (uint8_t *) dataOut, strlen(dataOut),
	UART_TRANSMIT_TIMEOUT);
	snprintf(dataOut, MAX_BUF_SIZE, "       Z      | %8ld\r\n",
			data_nost.AXIS_Z);
	HAL_UART_Transmit(&huart, (uint8_t *) dataOut, strlen(dataOut),
	UART_TRANSMIT_TIMEOUT);

	return COMPONENT_OK;
}

static DrvStatusTypeDef LSM6DSL_G_SelfTest(UART_HandleTypeDef huart) {
	int i = 0;
	SensorAxes_t data_nost;
	SensorAxes_t data;

	/* Wait defined time for stable output */
	HAL_Delay(G_POWER_UP_DELAY);

	/* Read first data and discard it */
	if (LSM6DSL_G_Get_Data(&data) == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	data_nost.AXIS_X = 0;
	data_nost.AXIS_Y = 0;
	data_nost.AXIS_Z = 0;

	/* Read valid data multiple times and average it */
	for (i = 0; i < N_SAMPLES; i++) {
		if (LSM6DSL_G_Get_Data(&data) == COMPONENT_ERROR) {
			return COMPONENT_ERROR;
		}
		data_nost.AXIS_X += data.AXIS_X;
		data_nost.AXIS_Y += data.AXIS_Y;
		data_nost.AXIS_Z += data.AXIS_Z;
	}
	data_nost.AXIS_X /= N_SAMPLES;
	data_nost.AXIS_Y /= N_SAMPLES;
	data_nost.AXIS_Z /= N_SAMPLES;

	/* Print measured data */
	snprintf(dataOut, MAX_BUF_SIZE,
			"\r\nMeasured angular velocity [mdps]:\r\n");
	HAL_UART_Transmit(&huart, (uint8_t *) dataOut, strlen(dataOut),
	UART_TRANSMIT_TIMEOUT);
	snprintf(dataOut, MAX_BUF_SIZE, "       X      |  %8ld\r\n",
			data_nost.AXIS_X);
	HAL_UART_Transmit(&huart, (uint8_t *) dataOut, strlen(dataOut),
	UART_TRANSMIT_TIMEOUT);
	snprintf(dataOut, MAX_BUF_SIZE, "       Y      |  %8ld\r\n",
			data_nost.AXIS_Y);
	HAL_UART_Transmit(&huart, (uint8_t *) dataOut, strlen(dataOut),
	UART_TRANSMIT_TIMEOUT);
	snprintf(dataOut, MAX_BUF_SIZE, "       Z      |  %8ld\r\n",
			data_nost.AXIS_Z);
	HAL_UART_Transmit(&huart, (uint8_t *) dataOut, strlen(dataOut),
	UART_TRANSMIT_TIMEOUT);

	return COMPONENT_OK;
}

static DrvStatusTypeDef LSM6DSL_X_Get_Data(SensorAxes_t *data) {
	uint8_t status;

	/* Wait for data ready */
	do {
		if (BSP_ACCELERO_Get_DRDY_Status(LSM6DSL_X_0_handle, &status)
				== COMPONENT_ERROR) {
			return COMPONENT_ERROR;
		}
	} while (status == 0);

	/* Read accelero data */
	if (BSP_ACCELERO_Get_Axes(LSM6DSL_X_0_handle, data) == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}

static DrvStatusTypeDef LSM6DSL_G_Get_Data(SensorAxes_t *data) {
	uint8_t status;

	/* Wait for data ready */
	do {
		if (BSP_GYRO_Get_DRDY_Status(LSM6DSL_G_0_handle, &status)
				== COMPONENT_ERROR) {
			return COMPONENT_ERROR;
		}
	} while (status == 0);

	/* Read accelero data */
	if (BSP_GYRO_Get_Axes(LSM6DSL_G_0_handle, data) == COMPONENT_ERROR) {
		return COMPONENT_ERROR;
	}

	return COMPONENT_OK;
}



/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
