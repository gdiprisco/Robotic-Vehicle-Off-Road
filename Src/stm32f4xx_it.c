/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
extern uint8_t buffer[4];
extern uint8_t speed_buffer[3];
extern uint8_t AUTONOMOUS;
extern uint8_t RIGHT;
extern uint8_t LEFT;
extern uint8_t PRINT_INFO;
extern uint8_t headlight;
extern uint8_t warning;
extern uint8_t rearlight;
extern UART_HandleTypeDef huart6;

extern void Enable_drivers();
extern void Disable_drivers(UART_HandleTypeDef huart);

//F000 : forward with 000 speed .
//b000 : back with 000 speed .
//L000 : forward turn left with 000 speed .
//R000 : forward turn right with 000 speed .
//l000 : back turn left with 000 speed .
//r000 : back turn right with 000 speed .
//A000 : autonomous drive forward with 000 speed .
//S000 : emergency stop all.
//H00x : light intensity in x mode.
//Plan : show lane sensor informations
//Pagy : show accelerometer and gyroscope informations
//Pspe : show speed informations

uint8_t decode_buffer(uint8_t*buffer, uint8_t* speed_buffer, uint8_t buffer_len) {
	uint8_t verified = 1;
	switch (buffer[0]) {
	case (uint8_t) 'A': //65
		AUTONOMOUS = 1;
		RIGHT = 0;
		LEFT = 0;
		Enable_drivers();
		rearlight = 0;
		warning = 0;
		break;
	case (uint8_t) 'F': //70
		AUTONOMOUS = 0;
		RIGHT = 0;
		LEFT = 0;
		Enable_drivers();
		rearlight = 0;
		warning = 0;
		break;
	case (uint8_t) 'b': //98
		AUTONOMOUS = 0;
		RIGHT = 1;
		LEFT = 1;
		Enable_drivers();
		rearlight = 1;
		warning = 0;
		break;
	case (uint8_t) 'L': //76
		AUTONOMOUS = 0;
		RIGHT = 0;
		LEFT = 1;
		Enable_drivers();
		rearlight = 0;
		warning = 0;
		break;
	case (uint8_t) 'R': //82
		AUTONOMOUS = 0;
		RIGHT = 1;
		LEFT = 0;
		Enable_drivers();
		rearlight = 0;
		warning = 0;
		break;
	case (uint8_t) 'l': //108
		AUTONOMOUS = 0;
		RIGHT = 1;
		LEFT = 0;
		Enable_drivers();
		rearlight = 1;
		warning = 0;
		break;
	case (uint8_t) 'r': //114
		AUTONOMOUS = 0;
		RIGHT = 0;
		LEFT = 1;
		Enable_drivers();
		rearlight = 1;
		warning = 0;
		break;
	case (uint8_t) 'S': //83
		AUTONOMOUS = 0;
		RIGHT = 0;
		LEFT = 0;
		Disable_drivers(huart6);
		buffer[1] = 48;
		buffer[2] = 48;
		buffer[3] = 48;
		speed_buffer[0]=0;
		speed_buffer[1]=0;
		speed_buffer[2]=0;
		warning = 1;
		break;
	case (uint8_t) 'H': //72
		if ((buffer[1] == (uint8_t) '0') && (buffer[2] == (uint8_t) '0')) {
			headlight =
					((buffer[3] >= (uint8_t) '0')
							&& (buffer[3] <= (uint8_t) '3')) ?
							((uint8_t) buffer[3] - 48) : headlight;
			return 1;
		}
		return 0;
	case (uint8_t) 'P': //80
		if (buffer[1] == (uint8_t) 'l' && buffer[2] == (uint8_t) 'a'
				&& buffer[3] == (uint8_t) 'n') {
			PRINT_INFO = 1;
			return 1;
		} else if (buffer[1] == (uint8_t) 'a' && buffer[2] == (uint8_t) 'g'
				&& buffer[3] == (uint8_t) 'y') {
			PRINT_INFO = 2;
			return 1;
		} else if (buffer[1] == (uint8_t) 's' && buffer[2] == (uint8_t) 'p'
				&& buffer[3] == (uint8_t) 'e') {
			PRINT_INFO = 3;
			return 1;
		} else
			return 0;
	default:
		return 0;
	}

	for (uint8_t i = 1; (i < buffer_len) && (verified); i++) {

		if (buffer[i] >= 48 && buffer[i] <= 57) {
			verified = 1;
		} else {
			verified = 0;
		}
	}

	if (verified) {
		for (uint8_t i = 1; i < buffer_len; i++) {
			speed_buffer[i - 1] = buffer[i];
		}
		return 1;
	} else {
		return 0;
	}

}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_tim4_ch2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 stream3 global interrupt.
*/
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_ch2);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*) buffer, sizeof(buffer));
	decode_buffer(buffer, speed_buffer, 4);
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
