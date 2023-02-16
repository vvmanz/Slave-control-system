/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _BACK 0
#define _FORW 1
#define NUM_READ 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint16_t adcValue[2] = { 0, };
volatile uint16_t timed;

extern volatile uint8_t adcFlag;

extern volatile uint8_t enc1Value;

extern volatile int32_t enc1Val, resultEncValMot1, realVelMot1, pervTickMot1;
volatile float deltVelMot1;

extern volatile uint16_t enc2Value;
extern volatile int32_t enc2Val, resultEncValMot2, realVelMot2, pervTickMot2;
volatile float deltVelMot2;

extern volatile float angle_1, angle_2;

volatile uint8_t encoderDirectionMot2, encoderDirectionMot1;

extern volatile float cs1RealCur, cs2RealCur;

extern int pid_flag;

int timer_1, timer_2;

int old_emergency_1, current_emergency_1;
int old_emergency_2, current_emergency_2;
int emergency_counter;
float emergency_current;

void emergency() {
//	HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, 1); // Аварийный светодиод
	pid_flag = 3; // Отключение двигателей
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 channel1 global interrupt.
 */
void DMA1_Channel1_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	adcFlag = 1;
	/* USER CODE END DMA1_Channel1_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_adc1);
	/* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

	/* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
 * @brief This function handles TIM1 update interrupt.
 */
void TIM1_UP_IRQHandler(void) {
	/* USER CODE BEGIN TIM1_UP_IRQn 0 */
	if (_ENABLE_MOTOR_1) {
		encoderDirectionMot1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);

		if (encoderDirectionMot1)
			enc1Val -= 32;
		else
			enc1Val += 32;
	}
	/* USER CODE END TIM1_UP_IRQn 0 */
	HAL_TIM_IRQHandler(&htim1);
	/* USER CODE BEGIN TIM1_UP_IRQn 1 */

	/* USER CODE END TIM1_UP_IRQn 1 */
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void) {
	/* USER CODE BEGIN TIM2_IRQn 0 */
	if (_ENABLE_MOTOR_2) {
		encoderDirectionMot2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);

		if (encoderDirectionMot2)
			enc2Val -= 1000;
		else
			enc2Val += 1000;
	}
	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void) {
	/* USER CODE BEGIN TIM3_IRQn 0 */
	pid_flag = 1;
	if (_ENABLE_MOTOR_1) {
		resultEncValMot1 = enc1Val + enc1Value;
		angle_1 = -(float) +resultEncValMot1 / 32 * 360;

	}
	/*----�������� �������������� �������� �����, ������� �������� ��������� �2----*/
	if (_ENABLE_MOTOR_2) {
		resultEncValMot2 = enc2Val + enc2Value;
		angle_2 = (float) +resultEncValMot2 / 1000 * 360;
	}

	/*----���� ����������� �� �������----*/
	timed += 1;
	if (timed >= 30) {
		if (_ENABLE_MOTOR_1) {
			deltVelMot1 = (float) resultEncValMot1 - (float) pervTickMot1;
			pervTickMot1 = resultEncValMot1;
		}
		if (_ENABLE_MOTOR_2) {
			deltVelMot2 = (float) resultEncValMot2 - (float) pervTickMot2;
			pervTickMot2 = resultEncValMot2;
		}
		if (_ENABLE_MOTOR_1) {
			realVelMot1 = -deltVelMot1 / 31 * 92.5 * 60;
		}
		if (_ENABLE_MOTOR_2) {
			realVelMot2 = deltVelMot2 / 999 * 92.5 * 60;
		}
		timed = 0;
	}



	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */

	/* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
