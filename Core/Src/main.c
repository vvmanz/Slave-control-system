/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define NUM_READ 30
#define NUM_READ 30
//UIRS-2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

struct Motor motor_1;
struct Motor motor_2;

volatile uint16_t adcValBuf[2] = { 0, };

volatile uint8_t adcFlag;

volatile uint16_t adcValueCh1, adcValueCh2;
volatile float cs1ADCValFilter, cs2ADCValFilter;

volatile uint16_t enc1Flag, enc2Flag;
volatile uint16_t enc1Value, enc2Value;

int pid_flag;

volatile int32_t enc1Val, resultEncValMot1, pervTickMot1;
volatile int32_t enc2Val, resultEncValMot2, pervTickMot2;

const float pwm_step = 5;

void detect_and_limit_oscillations(struct Motor *motor, float osc_threshold) {
	if (fabs(motor->pwm_difference) > osc_threshold
			&& fabs(motor->err_angle) <= motor->hyst_error) {
		(motor->osc_counter)++;
	} else {
		motor->osc_counter = 0;
	}

	if (motor->osc_counter > 0) {
		if (motor->pwm_difference > 0) {
			motor->out_pwm = motor->previous_pwm + pwm_step;
		} else {
			motor->out_pwm = motor->previous_pwm - pwm_step;
		}
	}
}
//void pid(struct Motor *motor, int choice) {
//	motor->counter_pid++;
//	if (motor->counter_pid >= 300) {
//		motor->task_real = motor->task * 27;
//		motor->hyst_error = 100;
//		motor->err_angle = motor->task_real - motor->angle;
//
//		if (fabs(motor->err_angle) <= motor->hyst_error) {
//			motor->err_angle = 0;
//		}
//
//		motor->out_angle = motor->err_angle * motor->kp_angle;
//		if (motor->out_angle > 8000) {
//			motor->out_angle = 8000;
//		}
//		if (motor->out_angle < -8000) {
//			motor->out_angle = -8000;
//		}
//		motor->err_speed = motor->out_angle - motor->speed;
//
//		if (fabs(motor->err_angle) <= motor->hyst_error) {
//			motor->err_speed = 0;
//		}
//
//		motor->P_speed = motor->err_speed * motor->kp_speed;
//		motor->I_speed += motor->err_speed * motor->ki_speed * 0.1;
//		if (motor->I_speed >= 2.5) {
//			motor->I_speed = 2.5;
//		}
//		if (motor->I_speed <= -2.5) {
//			motor->I_speed = -2.5;
//		}
//		motor->out_speed = motor->P_speed + motor->I_speed;
//
//		if (motor->out_speed >= 2.5) {
//			motor->out_speed = 2.5;
//		}
//		if (motor->out_speed <= -2.5) {
//			motor->out_speed = -2.5;
//		}
//		motor->err_current = motor->out_speed - motor->current;
//
//		if (fabs(motor->err_angle) <= motor->hyst_error) {
//			motor->err_current = 0;
//		}
//
//		motor->P_current = motor->err_current * motor->kp_current;
//		motor->I_current += motor->err_current * motor->ki_current * 0.1;
//
//		if (motor->I_current >= 1) {
//			motor->I_current = 1;
//		}
//		if (motor->I_current <= -1) {
//			motor->I_current = -1;
//		}
//
//		motor->out_current = motor->P_current + motor->I_current;
//
//		if (choice == 1) {
//			if (motor->out_current >= 0) {
//				HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_RESET);
//
//			}
//			if (motor->out_current < 0) {
//				HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_SET);
//			}
//		}
//
//		if (choice == 2) {
//			if (motor->out_current >= 0) {
//				HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_RESET);
//			}
//			if (motor->out_current < 0) {
//				HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_SET);
//			}
//		}
//
//		if (motor->out_current >= 1) {
//			motor->out_current_limited = 1;
//		}
//		if (motor->out_current <= -1) {
//			motor->out_current_limited = -1;
//		}
//		if (motor->out_current > -1 && motor->out_current < 1) {
//			motor->out_current_limited = motor->out_current;
//		}
//
//		motor->out_pwm = fabsf(motor->out_current_limited * 1000);
//		if (motor->out_pwm < 2) {
//			motor->out_pwm = 1;
//		}
//		if (motor->out_pwm > 990) {
//			motor->out_pwm = 990;
//		}
//		motor->pwm_difference = motor->out_pwm - motor->previous_pwm;
//
//		if (fabs(motor->pwm_difference) > pwm_step) {
//			if (motor->pwm_difference > 0) {
//				motor->out_pwm = motor->previous_pwm + pwm_step;
//			} else {
//				motor->out_pwm = motor->previous_pwm - pwm_step;
//			}
//		}
//		if (choice == 1) {
//			TIM3->CCR1 = motor->out_pwm;
//		} else if (choice == 2) {
//			TIM3->CCR2 = motor->out_pwm;
//		}
//		motor->previous_pwm = motor->out_pwm;
//
//		motor->counter_pid = 0;
//	}
//
//}

volatile float tasking;
void pid(struct Motor *motor, float task, int choice) {
//	motor->counter_pid++;
//	motor->task_real = task * 27;
//	motor->hyst_error = 50;
//	if (motor->counter_pid >= 300) {
//		motor->err_angle = motor->task_real - motor->angle;
//
//		if (fabs(motor->err_angle) <= motor->hyst_error) {
//			motor->err_angle = 0;
//
//			if (choice == 1) {
//				TIM3->CCR1 -=2;
//			} else if (choice == 2) {
//				TIM3->CCR2 -=2;
//			}
//		}
//
//		motor->out_angle = motor->err_angle * motor->kp_angle;
//		if (motor->out_angle > 8000) {
//			motor->out_angle = 8000;
//		}
//		if (motor->out_angle < -8000) {
//			motor->out_angle = -8000;
//		}
//		motor->err_speed = motor->out_angle - motor->speed;
//
//		if (fabs(motor->err_angle) <= motor->hyst_error) {
//			motor->err_speed = 0;
////			motor->I_speed -= motor->err_speed * motor->ki_speed * 0.1;
//			motor->I_speed = 0;
//		}
//
//		motor->P_speed = motor->err_speed * motor->kp_speed;
//		motor->I_speed += motor->err_speed * motor->ki_speed * 0.1;
//		if (motor->I_speed >= 2.5) {
//			motor->I_speed = 2.5;
//		}
//		if (motor->I_speed <= -2.5) {
//			motor->I_speed = -2.5;
//		}
//		motor->out_speed = motor->P_speed + motor->I_speed;
//
//		if (motor->out_speed >= 2.5) {
//			motor->out_speed = 2.5;
//		}
//		if (motor->out_speed <= -2.5) {
//			motor->out_speed = -2.5;
//		}
//		motor->counter_pid = 0;
//	}

//	motor->err_current = motor->out_speed - motor->current;
	motor->err_current = task - motor->current;

//	if (fabs(motor->err_angle) <= motor->hyst_error) {
//		motor->err_current = 0;
//		motor->I_current = 0;
////		motor->I_current -= motor->err_current * motor->ki_current * 0.000333;
//	}

	motor->P_current = motor->err_current * motor->kp_current;
	motor->I_current += motor->err_current * motor->ki_current * 0.000333;

	if (motor->I_current >= 1) {
		motor->I_current = 1;
	}
	if (motor->I_current <= -1) {
		motor->I_current = -1;
	}

	motor->out_current = motor->P_current + motor->I_current + motor->speed*0.003375;

	if (choice == 1) {
		if (motor->out_current >= 0) {
			HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_RESET);

		}
		if (motor->out_current < 0) {
			HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_SET);
		}
	}

	if (choice == 2) {
		if (motor->out_current >= 0) {
			HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_RESET);
		}
		if (motor->out_current < 0) {
			HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_SET);
		}
	}

	if (motor->out_current >= 1) {
		motor->out_current_limited = 1;
	}
	if (motor->out_current <= -1) {
		motor->out_current_limited = -1;
	}
	if (motor->out_current > -1 && motor->out_current < 1) {
		motor->out_current_limited = motor->out_current;
	}

	motor->out_pwm = fabsf(motor->out_current_limited * 1000);
	if (motor->out_pwm < 2) {
		motor->out_pwm = 1;
	}
	if (motor->out_pwm > 990) {
		motor->out_pwm = 990;
	}
//	motor->pwm_difference = motor->out_pwm - motor->previous_pwm;

//	detect_and_limit_oscillations(motor, 0.5);

//	if (fabs(motor->pwm_difference) > pwm_step) {
//		if (motor->pwm_difference > 0) {
//			motor->out_pwm = motor->previous_pwm + pwm_step;
//		} else {
//			motor->out_pwm = motor->previous_pwm - pwm_step;
//		}
//	}
	if (choice == 1) {
		TIM3->CCR1 = motor->out_pwm;
	} else if (choice == 2) {
		TIM3->CCR2 = motor->out_pwm;
	}
//	motor->previous_pwm = motor->out_pwm;
}

float cs1Voltage, cs2Voltage;
float cs1Current, cs2Current;

int counter_test, flag_swtich;

int32_t slave_transmit[6];
int32_t slave_receive[6];

int32_t float_to_int32(float value) {
	float scaled_value = value * 1000000.0f;
	int32_t int_value = (int32_t) round(scaled_value);
	return int_value;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int Median_ADC_CS1(int newVal);
int Median_ADC_CS2(int newVal);
float Median_Cur_CS1(float newVal);
float Median_Cur_CS2(float newVal);
float simpleKalman1(float newVal);
float simpleKalman2(float newVal);
void currentSearch(float cs1ADCValFilter, float cs2ADCValFilter);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	motor_1.kp_angle = 0.412;
	motor_1.kp_speed = 0.000085;
	motor_1.ki_speed = 0.000098;
	motor_1.kp_current = 0.0001;
	motor_1.ki_current = 19.0;

	motor_2.kp_angle = 0.4;
	motor_2.kp_speed = 0.000071;
	motor_2.ki_speed = 0.000098;
	motor_2.kp_current = 0.0001;
	motor_2.ki_current = 19.0;

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_SPI2_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcValBuf, 2);

	if (_ENABLE_MOTOR_1) {
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
		HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
		HAL_TIM_Base_Start_IT(&htim1);

		HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_SET);

//		HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_RESET); ПОЛОЖИТЕЛЬНОЕ
		TIM3->CCR1 = 1;
	}

	if (_ENABLE_MOTOR_2) {
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);

		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		HAL_TIM_Base_Start_IT(&htim2);

		HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_RESET);

//		HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_SET); ПОЛОЖИТЕЛЬНОЕ
		TIM3->CCR2 = 1;
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (TIM3->CCR1 == 0) {
			TIM3->CCR1 = 1;
		}
		if (TIM3->CCR2 == 0) {
			TIM3->CCR2 = 1;
		}
		if (adcFlag == 1) {
			adcFlag = 0;
			HAL_ADC_Stop_DMA(&hadc1);
			adcValueCh1 = adcValBuf[0];
			adcValueCh2 = adcValBuf[1];
			adcValBuf[0] = 0;
			adcValBuf[1] = 0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcValBuf, 2);
		}

		if (_ENABLE_MOTOR_1) {
			cs1ADCValFilter = Median_ADC_CS1(adcValueCh1);

		}
		if (_ENABLE_MOTOR_2) {
			cs2ADCValFilter = Median_ADC_CS2(adcValueCh2);
		}

		currentSearch(cs1ADCValFilter, cs2ADCValFilter);

		if (_ENABLE_MOTOR_1) {
			enc1Value = TIM1->CNT;
		}
		if (_ENABLE_MOTOR_2) {
			enc2Value = TIM2->CNT;
		}

		if (pid_flag == 1) {
//			pid(&motor_1, tasking, 1);
			pid(&motor_2, tasking, 2);
			pid_flag = 0;
		}
//
//		counter_test++;
//		if (counter_test > 15000) {
//			motor_2.P_current = 0;
//			motor_2.I_current = 0;
//			motor_2.P_speed = 0;
//			motor_2.I_speed = 0;
//			motor_2.out_angle = 0;
//			tasking += 360;
//			counter_test = 0;
//		}
//		counter_test++;
//		if (counter_test > 9000 && counter_test < 18000) {
//			motor_1.task = 360;
//			motor_2.task = 360;
//		}
//		if (counter_test > 18000 && counter_test < 27000) {
//			motor_1.task = 0;
//			motor_2.task = 0;
//		}
//		if (counter_test > 27000) {
//			counter_test = 0;
//		}

//		HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_RESET);
//		TIM3->CCR2 = 250;
////
//		HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_SET);
//		TIM3->CCR1 = 500;
//		if (_SPI_ENABLE) {
//			slave_transmit[0] = float_to_int32(cs1RealCur);
//			slave_transmit[1] = float_to_int32(realVelMot1);
//			slave_transmit[2] = float_to_int32(angle_1);
//
//			slave_transmit[3] = float_to_int32(cs2RealCur);
//			slave_transmit[4] = float_to_int32(realVelMot2);
//			slave_transmit[5] = float_to_int32(angle_2);
//
//			if (HAL_GPIO_ReadPin(SPI_NSS_GPIO_Port, SPI_NSS_Pin) == 0) {
//
//				HAL_TIM_Base_Start_IT(&htim4);
//				if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET) {
//					__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
//				}
//			}
//		}

//		counter_test++;
//		if (flag_swtich == 0) {
//
//			if (counter_test > 2000) {
//				TIM3->CCR1 += 70;
//				if (TIM3->CCR1 > 800) {
//					flag_swtich = 1;
//					counter_test = 0;
//				}
//				counter_test = 0;
//			}
//
//		if (flag_swtich == 1){
//			if (counter_test > 100){
//				TIM3->CCR1 --;
//				counter_test =0;
//			}
//			if (TIM3->CCR1 <10){
//				flag_swtich = 0;
//				counter_test =0;
//			}
//
//		}

//		if (flag_swtich == 0) {
//			HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_RESET);
//			TIM3->CCR2 = 500;
//
//			HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_SET);
//			TIM3->CCR1 = 500;
//			counter_test++;
//			if (counter_test > 9000) {
//				counter_test = 0;
//				flag_swtich = 1;
//				counter_test = 0;
//			}
//		}
//
//		if (flag_swtich == 1) {
//			counter_test++;
//			if (counter_test > 5) {
//				TIM3->CCR2 --;
//				TIM3->CCR1--;
//				counter_test = 0;
//			}
//			if (TIM3->CCR1 < 5) {
//				counter_test = 0;
//				TIM3->CCR1 = 0;
//				TIM3->CCR2 = 0;
//				flag_swtich = 2;
//
//			}
//		}
//
//		if (flag_swtich == 2) {
//
//			HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_SET);
//			TIM3->CCR2 = 500;
//
//			HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_RESET);
//			TIM3->CCR1 = 500;
//			counter_test++;
//			if (counter_test > 9000) {
//				counter_test = 0;
//				flag_swtich = 3;
//				counter_test = 0;
//			}
//		}
//
//		if (flag_swtich == 3) {
//			counter_test++;
//			if (counter_test > 5) {
//				TIM3->CCR2 --;
//				TIM3->CCR1--;
//				counter_test = 0;
//			}
//			if (TIM3->CCR1 < 5) {
//				counter_test = 0;
//				TIM3->CCR2 = 0;
//				TIM3->CCR1 = 0;
//				flag_swtich = 0;
//			}
//		}
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void currentSearch(float cs1ADCValFilter, float cs2ADCValFilter) {
	cs1Voltage = cs1ADCValFilter * _CS1_PRESCAL;
	cs2Voltage = cs2ADCValFilter * _CS2_PRESCAL;

	cs1Current = (cs1Voltage - _CS1_VOLTAGE_MID_POINT) / _CS1_SENS;

	cs2Current = (cs2Voltage - _CS2_VOLTAGE_MID_POINT) / _CS2_SENS;

	motor_1.current = simpleKalman1(Median_Cur_CS1(cs1Current));

	motor_2.current = simpleKalman2(Median_Cur_CS2(cs2Current));
}

int Median_ADC_CS1(int newVal) {
	static float buffer[NUM_READ];  // ����������� �����
	static uint32_t count = 0;
	buffer[count] = newVal;
	if ((count < NUM_READ - 1) && (buffer[count] > buffer[count + 1])) {
		for (int i = count; i < NUM_READ - 1; i++) {
			if (buffer[i] > buffer[i + 1]) {
				float buff = buffer[i];
				buffer[i] = buffer[i + 1];
				buffer[i + 1] = buff;
			}
		}
	} else {
		if ((count > 0) && (buffer[count - 1] > buffer[count])) {
			for (int i = count; i > 0; i--) {
				if (buffer[i] < buffer[i - 1]) {
					float buff = buffer[i];
					buffer[i] = buffer[i - 1];
					buffer[i - 1] = buff;
				}
			}
		}
	}
	if (++count >= NUM_READ)
		count = 0;
	return buffer[(int) NUM_READ / 2];
}

int Median_ADC_CS2(int newVal) {
	static float buffer[NUM_READ];  // ����������� �����
	static uint32_t count = 0;
	buffer[count] = newVal;
	if ((count < NUM_READ - 1) && (buffer[count] > buffer[count + 1])) {
		for (int i = count; i < NUM_READ - 1; i++) {
			if (buffer[i] > buffer[i + 1]) {
				float buff = buffer[i];
				buffer[i] = buffer[i + 1];
				buffer[i + 1] = buff;
			}
		}
	} else {
		if ((count > 0) && (buffer[count - 1] > buffer[count])) {
			for (int i = count; i > 0; i--) {
				if (buffer[i] < buffer[i - 1]) {
					float buff = buffer[i];
					buffer[i] = buffer[i - 1];
					buffer[i - 1] = buff;
				}
			}
		}
	}
	if (++count >= NUM_READ)
		count = 0;
	return buffer[(int) NUM_READ / 2];
}

float Median_Cur_CS1(float newVal) {
	static float buffer[NUM_READ];  // ����������� �����
	static uint32_t count = 0;
	buffer[count] = newVal;
	if ((count < NUM_READ - 1) && (buffer[count] > buffer[count + 1])) {
		for (int i = count; i < NUM_READ - 1; i++) {
			if (buffer[i] > buffer[i + 1]) {
				float buff = buffer[i];
				buffer[i] = buffer[i + 1];
				buffer[i + 1] = buff;
			}
		}
	} else {
		if ((count > 0) && (buffer[count - 1] > buffer[count])) {
			for (int i = count; i > 0; i--) {
				if (buffer[i] < buffer[i - 1]) {
					float buff = buffer[i];
					buffer[i] = buffer[i - 1];
					buffer[i - 1] = buff;
				}
			}
		}
	}
	if (++count >= NUM_READ)
		count = 0;
	return buffer[(int) NUM_READ / 2];
}

float Median_Cur_CS2(float newVal) {
	static float buffer[NUM_READ];  // ����������� �����
	static uint32_t count = 0;
	buffer[count] = newVal;
	if ((count < NUM_READ - 1) && (buffer[count] > buffer[count + 1])) {
		for (int i = count; i < NUM_READ - 1; i++) {
			if (buffer[i] > buffer[i + 1]) {
				float buff = buffer[i];
				buffer[i] = buffer[i + 1];
				buffer[i + 1] = buff;
			}
		}
	} else {
		if ((count > 0) && (buffer[count - 1] > buffer[count])) {
			for (int i = count; i > 0; i--) {
				if (buffer[i] < buffer[i - 1]) {
					float buff = buffer[i];
					buffer[i] = buffer[i - 1];
					buffer[i - 1] = buff;
				}
			}
		}
	}
	if (++count >= NUM_READ)
		count = 0;
	return buffer[(int) NUM_READ / 2];
}

float _err_measure = 0.4;
float _q = 0.05;
float simpleKalman1(float newVal) {
	float _kalman_gain, _current_estimate;
	static float _err_estimate = 0.04;
	static float _last_estimate;
	_kalman_gain = (float) _err_estimate / (_err_estimate + _err_measure);
	_current_estimate = _last_estimate
			+ (float) _kalman_gain * (newVal - _last_estimate);
	_err_estimate = (1.0 - _kalman_gain) * _err_estimate
			+ fabs(_last_estimate - _current_estimate) * _q;
	_last_estimate = _current_estimate;
	return _current_estimate;
}

float simpleKalman2(float newVal) {
	float _kalman_gain, _current_estimate;
	static float _err_estimate = 0.04;
	static float _last_estimate;
	_kalman_gain = (float) _err_estimate / (_err_estimate + _err_measure);
	_current_estimate = _last_estimate
			+ (float) _kalman_gain * (newVal - _last_estimate);
	_err_estimate = (1.0 - _kalman_gain) * _err_estimate
			+ fabs(_last_estimate - _current_estimate) * _q;
	_last_estimate = _current_estimate;
	return _current_estimate;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
