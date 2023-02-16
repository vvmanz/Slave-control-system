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
#include "tim.h"
#include "gpio.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_READ 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t tim1, tim2;
volatile uint16_t adcValBuf[2] = { 0, };
volatile uint8_t encValBuf1[1] = { 0, };
volatile uint8_t encValBuf2[1] = { 0, };

volatile uint8_t adcFlag;

volatile uint16_t adcValueCh1, adcValueCh2;
volatile float cs1ADCValFilter, cs2ADCValFilter;
volatile float cs1RealCur, cs2RealCur;

volatile uint16_t enc1Flag, enc2Flag;
volatile uint16_t enc1Value, enc2Value;

int tim3_1, tim3_2;

float reduce_counter;
float counter;

int pid_flag;
//Motor 3
float kp_angle_1 = 0.55611920073834;
float kp_speed_1 = 11.565570934863e-05;
float ki_speed_1 = 10.9857191588518e-05;
float kp_current_1 = 0.174697976230632;
float ki_current_1 = 13.9829080644131;

float kp_angle_2 = 0.55611920073834;
float kp_speed_2 = 11.565570934863e-05;
float ki_speed_2 = 9.9657191588518e-05;
float kp_current_2 = 0.174697976230632;
float ki_current_2 = 12.9829080644131;

volatile float out_angle_1, P_speed_1, I_speed_1, out_speed_1, P_current_1,
		I_current_1, out_current_1;
volatile float task_angle_1, task_angle_2;
volatile float err_angle_1, err_speed_1, err_current_1;
volatile float out_pwm_1;
volatile float out_angle_2, P_speed_2, I_speed_2, out_speed_2, P_current_2,
		I_current_2, out_current_2;
volatile float err_angle_2, err_speed_2, err_current_2;
volatile float out_pwm_2;

volatile int32_t enc1Val, resultEncValMot1, realVelMot1, pervTickMot1;
volatile int32_t enc2Val, resultEncValMot2, realVelMot2, pervTickMot2;

volatile float angle_1, angle_2;
volatile float task_1, task_2;

volatile float out_current_limited_2, out_current_limited_1;
volatile int counter_pid_2, counter_pid_1;

void pid_1(int task) {

	counter_pid_1++;
	if (counter_pid_1 >= 30) {
		err_angle_1 = task - angle_1;
		out_angle_1 = err_angle_1 * kp_angle_1;
		if (out_angle_1 > 12500) {
			out_angle_1 = 12500;
		}
		if (out_angle_1 < -12500) {
			out_angle_1 = -12500;
		}

		err_speed_1 = -out_angle_1 + realVelMot1;
		P_speed_1 = err_speed_1 * kp_speed_1;
		I_speed_1 += err_speed_1 * ki_speed_1 * 0.01;
		if (I_speed_1 >= 1.9) {
			I_speed_1 = 1.9;
		}
		if (I_speed_1 <= -1.9) {
			I_speed_1 = -1.9;
		}
		out_speed_1 = P_speed_1 + I_speed_1;

		if (out_speed_1 >= 1.9) {
			out_speed_1 = 1.9;
		}
		if (out_speed_1 <= -1.9) {
			out_speed_1 = -1.9;
		}
		counter_pid_1 = 0;
	}

	err_current_1 = out_speed_1 - cs1RealCur;
	P_current_1 = err_current_1 * kp_current_1;
	I_current_1 += err_current_1 * ki_current_1 * 0.000333;

	if (I_current_1 >= 1) {
		I_current_1 = 1;
	}
	if (I_current_1 <= -1) {
		I_current_1 = -1;
	}

	out_current_1 = P_current_1 + I_current_1;

	if (out_current_1 >= 0) {
		HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_RESET);
	}
	if (out_current_1 < 0) {
		HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_SET);
	}

	if (out_current_1 >= 1) {
		out_current_limited_1 = 1;
	}
	if (out_current_1 <= -1) {
		out_current_limited_1 = -1;
	}
	if (out_current_1 > -1 && out_current_1 < 1) {
		out_current_limited_1 = out_current_1;
	}
	out_pwm_1 = fabsf(out_current_limited_1 * 1000);

	if (out_pwm_1 == 0) {
		out_pwm_1 = 1;
	}
	if (out_pwm_1 == 1000) {
		out_pwm_1 = 999;
	}
	TIM3->CCR1 = out_pwm_1;
}

void pid_2(int task) {
	counter_pid_2++;
	if (counter_pid_2 >= 30) {
		err_angle_2 = task - angle_2;
		out_angle_2 = err_angle_2 * kp_angle_2;
		if (out_angle_2 > 12500) {
			out_angle_2 = 12500;
		}
		if (out_angle_2 < -12500) {
			out_angle_2 = -12500;
		}

		err_speed_2 = -out_angle_2 + realVelMot2;
		P_speed_2 = err_speed_2 * kp_speed_2;
		I_speed_2 += err_speed_2 * ki_speed_2 * 0.01;
		if (I_speed_2 >= 1.9) {
			I_speed_2 = 1.9;
		}
		if (I_speed_2 <= -1.9) {
			I_speed_2 = -1.9;
		}
		out_speed_2 = P_speed_2 + I_speed_2;

		if (out_speed_2 >= 1.9) {
			out_speed_2 = 1.9;
		}
		if (out_speed_2 <= -1.9) {
			out_speed_2 = -1.9;
		}
		counter_pid_2 = 0;
	}

	err_current_2 = out_speed_2 - cs2RealCur;
	P_current_2 = err_current_2 * kp_current_2;
	I_current_2 += err_current_2 * ki_current_2 * 0.000333;

	if (I_current_2 >= 1) {
		I_current_2 = 1;
	}
	if (I_current_2 <= -1) {
		I_current_2 = -1;
	}

	out_current_2 = P_current_2 + I_current_2;

	if (out_current_2 >= 0) {
		HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_RESET);
	}
	if (out_current_2 < 0) {
		HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_SET);
	}

	if (out_current_2 >= 1) {
		out_current_limited_2 = 1;
	}
	if (out_current_2 <= -1) {
		out_current_limited_2 = -1;
	}
	if (out_current_2 > -1 && out_current_2 < 1) {
		out_current_limited_2 = out_current_2;
	}
	out_pwm_2 = fabsf(out_current_limited_2 * 1000);

	if (out_pwm_2 == 0) {
		out_pwm_2 = 1;
	}
	if (out_pwm_2 == 1000) {
		out_pwm_2 = 999;
	}
	TIM3->CCR2 = out_pwm_2;
}

float cs1Voltage, cs2Voltage;
float cs1Current, cs2Current;

int iter_zero_1 = 0;
float sum_zero_1;
float sum_adc_zero_1;

int iter_zero_2 = 0;
float sum_zero_2;
float sum_adc_zero_2;

float cs1_mid_zero, cs2_mid_zero;
float cs1_adc_zero, cs2_adc_zero;

int zero_search_1 = 0;
int zero_search_2 = 2;
int direction;
int crit_counter;


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
float runMiddleArifmOptim(float newVal);
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
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcValBuf, 2);

	if (_ENABLE_MOTOR_1) {
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
		HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
		HAL_TIM_Base_Start_IT(&htim1);
		TIM3->CCR1 = 1;

		HAL_GPIO_WritePin(GPIOA, IN_1_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, IN_1_2_Pin, GPIO_PIN_RESET);
	}

	if (_ENABLE_MOTOR_2) {
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);

		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		HAL_TIM_Base_Start_IT(&htim2);
		TIM3->CCR2 = 1;

		HAL_GPIO_WritePin(GPIOB, IN_2_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, IN_2_2_Pin, GPIO_PIN_SET);
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
		TIM3->CCR2 = 500;

//		if (pid_flag == 1) {
//			pid_2(task_1);
//			pid_1(task_1);
//			pid_flag = 0;
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

	cs1RealCur = Median_Cur_CS1(cs1Current);
	cs1RealCur = simpleKalman1(cs1RealCur);
	cs2RealCur = Median_Cur_CS2(cs2Current);
	cs2RealCur = simpleKalman2(cs2RealCur);

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
float _q = 0.01;
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
