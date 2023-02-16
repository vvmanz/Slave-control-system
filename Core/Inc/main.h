/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern volatile uint16_t adcValueCh1, adcValueCh2;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN_1_1_Pin GPIO_PIN_4
#define IN_1_1_GPIO_Port GPIOA
#define IN_1_2_Pin GPIO_PIN_5
#define IN_1_2_GPIO_Port GPIOA
#define IN_2_1_Pin GPIO_PIN_0
#define IN_2_1_GPIO_Port GPIOB
#define IN_2_2_Pin GPIO_PIN_1
#define IN_2_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define IN_1_1_Pin GPIO_PIN_4
#define IN_1_1_GPIO_Port GPIOA
#define IN_1_2_Pin GPIO_PIN_5
#define IN_1_2_GPIO_Port GPIOA
#define IN_2_1_Pin GPIO_PIN_0
#define IN_2_1_GPIO_Port GPIOB
#define IN_2_2_Pin GPIO_PIN_1
#define IN_2_2_GPIO_Port GPIOB

#define _ENABLE_MOTOR_1 1
#define _ENABLE_MOTOR_2 1

#define _CS1_ADC_MID_POINT 1965
#define _CS1_CPU_POWER 3.3
#define _CS1_QUANT_LEVEL 4096.0
#define _CS1_PRESCAL (_CS1_CPU_POWER / _CS1_QUANT_LEVEL)
#define _CS1_VOLTAGE_MID_POINT 1.58
#define _CS1_SENS 0.08

#define _CS2_ADC_MID_POINT 2010
#define _CS2_CPU_POWER 3.3
#define _CS2_QUANT_LEVEL 4096.0
#define _CS2_PRESCAL (_CS2_CPU_POWER / _CS2_QUANT_LEVEL)
#define _CS2_VOLTAGE_MID_POINT 1.612
#define _CS2_SENS 0.09
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
