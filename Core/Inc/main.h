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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM0_Pin GPIO_PIN_0
#define PWM0_GPIO_Port GPIOC
#define PWM45_Pin GPIO_PIN_1
#define PWM45_GPIO_Port GPIOC
#define PWM90_Pin GPIO_PIN_2
#define PWM90_GPIO_Port GPIOC
#define PWM135_Pin GPIO_PIN_3
#define PWM135_GPIO_Port GPIOC
#define PWM180_Pin GPIO_PIN_4
#define PWM180_GPIO_Port GPIOC
#define PWM225_Pin GPIO_PIN_6
#define PWM225_GPIO_Port GPIOC
#define PWM270_Pin GPIO_PIN_7
#define PWM270_GPIO_Port GPIOC
#define PWM315_Pin GPIO_PIN_8
#define PWM315_GPIO_Port GPIOC
#define BIAS_Pin GPIO_PIN_9
#define BIAS_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ADC_TRIG_OUT_Pin GPIO_PIN_10
#define ADC_TRIG_OUT_GPIO_Port GPIOC
#define PERIOD_Pin GPIO_PIN_11
#define PERIOD_GPIO_Port GPIOC
#define TIMEBASE_Pin GPIO_PIN_12
#define TIMEBASE_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
