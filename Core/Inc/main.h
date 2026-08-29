/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"
#include <stdio.h>

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
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef *STEP_Port;
  uint16_t STEP_Pin;

  GPIO_TypeDef *DIR_Port;
  uint16_t DIR_Pin;

  double current_stroke_length_mm; // Current actuator stroke length in mm.

} StepperMotor;

void Stepper_Move_Select(StepperMotor *motor, GPIO_PinState direction,
                         uint32_t pulses);     
void delay_us(uint16_t us);
void Stepper_Move3(StepperMotor *motor1, GPIO_PinState dir1, uint32_t steps1,
                   StepperMotor *motor2, GPIO_PinState dir2, uint32_t steps2,
                   StepperMotor *motor3, GPIO_PinState dir3, uint32_t steps3);
void simscape_ik(double Z, double roll, double pitch, double q[3]);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_BUTTON_Pin GPIO_PIN_3
#define START_BUTTON_GPIO_Port GPIOF
#define STEP_OUT_ACT3_Pin GPIO_PIN_9
#define STEP_OUT_ACT3_GPIO_Port GPIOE
#define DIR_OUT_ACT2_Pin GPIO_PIN_11
#define DIR_OUT_ACT2_GPIO_Port GPIOE
#define DIR_OUT_ACT1_Pin GPIO_PIN_13
#define DIR_OUT_ACT1_GPIO_Port GPIOE
#define STEP_OUT_ACT2_Pin GPIO_PIN_14
#define STEP_OUT_ACT2_GPIO_Port GPIOE
#define DIR_OUT_ACT3_Pin GPIO_PIN_12
#define DIR_OUT_ACT3_GPIO_Port GPIOG
#define STEP_OUT_ACT1_Pin GPIO_PIN_14
#define STEP_OUT_ACT1_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
