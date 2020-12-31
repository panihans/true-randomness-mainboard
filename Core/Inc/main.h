/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define MOTORS_MIN_CCR 20000
#define MOTORS_MAX_CCR 65000
#define MOTORS_MAX_SPEED 100
#define MOTORS_CO ((MOTORS_MAX_CCR - MOTORS_MIN_CCR) / MOTORS_MAX_SPEED)


#define ESC_IDLE_CCR 4000
#define ESC_MIN_CCR 4200
#define ESC_MAX_CCR 7000
#define ESC_MAX_SPEED 100
#define ESC_CO ((ESC_MAX_CCR - ESC_MIN_CCR) / ESC_MAX_SPEED)

#define SERVO_IN_MIN_CCR 4700
#define SERVO_OUT_MIN_CCR 5000
#define SERVO_MAX_SPEED 100
#define SERVO_CO (400/SERVO_MAX_SPEED)

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

void Set_Servo_Speed(volatile uint32_t * channel_a, int32_t servo_speed, GPIO_PinState ir_status, int32_t ir_control);

void Set_Thrower_Speed(volatile uint32_t * channel_a, int32_t thrower_speed);

void Set_Motor_Speed(volatile uint32_t * channel_a, volatile uint32_t * channel_b, int32_t motor_speed);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
