/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define Brake_Switch_Pin GPIO_PIN_1
#define Brake_Switch_GPIO_Port GPIOC
#define Steering_Adc_Pin GPIO_PIN_2
#define Steering_Adc_GPIO_Port GPIOC
#define Reverse_Switch_Pin GPIO_PIN_1
#define Reverse_Switch_GPIO_Port GPIOA
#define Trq_Spd_Btn_Pin GPIO_PIN_2
#define Trq_Spd_Btn_GPIO_Port GPIOA
#define Fake_Brake_Pin GPIO_PIN_3
#define Fake_Brake_GPIO_Port GPIOA
#define Ignition_Button_2_Pin GPIO_PIN_5
#define Ignition_Button_2_GPIO_Port GPIOA
#define Reset_Vel_Btn_Pin GPIO_PIN_6
#define Reset_Vel_Btn_GPIO_Port GPIOA
#define Decrease_Vel_Btn_Pin GPIO_PIN_7
#define Decrease_Vel_Btn_GPIO_Port GPIOA
#define Increase_Vel_Btn_Pin GPIO_PIN_4
#define Increase_Vel_Btn_GPIO_Port GPIOC
#define System_On_Off_Pin GPIO_PIN_10
#define System_On_Off_GPIO_Port GPIOB
#define Ignition_Button_Pin GPIO_PIN_14
#define Ignition_Button_GPIO_Port GPIOB
#define Beacon_Trigger_Pin GPIO_PIN_8
#define Beacon_Trigger_GPIO_Port GPIOA
#define RPM_Sensor_R_Pin GPIO_PIN_4
#define RPM_Sensor_R_GPIO_Port GPIOB
#define RPM_Sensor_R_EXTI_IRQn EXTI4_IRQn
#define RPM_Sensor_L_Pin GPIO_PIN_5
#define RPM_Sensor_L_GPIO_Port GPIOB
#define RPM_Sensor_L_EXTI_IRQn EXTI9_5_IRQn
#define CAN1_STDBY_Pin GPIO_PIN_6
#define CAN1_STDBY_GPIO_Port GPIOB
#define CAN2_STDBY_Pin GPIO_PIN_7
#define CAN2_STDBY_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
