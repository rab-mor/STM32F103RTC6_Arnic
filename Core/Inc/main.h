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
#include "stm32f1xx_hal.h"

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
#define RLY4_B_Pin GPIO_PIN_13
#define RLY4_B_GPIO_Port GPIOC
#define RLY3_A_Pin GPIO_PIN_14
#define RLY3_A_GPIO_Port GPIOC
#define RLY3_B_Pin GPIO_PIN_15
#define RLY3_B_GPIO_Port GPIOC
#define Xtal_1_Pin GPIO_PIN_0
#define Xtal_1_GPIO_Port GPIOD
#define Xtal_2_Pin GPIO_PIN_1
#define Xtal_2_GPIO_Port GPIOD
#define RLY2_A_Pin GPIO_PIN_0
#define RLY2_A_GPIO_Port GPIOC
#define RLY2_B_Pin GPIO_PIN_1
#define RLY2_B_GPIO_Port GPIOC
#define RLY1_A_Pin GPIO_PIN_2
#define RLY1_A_GPIO_Port GPIOC
#define RLY1_B_Pin GPIO_PIN_3
#define RLY1_B_GPIO_Port GPIOC
#define ADC2_CurrentSensor8_Pin GPIO_PIN_0
#define ADC2_CurrentSensor8_GPIO_Port GPIOA
#define ADC2_CurrentSensor7_Pin GPIO_PIN_1
#define ADC2_CurrentSensor7_GPIO_Port GPIOA
#define ADC2_CurrentSensor6_Pin GPIO_PIN_2
#define ADC2_CurrentSensor6_GPIO_Port GPIOA
#define ADC2_CurrentSensor5_Pin GPIO_PIN_3
#define ADC2_CurrentSensor5_GPIO_Port GPIOA
#define ADC1_CurrentSensor4_Pin GPIO_PIN_4
#define ADC1_CurrentSensor4_GPIO_Port GPIOA
#define ADC1_CurrentSensor3_Pin GPIO_PIN_5
#define ADC1_CurrentSensor3_GPIO_Port GPIOA
#define ADC1_CurrentSensor2_Pin GPIO_PIN_6
#define ADC1_CurrentSensor2_GPIO_Port GPIOA
#define ADC1_CurrentSensor1_Pin GPIO_PIN_7
#define ADC1_CurrentSensor1_GPIO_Port GPIOA
#define ADC1_24V_Pin GPIO_PIN_4
#define ADC1_24V_GPIO_Port GPIOC
#define ADC2_5V_Pin GPIO_PIN_5
#define ADC2_5V_GPIO_Port GPIOC
#define WDOG_Pin GPIO_PIN_2
#define WDOG_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_10
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_11
#define EEPROM_SDA_GPIO_Port GPIOB
#define F_CS_Pin GPIO_PIN_12
#define F_CS_GPIO_Port GPIOB
#define F_SCK_Pin GPIO_PIN_13
#define F_SCK_GPIO_Port GPIOB
#define F_MISO_Pin GPIO_PIN_14
#define F_MISO_GPIO_Port GPIOB
#define F_MOSI_Pin GPIO_PIN_15
#define F_MOSI_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define RLY5_B_Pin GPIO_PIN_15
#define RLY5_B_GPIO_Port GPIOA
#define RLY5_A_Pin GPIO_PIN_2
#define RLY5_A_GPIO_Port GPIOD
#define RLY6_B_Pin GPIO_PIN_3
#define RLY6_B_GPIO_Port GPIOB
#define RLY6_A_Pin GPIO_PIN_4
#define RLY6_A_GPIO_Port GPIOB
#define RLY7_B_Pin GPIO_PIN_5
#define RLY7_B_GPIO_Port GPIOB
#define RLY7_A_Pin GPIO_PIN_6
#define RLY7_A_GPIO_Port GPIOB
#define RLY8_B_Pin GPIO_PIN_7
#define RLY8_B_GPIO_Port GPIOB
#define RLY8_A_Pin GPIO_PIN_8
#define RLY8_A_GPIO_Port GPIOB
#define RLY4_A_Pin GPIO_PIN_9
#define RLY4_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
