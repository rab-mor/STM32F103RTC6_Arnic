/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RLY4_B_Pin|RLY3_A_Pin|RLY3_B_Pin|RLY2_A_Pin
                          |RLY2_B_Pin|RLY1_A_Pin|RLY1_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, WDOG_Pin|RLY6_B_Pin|RLY6_A_Pin|RLY7_B_Pin
                          |RLY7_A_Pin|RLY8_B_Pin|RLY8_A_Pin|RLY4_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RLY5_B_GPIO_Port, RLY5_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RLY5_A_GPIO_Port, RLY5_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RLY4_B_Pin RLY3_A_Pin RLY3_B_Pin RLY2_A_Pin
                           RLY2_B_Pin RLY1_A_Pin RLY1_B_Pin */
  GPIO_InitStruct.Pin = RLY4_B_Pin|RLY3_A_Pin|RLY3_B_Pin|RLY2_A_Pin
                          |RLY2_B_Pin|RLY1_A_Pin|RLY1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : WDOG_Pin RLY6_B_Pin RLY6_A_Pin RLY7_B_Pin
                           RLY7_A_Pin RLY8_B_Pin RLY8_A_Pin RLY4_A_Pin */
  GPIO_InitStruct.Pin = WDOG_Pin|RLY6_B_Pin|RLY6_A_Pin|RLY7_B_Pin
                          |RLY7_A_Pin|RLY8_B_Pin|RLY8_A_Pin|RLY4_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RLY5_B_Pin */
  GPIO_InitStruct.Pin = RLY5_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RLY5_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RLY5_A_Pin */
  GPIO_InitStruct.Pin = RLY5_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RLY5_A_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
