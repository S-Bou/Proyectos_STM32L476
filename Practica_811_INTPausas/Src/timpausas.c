/**
 *******************************************************************************************************************************************************
  * @file    timpausas.c 
  * @author  Bou
  * @version V1.0.0
  * @date    XX-XXXX-20XX
  * @brief   This file provides...
  *******************************************************************************************************************************************************
 */
#include "timpausas.h"
/********************************************************************************************************************************************************/
TIM_HandleTypeDef htim6;
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void TIM6_Init(void)
{
	HAL_NVIC_EnableIRQ(TIM6_IRQn);
	__HAL_RCC_TIM6_CLK_ENABLE();
	
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = (SystemCoreClock/20000) - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = (SystemCoreClock/10000) - 1;
  htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
}
/********************************************************************************************************************************************************/
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_PIN_5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param None
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
/********************************************************************************************************************************************************/

