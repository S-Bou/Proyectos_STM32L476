/**
 *******************************************************************************************************************************************************
  * @file    examen.c 
  * @author  Bou
  * @version V1.0.0
  * @date    XX-XXXX-20XX
  * @brief   This file provides...
  ***************************************************************************_ncludes_*****************************************************************
 */
#include "stm32l4xx_hal.h"
#include "examen.h"
/***************************************************************************_Global_variables_*******************************************************/

/***************************************************************************_Functions_***************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void GPIO_LED_Config(void)
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
  * @param  None
  * @retval None
  */
void GPIO_BUTTON_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
	
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/********************************************************************************************************************************************************/

