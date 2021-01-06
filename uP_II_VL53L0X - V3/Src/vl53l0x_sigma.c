/**
 *******************************************************************************************************************************************************
  * @file    VL53L0X_SIGMA.c 
  * @author  Bou
  * @version V1.0.0
  * @date    XX-XXXX-20XX
  * @brief   This file provides...
  *******************************************************************************************************************************************************
 */
#include "stm32l4xx_hal.h"
#include "vl53l0x_sigma.h"

/********************************************************************************************************************************************************/
UART_HandleTypeDef huart2;
/********************************************************************************************************************************************************/
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MY_USART2_UART_Init(void)
{
	/* USER CODE BEGIN USART2_Init 0 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* Peripheral clock enable */
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART2 GPIO Configuration
	PA2     ------> USART2_TX
	PA3     ------> USART2_RX
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* USER CODE END USART2_Init 0 */
	/* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler("vl53l0x_sigma.h", 52);
  }
	/* USER CODE END USART2_Init 1 */
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void GPIO_LED_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Button */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Led */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(char file[20], int line)
{
	char uartCom[100];
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	if(line==0)
	{
		/* Toogle LED */
		for(uint8_t i=0;i<10;i++)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(50);
		}
	}
	sprintf(uartCom, "Error in line %d\n\r", line);
	HAL_UART_Transmit(&huart2, (uint8_t *)uartCom, strlen(uartCom), 100);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	/* USER CODE END Error_Handler_Debug */
}
/********************************************************************************************************************************************************/


/********************************************************************************************************************************************************/

