/**
 *******************************************************************************************************************************************************
  * @file       CODE.c 
  * @author  Bou
  * @version V1.0.0
  * @date    09-October-20xx
  * @brief     This file provides configuration of external interrupts 
	*               and handles value of a variable.
  *******************************************************************************************************************************************************
 */
#include "stm32l4xx_hal.h"
#include "cars.h"
/********************************************************************************************************************************************************/
static volatile uint32_t  cars_axis_count;
/********************************************************************************************************************************************************/
/**
  * @brief    This function config external interrupt in port C pin 13, 
	*               and reset variable cars_axis_count.
  * @param  None
  * @retval   None
  */
void cars_Init ( void )
{
	cars_axis_count = 0;
	
		GPIO_InitTypeDef port;
	// Step 1: disable interrupt
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	// Step 2: configure target device
		// Enable GPIOx clock
		__HAL_RCC_GPIOC_CLK_ENABLE();
		// Configure PA0 pin as input floating
		port.Pin    = GPIO_PIN_13;
		port.Mode = GPIO_MODE_IT_FALLING;
		port.Pull   = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &port);
	// Step 3: configure NVIC related interrupt
		HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
	// Step 4: enable interrupt
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
/********************************************************************************************************************************************************/
/**
  * @brief   This function return value of cars_axis_count.
  * @param  None
  * @retval   cars_axis_count
  */
uint32_t cars_GetCount ( void )
{
	return cars_axis_count;
}
/********************************************************************************************************************************************************/
/**
  * @brief   This function is called when occurs an interrupt.
  * @param  None
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	cars_IncrementCount();
}
/********************************************************************************************************************************************************/
/**
  * @brief   This function increase variable.
  * @param  None
  * @retval None
  */
void cars_IncrementCount ( void )
{
	cars_axis_count = cars_axis_count + 1;
}
/********************************************************************************************************************************************************/
