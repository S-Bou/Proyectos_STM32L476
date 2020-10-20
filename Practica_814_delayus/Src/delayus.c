/**
 *******************************************************************************************************************************************************
  * @file       delayus.c 
  * @author  Bou
  * @version V1.0.0
  * @date    16-october-2020
  * @brief     This file provides...
  *******************************************************************************************************************************************************
 */
#include "delayus.h"
/********************************************************************************************************************************************************/
static TIM_HandleTypeDef timer_handle;
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  Number of microseconds
  * @retval None
  */
void delay_us(uint32_t microseconds)
{
		uint32_t overflows;
		uint32_t partial;
		uint32_t cnt_initial;

		overflows = microseconds/65536 + 1; 
		partial = microseconds % 65536;
		cnt_initial = 65536 - partial;

		__HAL_TIM_CLEAR_FLAG(&timer_handle, TIM_FLAG_UPDATE);
		__HAL_TIM_SET_COUNTER(&timer_handle, cnt_initial);

		HAL_TIM_Base_Start(&timer_handle);
	
		uint32_t counter_flags = 0;
		while (counter_flags < overflows)
		{
				while(__HAL_TIM_GET_FLAG(&timer_handle, TIM_FLAG_UPDATE) == 0) {}; 
				__HAL_TIM_CLEAR_FLAG(&timer_handle, TIM_FLAG_UPDATE);
				counter_flags++;
		}

		HAL_TIM_Base_Stop(&timer_handle);
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void delay_init(void)
{
		__HAL_RCC_TIM6_CLK_ENABLE();
	
    timer_handle.Instance = TIM6;
		timer_handle.Init.Prescaler = (SystemCoreClock/1000000) +1;
		timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
		timer_handle.Init.Period = 65535;
		timer_handle.Init.ClockDivision = 0;
		timer_handle.Init.RepetitionCounter = 0;
 
    HAL_TIM_Base_Init(&timer_handle);
 
}
/********************************************************************************************************************************************************/
