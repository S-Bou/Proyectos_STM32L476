/**
 *******************************************************************************************************************************************************
  * @file       mydelay.c 
  * @author  Bou
  * @version V1.0.0
  * @date    16-october-2020
  * @brief     This file provides...
  *******************************************************************************************************************************************************
 */
#include "stm32l4xx_hal.h"
#include "mydelay.h"
/********************************************************************************************************************************************************/

/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
static TIM_HandleTypeDef timer_handle;
 
// incluye algo que falta y es indispensable
 
void delayus (uint32_t microseconds)
{
    uint32_t overflows;
    uint32_t partial;
    uint32_t cnt_initial;
 
    overflows = microseconds/65536 + 1;
    //partial = microseconds - overflow * 65536;  
    partial = microseconds % 65536;
    cnt_initial = 65536 - partial;
 
   __HAL_TIM_CLEAR_FLAG(&timer_handle, TIM_FLAG_UPDATE);
__HAL_TIM_SET_COUNTER(&timer_handle,??????);
 
    counter_flags = 0;
    while (counter_flags < overflows)
    {
        while(comprobar flag == 0) {};  // __HAL_TIM_GET_FLAG(
        // ????borrar flag
        counter_flags++;
    }
 
    HAL_TIM_Base_Stop(
}
 

/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void delay_init(void)
{
    // poner en macha el RCC del relpoj
    timer_handle.Instance = TIM6;
    timer_handle.Init.Prescaler = ...
 
    HAL_TIM_Base_Init( );
 
}
/********************************************************************************************************************************************************/
