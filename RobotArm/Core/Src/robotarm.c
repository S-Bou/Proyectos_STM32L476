/**
 *******************************************************************************************************************************************************
  * @file    robotarm.c 
  * @author  Bou
  * @version V1.0.0
  * @date    XX-XXXX-20XX
  * @brief   This file provides...
  *******************************************************************************************************************************************************
 */
#include "robotarm.h"
/********************************************************************************************************************************************************/
extern TIM_HandleTypeDef htim8;
extern ADC_HandleTypeDef hadc1;
uint32_t adc_val[4];
uint32_t pos_val[4];
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_ADC_Start_DMA(&hadc1, adc_val, 4);
	HandlerServosPot();
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void HandlerServosPot(void)
{	
	for(uint8_t i=0; i<4; i++){
				pos_val[i] = (adc_val[i]/55)+MIN_DEGREE;
	}
		if ( pos_val[3] < MIN_DEGREE_GRIPPER ){
			pos_val[3] = MIN_DEGREE_GRIPPER ;
		}else if ( pos_val[3] > MAX_DEGREE_GRIPPER ){
			pos_val[3] = MAX_DEGREE_GRIPPER ;
		}
	
	htim8.Instance->CCR1 = pos_val[0];
	htim8.Instance->CCR2 = pos_val[1];
	htim8.Instance->CCR3 = pos_val[2];
	htim8.Instance->CCR4 = pos_val[3];
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param None
  * @retval None
  */
void MoveServoGripper(uint8_t min, uint8_t max)
{
//		htim2.Instance->CCR2 = min;

//		htim2.Instance->CCR2 = max;

}
/********************************************************************************************************************************************************/

