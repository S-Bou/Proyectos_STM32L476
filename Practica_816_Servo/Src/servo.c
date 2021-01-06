/**
    @file servo.c
    @brief Servo handling using bl, bla
    
    IMPORTANT: Used resources
      TIM2
      PA5     ------> TIM2_CH1 
*/

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "error.h"
#include "servo.h"


// We use variables here for calibratiuon pourposes
static int16_t TENTH_DEGREES_MIN      = -900;
static int16_t TIMMING_MIN            = 1000;
static int16_t TENTH_DEGREES_MAX      = 900;
static int16_t TIMMING_MAX            = 2000;
static int16_t TIMMING_TOTAL          = 20000;

static TIM_HandleTypeDef htim;

/*************************************************************************************************/
/**
  @brief Set-ups the underlaying hardware, bla, bla ...
  @param none
  Example:
  @verbatim
    servo_Init();
  @endverbatim
 */

error_t servo_Init(void) 
{

   /* GPIO Ports Clock Enable */
   __HAL_RCC_GPIOA_CLK_ENABLE();
   
   /* Peripheral clock enable */
   __HAL_RCC_TIM2_CLK_ENABLE();
   

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim.Instance = TIM2;
    htim.Init.Prescaler = (SystemCoreClock/1000000) +1;   // 1 uS
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = TIMMING_TOTAL;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim) != HAL_OK)
    {
        return Error_ServoInit;
    }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK)
  {
      return Error_ServoInit;
  }

  if (HAL_TIM_PWM_Init(&htim) != HAL_OK)
  {
      return Error_ServoInit;
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
  {
      return Error_ServoInit;
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (TIMMING_MAX+TIMMING_MIN)/2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
      return Error_ServoInit;
  }

  //HAL_TIM_MspPostInit(&htim2);
  
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // this functions does not retuns anything
  
  HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_1);
  
  return Error_NoError;
}

/*************************************************************************************************/
/**
  @brief Sets the position of the servo
  @param tenth_degree tenth of dreeg for the position
  Example:
  @verbatim
    servo_SetPosition(456); // the position will be 45.6 degrees
  @endverbatim
 */
error_t servo_SetPosition(int16_t tenth_degree) 
{
    // pa currar hoy
    uint32_t capture_value;
   
    capture_value = (((int32_t)tenth_degree - TENTH_DEGREES_MIN) * (TIMMING_MAX - TIMMING_MIN))/(TENTH_DEGREES_MAX - TENTH_DEGREES_MIN) + TIMMING_MIN; 
    htim.Instance->CCR1 = capture_value;
   
    return Error_NoError;   
}

/*** END OF FILE *********************************************************************************/
