/**
 ********************************************************************************************************************************************************
 * @file       VL53L0X_SIGMA.h
 * @brief     VL53L0X_SIGMA header file
 * @version V1.0.0
 * @date     7-JAN-2021
 ********************************************************************************************************************************************************
 */
#ifndef  __VL53L0X_SIGMA_H
#define __VL53L0X_SIGMA_H

#ifdef __cplusplus
extern "C" {
#endif
 /***************************************************************************_includes_*****************************************************************/
#include "X-NUCLEO-53L0A1.h"
#include "vl53l0x_api.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
/***************************************************************************_Global_variables_*********************************************************/
typedef enum {
	LONG_RANGE 	   	 = 0, /*!< Long range mode */
	HIGH_SPEED 		   = 1, /*!< High speed mode */
	HIGH_ACCURACY = 2, /*!< High accuracy=precision mode */
} RangingConfig;
/***************************************************************************_Functions_***************************************************************/
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange);
void SetupSingleShot(RangingConfig rangingConfig);
void RangeDemo(RangingConfig Rangingconfig);
void SoundBack(uint8_t caso, uint16_t time);
void EXTI15_10_IRQHandler_Config(void);
void Error_Handler(char file[20], int line);
void MY_USART2_UART_Init(void); 
void GPIO_LED_Config(void);
void DetectSensor(void);
void TIM6_Config(uint32_t period);     
/********************************************************************************************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __VL53L0X_SIGMA_H */

