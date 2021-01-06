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
/********************************************************************************************************************************************************/
#include "X-NUCLEO-53L0A1.h"
#include "vl53l0x_api.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
/********************************************************************************************************************************************************/

/********************************************************************************************************************************************************/
void DetectSensors(void);
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange);
void TimeStamp_Init(void);
void TimeStamp_Reset(void);
uint32_t TimeStamp_Get(void);
void SetupSingleShot(void);
void RangeDemo(void);
void Error_Handler(char file[20], int line);
void EXTI15_10_IRQHandler_Config(void);
void MY_TIM2_Init(void);
void MY_USART2_UART_Init(void);
void GPIO_LED_Config(void);
/********************************************************************************************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __VL53L0X_SIGMA_H */
