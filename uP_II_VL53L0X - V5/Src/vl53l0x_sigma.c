/**
 *******************************************************************************************************************************************************
  * @file       VL53L0X_SIGMA.c 
  * @author  Bou
  * @version V1.0.0
  * @date     7-JAN-2021
  * @brief   This file provides all functions to manage the device VL53L0X TOF laser sensor
  *******************************************************************************************************************************************************
 */
#include "stm32l4xx_hal.h"
#include "vl53l0x_sigma.h"
/********************************************************************************************************************************************************/
TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart2;
/*Global ranging struct*/
VL53L0X_RangingMeasurementData_t RangingMeasurementData;
VL53L0X_Dev_t VL53L0XDevs={
	.Id=XNUCLEO53L0A1_DEV_CENTER,
	.DevLetter='c',
	.I2cHandle=&XNUCLEO53L0A1_hi2c,
	.I2cDevAddr=0x52
};

/*leaky factor for filtered range r(n) = averaged_r(n-1)*leaky +r(n)(1-leaky)*/
int LeakyFactorFix8 = (int)( 0.6 *256);
char uartBUFF[200];
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
/********************************************************************************************************************************************************/
/**
  * @brief  This function reset all sensor then do presence detection, all present devices are data initiated and assigned to their final I2C address
  * @param  None
  * @retval None
  */
void DetectSensors(uint32_t timeRefresh)
{
	int i=1;
	uint16_t Id;
	int status;
	int FinalAddress;

	/*nitialize vl53l1x communication parameters*/
  XNUCLEO53L0A1_Init();
	/*Message to init console*/
	sprintf(uartBUFF, "Hi I am Ranging VL53L0X mcu L476RG\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);			
	/*Message to init display*/
  XNUCLEO53L0A1_SetDisplayString("HOLA");
  HAL_Delay(500);
	/* Reset all */
	status = XNUCLEO53L0A1_ResetId(i, 0);
	/* detect all sensors (even on-board)*/
	VL53L0X_Dev_t *pDev;
	pDev = &VL53L0XDevs;
	pDev->I2cDevAddr = 0x52;
	pDev->Present = 0;
	status = XNUCLEO53L0A1_ResetId( pDev->Id, 1);
	HAL_Delay(2);
	FinalAddress=0x52+(i+1)*2;

	do {
		/* Set I2C standard mode (400 KHz) before doing the first register access */
		if (status == VL53L0X_ERROR_NONE) {status = VL53L0X_WrByte(pDev, 0x88, 0x00);}

		/* Try to read one register using default 0x52 address */
		status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if (status) {
			sprintf(uartBUFF, "#%d Read id fail\n\r", i);
			HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);
			break;
		}
		if (Id == 0xEEAA) {
			/* Sensor is found => Change its I2C address to final one */
			status = VL53L0X_SetDeviceAddress(pDev,FinalAddress);
			if (status != 0) {
				sprintf(uartBUFF, "#%d VL53L0X_SetDeviceAddress fail\n\r", i);
				HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);
				break;
			}
			
			pDev->I2cDevAddr = FinalAddress;
			/* Check all is OK with the new I2C address and initialize the sensor */
			status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
			if (status != 0) {
				sprintf(uartBUFF, "#%d VL53L0X_RdWord fail\n\r", i);
				HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);
				break;
			}

			status = VL53L0X_DataInit(pDev);
			if( status == 0 ){
				pDev->Present = 1;
			}else{
				Error_Handler("VL53L0X_SIGMA.C",79);
				break;
			}

			sprintf(uartBUFF, "VL53L0X %d Present and initiated to final 0x%x\n\r", pDev->Id, pDev->I2cDevAddr);
			HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);								
			pDev->Present = 1;
		}else {
			sprintf(uartBUFF, "#%d unknown ID %x\n\r", i, Id);
			HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);								
			status = 1;
		}
	} while (0);
	/* if fail r can't use for any reason then put the  device back to reset */
	if (status) {XNUCLEO53L0A1_ResetId(i, 0);}
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function store new ranging data into the device structure, apply leaky integrator if needed
  * @param  None
  * @retval None
  */
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev, VL53L0X_RangingMeasurementData_t *pRange)
{
	if( pRange->RangeStatus == 0 ){
		if( pDev->LeakyFirst ){
			pDev->LeakyFirst = 0;
			pDev->LeakyRange = pRange->RangeMilliMeter;
		}else{
			pDev->LeakyRange = (pDev->LeakyRange*LeakyFactorFix8 + (256-LeakyFactorFix8)*pRange->RangeMilliMeter)>>8;
		}
	}else{
		pDev->LeakyFirst = 1;
	}
}
/********************************************************************************************************************************************************/
/**  
  * @brief  TIM5 init function, this function is used to get time stamp for UART logging
  * @param  None
  * @retval None
  */
/* */
//void MX_TIM5_Init(void)
//{

//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;

//  htim5.Instance = TIM5;
//  htim5.Init.Prescaler = 83;
//  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim5.Init.Period = 0xFFFFFFFF;
//  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  HAL_TIM_OC_Init(&htim5);

//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

//  sConfigOC.OCMode = TIM_OCMODE_TIMING;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1);

//}
///********************************************************************************************************************************************************/
//void TimeStamp_Init(void)
//{
//    MX_TIM5_Init();
//	__TIM5_CLK_ENABLE();
//}
///********************************************************************************************************************************************************/
//void TimeStamp_Reset(void)
//{
//    HAL_TIM_Base_Start(&htim5);
//    htim5.Instance->CNT=0;
//}
///********************************************************************************************************************************************************/	
//uint32_t TimeStamp_Get(void)
//{
//    return htim5.Instance->CNT;
//}
/********************************************************************************************************************************************************/
/**
  * @brief  This function setup all detected sensors for single shot mode and setup ranging configuration
  * @param  None
  * @retval None
 */
void SetupSingleShot(RangingConfig rangingConfig)
{
	int status;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

	if( VL53L0XDevs.Present){
		status=VL53L0X_StaticInit(&VL53L0XDevs);
		if( status ){Error_Handler("vl53l0x_sigma.c",213);}

		status = VL53L0X_PerformRefCalibration(&VL53L0XDevs, &VhvSettings, &PhaseCal);
		if( status ){Error_Handler("vl53l0x_sigma.c",216);}

		status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs, &refSpadCount, &isApertureSpads);
		if( status ){Error_Handler("vl53l0x_sigma.c",219);}

		status = VL53L0X_SetDeviceMode(&VL53L0XDevs, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
		if( status ){Error_Handler("vl53l0x_sigma.c",222);}

		status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
		if( status ){Error_Handler("vl53l0x_sigma.c",225);}

		status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
		if( status ){Error_Handler("vl53l0x_sigma.c",228);}

		/* Ranging configuration LONG_RANGE*/
		signalLimit = (FixPoint1616_t)(0.1*65536);
		sigmaLimit = (FixPoint1616_t)(60*65536);
		timingBudget = 33000;
		preRangeVcselPeriod = 18;
		finalRangeVcselPeriod = 14;

		status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
		if( status ){Error_Handler("vl53l0x_sigma.c",238);}

		status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
		if( status ){Error_Handler("vl53l0x_sigma.c",241);}

		status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs,  timingBudget);
		if( status ){Error_Handler("vl53l0x_sigma.c",244);}

		status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
		if( status ){Error_Handler("vl53l0x_sigma.c",247);}

		status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
		if( status ){Error_Handler("vl53l0x_sigma.c",250);}

		status = VL53L0X_PerformRefCalibration(&VL53L0XDevs, &VhvSettings, &PhaseCal);
		if( status ){Error_Handler("vl53l0x_sigma.c",253);}

		VL53L0XDevs.LeakyFirst=1;
	}
}
/********************************************************************************************************************************************************/
/**
  * @brief  Implement the ranging demo with all modes managed through the blue button (short and long press)
  *            This function implements a while loop until the blue button is pressed
  * @param UseSensorsMask Mask of any sensors to use if not only one present
  * @param rangingConfig Ranging configuration to be used (same for all sensors)
  * @retval None
  */
void RangeDemo(int UseSensorsMask)
{
	RangingConfig rangingConfig = LONG_RANGE;
	int status;
	char StrDisplay[5];

	/* Setup all sensors in Single Shot mode */
	SetupSingleShot(rangingConfig);

	/* only one sensor, call All-In-One blocking API function */
	status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDevs,&RangingMeasurementData);
	if( status==0 ){ /* Push data logging to UART */
		if(RangingMeasurementData.RangeMilliMeter>2200){
			sprintf(uartBUFF, "Device: %d; Out of range\n\r", VL53L0XDevs.Id);
			HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);	
		}else{
			sprintf(uartBUFF, "Device: %d; Distance: %d mm\n\r", VL53L0XDevs.Id, RangingMeasurementData.RangeMilliMeter);
			HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);	
    }
		Sensor_SetNewRange(&VL53L0XDevs,&RangingMeasurementData);
		/* Display distance in cm */
		if( RangingMeasurementData.RangeStatus == 0 ){
			sprintf(StrDisplay, "%3dc",(int)VL53L0XDevs.LeakyRange/10);	
		}else{
			sprintf(StrDisplay, " OUT");
			StrDisplay[0]=VL53L0XDevs.DevLetter;
		}
	}else{
	Error_Handler("vl53l0x_sigma.c",290);
	}
	XNUCLEO53L0A1_SetDisplayString(StrDisplay);
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: the file where is this function.
  * @param  line: the line where is this function
  * @retval None
  * Use:   Error_Handler("vl53l0x_sigma.h", 89);   or   Error_Handler("NULL", 0);
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
	sprintf(uartCom, "Error in %s line %d\n\r", file,line);
	HAL_UART_Transmit(&huart2, (uint8_t *)uartCom, strlen(uartCom), 100);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	/* USER CODE END Error_Handler_Debug */
}
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


/********************************************************************************************************************************************************/

