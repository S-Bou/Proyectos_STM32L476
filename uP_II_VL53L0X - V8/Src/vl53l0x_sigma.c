/*********************************************************************************************************************************************************
  * @file       VL53L0X_SIGMA.c 
  * @author  Bou
  * @version V1.0.0
  * @date     7-JAN-2021
  * @brief     This file provides all functions to manage the device VL53L0X TOF laser sensor
  */
 /***************************************************************************_includes_*****************************************************************/

#include "stm32l4xx_hal.h"
#include "vl53l0x_sigma.h"
/***************************************************************************_Global_variables_*********************************************************/
TIM_HandleTypeDef htim6;
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
uint8_t counter=0;
uint32_t time=100;
/***************************************************************************_Functions_***************************************************************/
/**
  * @brief    This function reset all sensor then do presence detection, all present devices are data initiated and assigned to their final I2C address
  * @param  None
  * @retval   None
  */
void DetectSensor(void)
{
	uint16_t Id;
	int status;
	int FinalAddress;

	/*Initialize all configured peripherals */
	MY_USART2_UART_Init();
	/*Congig pin as output to led*/
	GPIO_LED_Config();
	/*Config timer 6 signal*/
  //TIM6_Config();
  /*Config EXTI by whes is pressed the user button*/
	EXTI15_10_IRQHandler_Config();
	/*nitialize vl53l1x communication parameters*/	
  XNUCLEO53L0A1_Init();
	/*Message to init console*/
	sprintf(uartBUFF, "Hi I am Ranging VL53L0X mcu L476RG\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);			
	/*Message to init display*/
  XNUCLEO53L0A1_SetDisplayString("HOLA");
  HAL_Delay(500);
	/* Reset all */
	status = XNUCLEO53L0A1_ResetId(1, 0);
	/* detect sensor (even on-board)*/
	VL53L0X_Dev_t *pDev;
	pDev = &VL53L0XDevs;
	pDev->I2cDevAddr = 0x52;
	pDev->Present = 0;
	status = XNUCLEO53L0A1_ResetId( pDev->Id, 1);
	HAL_Delay(2);
	FinalAddress=0x56;

	do {
		/* Set I2C standard mode (400 KHz) before doing the first register access */
		if (status == VL53L0X_ERROR_NONE) {status = VL53L0X_WrByte(pDev, 0x88, 0x00);}

		/* Try to read one register using default 0x52 address */
		status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if (status) {
			sprintf(uartBUFF, "# Read id fail\n\r");
			HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);
			break;
		}
		if (Id == 0xEEAA) {
			/* Sensor is found => Change its I2C address to final one */
			status = VL53L0X_SetDeviceAddress(pDev,FinalAddress);
			if (status != 0) {
				sprintf(uartBUFF, "# VL53L0X_SetDeviceAddress fail\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);
				break;
			}
			pDev->I2cDevAddr = FinalAddress;
			/* Check all is OK with the new I2C address and initialize the sensor */
			status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
			if (status != 0) {
				sprintf(uartBUFF, "# VL53L0X_RdWord fail\n\r");
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
			sprintf(uartBUFF, "# unknown ID %x\n\r",Id);
			HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);								
			status = 1;
		}
	} while (0);
	/* if fail r can't use for any reason then put the  device back to reset */
	if (status) {XNUCLEO53L0A1_ResetId(1, 0);}
}
/********************************************************************************************************************************************************/
/**
  * @brief     This function store new ranging data into the device structure, apply leaky integrator if needed
  * @param  pDev: pointer to the device params
  * @param  pRange: pointer to the range status, that give the quality of the latest ranging.
  * @retval   None
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
  * @brief    This function setup all detected sensors for single shot mode and setup ranging configuration.
* @param   rangingConfig: ranging configuration mode to be used 
  * @retval   None
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
		if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",1);}

		status = VL53L0X_PerformRefCalibration(&VL53L0XDevs, &VhvSettings, &PhaseCal);
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",2);}

		status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs, &refSpadCount, &isApertureSpads);
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",3);}

		status = VL53L0X_SetDeviceMode(&VL53L0XDevs, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",4);}

		status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",5);}

		status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",5);}

		/* Ranging configuration */
		switch(rangingConfig) {
			case LONG_RANGE:
				signalLimit = (FixPoint1616_t)(0.1*65536);
				sigmaLimit = (FixPoint1616_t)(60*65536);
				timingBudget = 33000;
				preRangeVcselPeriod = 18;
				finalRangeVcselPeriod = 14;
				break;
			case HIGH_ACCURACY:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(18*65536);
				timingBudget = 200000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
			case HIGH_SPEED:
				signalLimit = (FixPoint1616_t)(0.25*65536);
				sigmaLimit = (FixPoint1616_t)(32*65536);
				timingBudget = 20000;
				preRangeVcselPeriod = 14;
				finalRangeVcselPeriod = 10;
				break;
			default:
				Error_Handler("vl53l0x_sigma.c ->SetupSingleShot",12);
		}

		status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
    if( status ){Error_Handler("vl53l0x_sigma.c ->SetupSingleShot",6);}

		status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",7);}

		status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs,  timingBudget);
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",8);}

		status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",9);}

		status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",10);}

		status = VL53L0X_PerformRefCalibration(&VL53L0XDevs, &VhvSettings, &PhaseCal);
    if( status ){Error_Handler("vl53l0x_sigma.c -> SetupSingleShot",11);}

		VL53L0XDevs.LeakyFirst=1;
	}
}
/********************************************************************************************************************************************************/
/**
  * @brief    Implement the ranging demo with LONG_RANGE mode.
* @param   Rangingconfig: mode to pass in function SetupSingleShot()
  * @retval   None
  */
void RangeDemo(RangingConfig Rangingconfig)
{
	int status;
	char StrDisplay[5];

	/* Setup sensor in Single Shot mode */
	SetupSingleShot(Rangingconfig);	
	/* only one sensor, call All-In-One blocking API function */
	status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDevs,&RangingMeasurementData);
	if( status ){Error_Handler("vl53l0x_sigma.c -> RangeDemo",1);}
	switch (counter){
		case 0:
			sprintf(uartBUFF, "Wait to press button..\n\r");
			HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);				
			sprintf(StrDisplay, "STOP");
		  XNUCLEO53L0A1_SetDisplayString(StrDisplay);		
			break;		
		case 1:
			if( status==0 ){ /* Push data logging to UART */
				if(RangingMeasurementData.RangeMilliMeter>2200){
					sprintf(uartBUFF, "Device: %d; Out of range\n\r", VL53L0XDevs.Id);
					HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);	
				}else{
					sprintf(uartBUFF, "Device: %d; Mode: measuring; Distance: %d mm\n\r", VL53L0XDevs.Id, RangingMeasurementData.RangeMilliMeter);
					HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);	
				}
				Sensor_SetNewRange(&VL53L0XDevs,&RangingMeasurementData);
				/* Display distance in cm */
				if( RangingMeasurementData.RangeStatus == 0 ){
					sprintf(StrDisplay, "%3dc",(int)VL53L0XDevs.LeakyRange/10);	
				}else{
					sprintf(StrDisplay, " OUT");
				}
			}else{
			Error_Handler("vl53l0x_sigma.c -> RangeDemo",1);
			}
			XNUCLEO53L0A1_SetDisplayString(StrDisplay);
			HAL_Delay(time);
			break;
		case 2:
			if( status==0 ){ /* Push data logging to UART */
			if(RangingMeasurementData.RangeMilliMeter<100){
				sprintf(uartBUFF, "Device: Toogle output\n\r");
				HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);	
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
					sprintf(StrDisplay, " ON ");
				}else{
					sprintf(StrDisplay, " OFF");
				}
			}else{
				if(RangingMeasurementData.RangeMilliMeter>2200){
					sprintf(uartBUFF, "Device: %d; Out of range\n\r", VL53L0XDevs.Id);
					HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);	
				}else{
					sprintf(uartBUFF, "Device: %d; Mode: switch; Distance: %d mm\n\r", VL53L0XDevs.Id, RangingMeasurementData.RangeMilliMeter);
					HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);	
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
						sprintf(StrDisplay, " ON ");
					}else{
						sprintf(StrDisplay, " OFF");
					}
				}				
			}
			Sensor_SetNewRange(&VL53L0XDevs,&RangingMeasurementData);
			}else{
			Error_Handler("vl53l0x_sigma.c -> RangeDemo",2);
			}
			XNUCLEO53L0A1_SetDisplayString(StrDisplay);
			break;
		case 3:
			if( status==0 ){ /* Push data logging to UART */
				if(RangingMeasurementData.RangeMilliMeter>1500){
          if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK){Error_Handler("vl53l0x_sigma.c -> Config_TIM6",1);}		
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);					
					sprintf(uartBUFF, "Device: %d; Mode: PWM; Out of range\n\r", VL53L0XDevs.Id);
					HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);					
					sprintf(StrDisplay, "BACK");					
				}
				if(RangingMeasurementData.RangeMilliMeter<20){
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);	
          if (HAL_TIM_Base_Stop_IT(&htim6) != HAL_OK){Error_Handler("vl53l0x_sigma.c -> Config_TIM6",1);}					
					sprintf(uartBUFF, "Device: %d; Mode: PWM; Distance: %d mm\n\r", VL53L0XDevs.Id, RangingMeasurementData.RangeMilliMeter);
					HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);				
					sprintf(StrDisplay, "STOP");

				}
				if(RangingMeasurementData.RangeMilliMeter<1500 && RangingMeasurementData.RangeMilliMeter>20){	
          TIM6_Config(RangingMeasurementData.RangeMilliMeter);		
          HAL_Delay(10);					
          if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK){Error_Handler("vl53l0x_sigma.c -> Config_TIM6",1);}					
					sprintf(uartBUFF, "Device: %d; Mode: PWM; Distance: %d mm\n\r", VL53L0XDevs.Id, RangingMeasurementData.RangeMilliMeter);
					HAL_UART_Transmit(&huart2, (uint8_t *)uartBUFF, strlen(uartBUFF), 100);

					Sensor_SetNewRange(&VL53L0XDevs,&RangingMeasurementData);
					/* Display distance in cm */
					if( RangingMeasurementData.RangeStatus == 0 ){
						sprintf(StrDisplay, "%3dc",(int)VL53L0XDevs.LeakyRange/10);	
					}else{
						sprintf(StrDisplay, " OUT");
					}
				}
			}else{
			Error_Handler("vl53l0x_sigma.c -> RangeDemo",1);
			}
			break;
		default:
			Error_Handler("vl53l0x_sigma.c -> RangeDemo",4);
			sprintf(StrDisplay, "OTER");		
			XNUCLEO53L0A1_SetDisplayString(StrDisplay);
	}
	XNUCLEO53L0A1_SetDisplayString(StrDisplay);
}
/********************************************************************************************************************************************************/
/**
  * @brief     This function is executed in case of error occurrence.
  * @param  file: the file where is this function.
  * @param  line: the line where is this function, or the number of error in the same function
  * @retval   None
  * Use:   Error_Handler("FILE->FUNCTION", 1);   or   Error_Handler("NULL", 0);
  */
void Error_Handler(char file[20], int line)
{
	char uartCom[100];

	if(line==0)
	{
		/* Toogle LED */
		for(uint8_t i=0;i<10;i++)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(50);
		}
	}
	/* User can add his own implementation to report the HAL error return state */
	sprintf(uartCom, "ERROR in file %s Error: %d\n\r", file,line);
	HAL_UART_Transmit(&huart2, (uint8_t *)uartCom, strlen(uartCom), 100);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}
/********************************************************************************************************************************************************/
/**
  * @brief  Configures EXTI lines 10 to 15 (connected to PC.13 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI lines 10 to 15 Interrupt priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
/********************************************************************************************************************************************************/
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
		if (counter>3){counter=1;}
    /* Increase counter when push user buton*/
		counter++;
		switch (counter){
			case 1:
				sprintf(uartBUFF, "CAS1");
			  break;
			case 2:
				sprintf(uartBUFF, "CAS2");
			  break;
			case 3:
				sprintf(uartBUFF, "CAS3");	
			  break;
			default:
				sprintf(uartBUFF, "OTRO");
				break;
		}							
		XNUCLEO53L0A1_SetDisplayString(uartBUFF);	
		for(uint32_t i=0; i<10000000; i++);
  }
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}
/********************************************************************************************************************************************************/
/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
void TIM6_Config(uint32_t period)                              
{
  __HAL_RCC_TIM6_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM6_IRQn, 2, 0);	
  HAL_NVIC_EnableIRQ(TIM6_IRQn);

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = (SystemCoreClock/1000) - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = period - 1;                 
  htim6.Init.ClockDivision = 0;
  htim6.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&htim6) != HAL_OK){Error_Handler("vl53l0x_sigma.c -> Config_TIM6",1);}

}
/********************************************************************************************************************************************************/
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
}
/********************************************************************************************************************************************************/
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MY_USART2_UART_Init(void)
{
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
		Error_Handler("vl53l0x_sigma.c -> MY_USART2_UART_Init",1);
  }
}
/********************************************************************************************************************************************************/
/**
  * @brief    This function config pins to work by the led ant the button in the target X_NUCLEO L4
  * @param  None
  * @retval   None
  */
void GPIO_LED_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	
  /*Configure GPIO pin : Extern red led + buzzer */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/********************************************************************************************************************************************************/
/******************************************************************_END_******************************************************************************/

