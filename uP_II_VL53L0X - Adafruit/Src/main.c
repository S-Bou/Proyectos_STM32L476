/**
  ******************************************************************************
  * This is the template for Informatica Industrial II. Lunes tarde group.
  * Based on the STMicroelectronics HAL template
  *
  * See http://aperles.blogs.upv.es/arm-cortex-m-practico-1-introduccion-a-los-microcontroladores-stm32-de-st/
  *
  * Good look,
  * Angel Perles
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "vl53l0x_sigma.h"
#include "vl53l0x_api.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
VL53L0X_Error   Status = VL53L0X_ERROR_NONE; ///< indicates whether or not the sensor has encountered an error
	
#define VL53L0X_I2C_ADDR  0x52 ///< Default sensor I2C address
#define TXBUFFERSIZE    (COUNTOF(buffInit) - 1)
#define TXBUFFERDEVSIZE    (COUNTOF(buff) - 1)
// I2C addresses of GPIO expanders on the X-NUCLEO-53L1A1
#define EXPANDER_1_ADDR 0x52 // 0x42 << 1
#define EXPANDER_2_ADDR 0x86 // 0x43 << 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
	extern UART_HandleTypeDef huart2;
	extern I2C_HandleTypeDef hi2c1;
  uint8_t buffInit[] = " *** UART2 Init OK ***\n\r ";
	uint8_t buff[50];
	uint8_t medida=0;
	uint8_t ErrorNUM=0;
  VL53L0X_RangingMeasurementData_t RangingData;
  VL53L0X_Dev_t            MyDevice;
  VL53L0X_Dev_t            *pMyDevice  = &MyDevice;
  VL53L0X_Version_t      Version;
  VL53L0X_Version_t      *pVersion   = &Version;
  VL53L0X_DeviceInfo_t   DeviceInfo;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/** Main program **************************************************************/
int main(void)
{
  /* STM32F4xx HAL library initialization */
  HAL_Init();
  /* Configure the System clock to 80 MHz */
  SystemClock_Config();
 /* Initialize all configured peripherals */
	MY_USART2_UART_Init();
	MY_I2C1_Init();
	HAL_UART_Transmit(&huart2, (uint8_t *)buffInit, TXBUFFERSIZE, 100);	
	/*User code*/
	GPIO_LED_Config();

	uint32_t  refSpadCount;
  uint8_t   isApertureSpads;
	uint8_t   VhvSettings;
  uint8_t   PhaseCal;
  // Initialize Comms
  pMyDevice->I2cDevAddr      =  0x52;  // default
  pMyDevice->comms_type      =  1;
  pMyDevice->comms_speed_khz =  400;

	Status = VL53L0X_DataInit( &MyDevice );         // Data initialization
	
		Error_Handler(11);		
	
	if( Status == VL53L0X_ERROR_NONE ) {
		Status = VL53L0X_StaticInit( pMyDevice );       // Device Initialization
		Error_Handler(1);		
	}

	if( Status == VL53L0X_ERROR_NONE ) {
		Status = VL53L0X_PerformRefSpadManagement( pMyDevice, &refSpadCount, &isApertureSpads ); // Device Initialization
		Error_Handler(2);		
	}

	if( Status == VL53L0X_ERROR_NONE ) {
		Status = VL53L0X_PerformRefCalibration( pMyDevice, &VhvSettings, &PhaseCal );           // Device Initialization
		Error_Handler(3);		
	}

	if( Status == VL53L0X_ERROR_NONE ) {
		Status = VL53L0X_SetDeviceMode( pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING ); 
		Error_Handler(4);		
	}
	
	// Enable/Disable Sigma and Signal check
	if( Status == VL53L0X_ERROR_NONE ) {
		Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
		Error_Handler(5);		
	}

	if( Status == VL53L0X_ERROR_NONE ) {
		Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
		Error_Handler(6);		
	}

	if( Status == VL53L0X_ERROR_NONE ) {
		Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1 );
		Error_Handler(7);		
	}

	if( Status == VL53L0X_ERROR_NONE ) {
		Status = VL53L0X_SetLimitCheckValue( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)( 1.5 * 0.023 * 65536 ) );
		Error_Handler(8);		
	}

	Error_Handler(12);
  /* Infinite loop */
  while (1)
  {
//		VL53L0X_WaitDeviceReadyForNewMeasurement(Dev, VL53L0X_DEFAULT_MAX_LOOP);

//		VL53L0X_GetRangingMeasurementData( Dev, &RangingData );

//		printf( (char*)buff, "%d\n\r", RangingData.RangeMilliMeter);
//		HAL_UART_Transmit( &huart2, buff, strlen( (char*)buff ), 0xFFFF );

//		VL53L0X_ClearInterruptMask( Dev , VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR);

//		HAL_Delay(200);
  }
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
