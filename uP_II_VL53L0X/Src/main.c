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
#include "vl53l0x_api.h"
#include "vl53l0x_sigma.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
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
  VL53L0X_RangingMeasurementData_t RangingData;
  VL53L0X_Dev_t  vl53l1_c; // center module 
  VL53L0X_DEV    Dev = &vl53l1_c; 
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

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
	
	// initialize vl53l1x communication parameters
  Dev->I2cHandle = &hi2c1;
  Dev->I2cDevAddr = 0x52;

	// set XSHUT (enable center module) -> expander 1, GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  HAL_I2C_Master_Receive( &hi2c1, EXPANDER_1_ADDR, buff, 1, 0xFFFF );
  buff[1] = buff[0] | ( 1 << ( 15 - 8 ) ); // set GPIO_15
  buff[0] = 0x13; // GPSR + 1 ( GPIO set pin state register)
  HAL_I2C_Master_Transmit( &hi2c1, EXPANDER_1_ADDR, buff, 2, 0xFFFF );

  HAL_Delay( 2 );
	
	/*** VL53L1X Initialization ***/
  VL53L0X_WaitDeviceBooted( Dev );
  VL53L0X_DataInit( Dev );
  VL53L0X_StaticInit( Dev );
  //VL53L0X_SetDistanceMode( Dev, DISTANCEMODE_LONG );
	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, 50000 );
  VL53L0X_SetInterMeasurementPeriodMilliSeconds( Dev, 500 );
  VL53L0X_StartMeasurement( Dev );
	
  /* Infinite loop */
  while (1)
  {
		 //VL53L0X_WaitMeasurementDataReady( Dev );
		VL53L0X_WaitDeviceReadyForNewMeasurement(Dev, VL53L0X_DEFAULT_MAX_LOOP);

    VL53L0X_GetRangingMeasurementData( Dev, &RangingData );

    printf( (char*)buff, "%d\n\r", RangingData.RangeMilliMeter);
    HAL_UART_Transmit( &huart2, buff, strlen( (char*)buff ), 0xFFFF );

    VL53L0X_ClearInterruptMask( Dev , VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR);
		
		HAL_Delay(3000);
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
static void SystemClock_Config(void)
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
