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
#include "rwflash.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*Definition of memory direction*/
#define DIR_MEM_INIT 0x0807F800
/*Definition of bank number */
#define BANK_NUM FLASH_BANK_1  /*Not used if only wtite in determinated page */
/*Definition of page number  */
#define PAGE_MEM_INIT 255
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*Example of vector of data in hexadecimal*/
//uint64_t DATA[4] = {0x1111111111111111, 0x2222222222222222, 0x3333333333333333, 0x4444444444444444};
/*Vector to store data readed in memory*/
uint64_t DATA_READ[4] = {0, 0, 0, 0};
/*Example of vector of data in decimal*/
uint64_t DATA[4] = {90, 30, 60, 90}; /*Example of  vector with position in degrees of some servos */
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
//static void Error_Handler(void);
//void fputc_SetXY(uint16_t x, uint16_t y);

/* Private functions ---------------------------------------------------------*/

/** Main program **************************************************************/
int main(void)
{
  /* STM32F4xx HAL library initialization */
  HAL_Init();
  /* Configure the System clock to 180 MHz */
  SystemClock_Config();
	/*User code*/
	/*Send information of memory direction to module*/
	InitFlashRW(BANK_NUM, DIR_MEM_INIT, PAGE_MEM_INIT);
	/*Write data in memory direction choosed */
	WritePageInFlash(DATA);
	/*Read data in memory direction defined and save in vector*/
	ReadMemDir(DIR_MEM_INIT, DATA_READ);
	/*Print in console the data saved in vector*/
	for(uint8_t i = 0; i < 4; i++)
	{
		printf("En la direccion %#x hay: ", DIR_MEM_INIT + i*8);
		printf("%#" PRIu64 "\n", DATA_READ [i]);
		HAL_Delay(500);
	}
  /* Infinite loop */
  while (1)
  {
		/**/
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
