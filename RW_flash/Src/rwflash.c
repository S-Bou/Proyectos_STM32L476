/**
 *******************************************************************************************************************************************************
  * @file    rwflash.c 
  * @author  Bou
  * @version V1.0.0
  * @date    18-October-2020
  * @brief   This file provides...
  *******************************************************************************************************************************************************
 */
#include "rwflash.h"
#include "string.h"
/********************************************************************************************************************************************************/

/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void ClearPage(void)
{
			FLASH_EraseInitTypeDef EraseInitStruct;
			
			EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
			EraseInitStruct.Banks       = FLASH_BANK_1;
			EraseInitStruct.Page         = 255;
			EraseInitStruct.NbPages   = 1;

      uint32_t PageError = 0;

      HAL_FLASH_Unlock();
	
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGAERR);
	
      if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
			{
    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	// indicator for erasing fails
      }
			
			HAL_FLASH_Lock();
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void WritePage(void)
{
	
	int8_t mydata[] = "Hola";
	uint64_t DirMEM = 0x0807F800;
	uint64_t userDATA = ((uint64_t) mydata[0])+((uint64_t) mydata[1]<<8)+((uint64_t) mydata[2]<<16)+((uint64_t) mydata[3]<<24);
	FLASH_EraseInitTypeDef EraseInitStruct;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks        = FLASH_BANK_1;
	EraseInitStruct.Page         = 255;
	EraseInitStruct.NbPages    = 1;

	uint32_t PageError = 0;

	HAL_FLASH_Unlock();

	  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); 
	
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	// indicator for erasing fails
	}
//	for(uint8_t i=0; i<strlen(mydata); i++)
//		for(uint8_t i=0; i<=1; i++)
//	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, DirMEM, userDATA) != HAL_OK)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	// indicator for program fails
		}
//		DirMEM+=8;
//		HAL_Delay(500);
//	}
	
	HAL_FLASH_Lock();
}
/********************************************************************************************************************************************************/
