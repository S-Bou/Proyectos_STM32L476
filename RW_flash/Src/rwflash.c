/**
 *******************************************************************************************************************************************************
  * @file    rwflash.c 
  * @author  Bou
  * @version V1.0.0
  * @date    18-October-2020
  * @brief   This file provides the definition of the functions to manage the flash memory.
  *******************************************************************************************************************************************************
 */
#include "rwflash.h"
#include "string.h"
/********************************************************************************************************************************************************/
/*Declaration of struct to store the information give by user.*/
FlashDirAndPage FlashDirStruct;
/********************************************************************************************************************************************************/
/**
  * @brief  This function configure a led to indicate if ocurs any fail and set the direction of where user whis start to write memory 
  * @param  Address specifies the address to be programmed.
  * @param  Page specifies the page to be programmed.
  * @retval None
  */
void InitFlashRW(uint32_t bank, uint32_t address, uint32_t page)
{
	/*Call to configuration of pin to use user led*/
	GPIO_Init_UserLED();
	/*Set the address given by user in the struct*/
	FlashDirStruct.DDU   = address;
	/*Set the page given by user in the struct*/
	FlashDirStruct.PAGE = page;
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
uint32_t WritePageInFlash(uint64_t userData[])
{
	uint32_t ErrorCode = 0;
	uint32_t addres = GetMemDir();
	uint32_t page =  GetPageDir();
	FlashDirStruct.LOND = sizeof(userData);
	
	FLASH_EraseInitTypeDef EraseInitStruct;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = page ;

	HAL_FLASH_Unlock();

	  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); 
	
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &ErrorCode) != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	// indicator for erasing fails
		return ErrorCode = HAL_FLASH_GetError();
	}
	
	for(uint32_t i = 0; i < FlashDirStruct.LOND; i++)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addres, userData[i]) == HAL_OK)
		{
			addres += 8;
		}else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	// indicator for program fails
			return ErrorCode = HAL_FLASH_GetError();
		}	
	}
	HAL_FLASH_Lock();
	
	return ALL_OK;
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
void ReadMemDir(uint32_t addres)
{
	for(uint8_t i = 0; i < FlashDirStruct.LOND; i++)
	{
		printf("En la direccion %#x hay: ", GetMemDir() + i*8);
		printf("%#x%x\n", *((uint32_t *)GetMemDir() + i*2), *((uint32_t *)GetMemDir() + (i*2+1)));
		HAL_Delay(100);
	}
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
uint32_t  GetMemDir(void)
{
	return FlashDirStruct.DDU;
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function...
  * @param  None
  * @retval None
  */
uint32_t  GetPageDir(void)
{
	return FlashDirStruct.PAGE;
}
/********************************************************************************************************************************************************/
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_Init_UserLED(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_PIN_5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
/********************************************************************************************************************************************************/
