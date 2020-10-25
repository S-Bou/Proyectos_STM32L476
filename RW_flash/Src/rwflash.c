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
/********************************************************************************************************************************************************/
/*Declaration of struct to store the information give by user */
FlashDirAndPage FlashDirStruct;
/********************************************************************************************************************************************************/
/**
  * @brief  This function configure a led to indicate if ocurs any fail and set the direction of where user whis start to write memory 
  * @param  Address specifies the address to be programmed.
  * @param  Page specifies the page to be programmed.
  * @retval None.
  */
void InitFlashRW(uint32_t bank, uint32_t address, uint32_t page)
{
	/*Call to configuration of pin to use user led */
	GPIO_Init_UserLED();
	/*Set the address given by user in the struct */
	FlashDirStruct.DDU   = address;
	/*Set the page given by user in the struct */
	FlashDirStruct.PAGE = page;
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function erase the page defined by the user before write the data.
  * @param  UserData contain the data defined by the user.
  * @retval ErrorCode can be used to know the tipe of erros ocurrs.
  * @retval All_OK can be used to know error has occurred and what kind is.
  */
uint32_t WritePageInFlash(uint64_t userData[])
{
	/*Declaration of variables */
	uint32_t ErrorCode = 0;
	uint32_t addres = GetMemDir();
	uint32_t pageSelected =  GetPageDir();
	/*Find out size of vector of data.*/
	FlashDirStruct.LOND = sizeof(userData);
	/*Declaration of struct to configuration the mode of erase */
	FLASH_EraseInitTypeDef EraseInitStruct;
	/*Configuration the mode of erase.*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	//EraseInitStruct.Banks = FLASH_BANK_1;           
	//EraseInitStruct.NbPages = 1;
	EraseInitStruct.Page = pageSelected;
	/*Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); 
	/*Erase the specified FLASH memory pages */
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &ErrorCode) != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	/*indicator for erasing fails */
		return ErrorCode = HAL_FLASH_GetError();	/*Return value of code if error */
	}
	
	for(uint32_t i = 0; i < FlashDirStruct.LOND; i++)
	{
		/*Program double word of a row at a specified address */
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addres, (uint64_t)userData[i]) == HAL_OK)
		{
			addres += 8;	/*Increase the direction to write next word */
		}else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	/*indicator for program fails */
			return ErrorCode = HAL_FLASH_GetError();	/*Return value of code if error */
		}	
	}
	/*Lock the FLASH control register access */
	HAL_FLASH_Lock();
	/*Return a value if all finished fine */
	return ALL_OK;
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function allows show in console the value in the specificated memory by user.
  * @param  Address specifies the address to be programmed.
  * @retval None
  */
void ReadMemDir(uint32_t addres, void *data)
{
	for(uint8_t i = 0; i < FlashDirStruct.LOND; i++)
	{
		*((uint64_t *)data + i) =   *((uint64_t *)GetMemDir() + i) ;            //  *((uint32_t *)GetMemDir() + i*2)+*((uint32_t *)GetMemDir() + (i*2+1));
	}
	
//	for(uint8_t i = 0; i < FlashDirStruct.LOND; i++)
//	{
//		printf("En la direccion %#x hay: ", GetMemDir() + i*8);
//		printf("%#x%x\n", *((uint32_t *)GetMemDir() + i*2), *((uint32_t *)GetMemDir() + (i*2+1)));
//		HAL_Delay(100);
//	}
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function get the direction of memory defined by the user.
  * @param  None
  * @retval FlashDirStruct.DDU contain the direction defined by the user.
  */
uint32_t  GetMemDir(void)
{
	/*Return the direction of memory defined by the user */
	return FlashDirStruct.DDU;
}
/********************************************************************************************************************************************************/
/**
  * @brief  This function get the number of page defined by the user.
  * @param  None
  * @retval FlashDirStruct.PAGE contain the number of the page defined by the user.
  */
uint32_t  GetPageDir(void)
{
	/*Return the number of the page defined by the user */
	return FlashDirStruct.PAGE;
}
/********************************************************************************************************************************************************/
/**
  * @brief GPIO initialization function to configure PA5 as output.
  * @param None
  * @retval None
  */
void GPIO_Init_UserLED(void)
{
	/*Declaration of struct to configuration the mode of  pin */
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
