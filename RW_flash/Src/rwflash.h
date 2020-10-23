/**
 ********************************************************************************************************************************************************
  * @file    rwflash.h
  * @author  Bou
  * @version V1.0.0
  * @date    18-October-2020
  * @brief   This file provides the declarations of the functions to manage the flash memory.
 ********************************************************************************************************************************************************
 */
#ifndef  __RWFLASH_H
#define __RWFLASH_H

#ifdef __cplusplus
extern "C" {
#endif
/********************************************************************************************************************************************************/
#include "stm32l4xx_hal.h"
#include "stdio.h"
#define ALL_OK 666
/********************************************************************************************************************************************************/
/**
  * @brief  Flash direction and data structure definition
  */
typedef struct
{
	uint32_t DDU;    /*This parameter contain the direction defined by the user.*/
	
	uint32_t PAGE;	/*This parameter contain the number of page where start the direction of memory.*/
	
	uint32_t LOND;
	
	uint32_t BANK_X;
	
} FlashDirAndPage;

/* Peripheral Control functions  **********************************************/
/** @addtogroup FLASH_Exported_Functions_Group2
  * @{
  */
void InitFlashRW(uint32_t bank, uint32_t address, uint32_t page);

uint32_t WritePageInFlash(uint64_t userData[]);

void ReadMemDir(uint32_t addres);

uint32_t  GetMemDir(void);

uint32_t  GetPageDir(void);

void GPIO_Init_UserLED(void);
/********************************************************************************************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __RWFLASH_H */
