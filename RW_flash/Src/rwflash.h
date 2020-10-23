/**
 ********************************************************************************************************************************************************
  * @file    rwflash.h
  * @author  Bou
  * @version V1.0.0
  * @date    18-October-2020
  * @brief   This file provides...
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
	
} FlashDirAndPage;

/* Peripheral Control functions  **********************************************/
/** @addtogroup FLASH_Exported_Functions_Group2
  * @{
  */
void InitFlashRW(uint32_t address, uint32_t page);

uint32_t WritePageInFlash(uint64_t userData[]);

uint32_t  GetMemDir(void);

uint32_t  GetPageDir(void);

void GPIO_Init(void);
/********************************************************************************************************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __RWFLASH_H */
