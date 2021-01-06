/**
    Copyright (C) 2018 The pepe Team
    
    @file    error.h
    @author  Angel Perles
    @version V0.1
    @date    2018-11-29
    @brief   Error support
          
    Bla, bla, and bla
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ERROR_H
#define ERROR_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef uint8_t error_t;

/* Exported constants --------------------------------------------------------*/
#define Error_NoError               0x00    // All worked OK
#define Error_ServoInit             0x01    // Problems initiating servo
#define Error_ServoCommand          0x02    // Problems commanding servo
#define Error_UnimplementedError    0xFF    // Not yet specified
//IMPORTANT: enumerated-based error list is safer

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void error_Handler(char *file, int line, error_t error_code); // "int" OK here
const char* error_ToStr(error_t error_code);

#endif
/*** End of file **************************************************************/
