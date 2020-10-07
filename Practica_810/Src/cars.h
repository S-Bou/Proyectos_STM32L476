/**
@file cars .h
*/
#ifndef  __CARS_H
#define __CARS_H

#ifdef __cplusplus
extern "C" {
#endif
/*################################### INCLUDES/DEFINES #####################################################*/
#include "stdint.h"
/*################################### VARIABLES ############################################################*/

/*################################### FUNCTIONS ############################################################*/
void cars_Init ( void );
uint32_t cars_GetCount ( void );
void cars_IncrementCount ( void );
/*###########################################################################################################*/
#ifdef __cplusplus
}
#endif

#endif /* __CARS_H */
