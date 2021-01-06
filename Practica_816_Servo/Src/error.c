/**
    @file error.c
    @author Angel Perles
    @version V0.1
    @date    2018-11-29  
    @brief Error report functions ... at this point

     This module implements functions to obtain an human-readable string explaining the meaning of each 
*/



/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>
#include "error.h"

/* Private typedef -----------------------------------------------------------*/
struct error_list_t {
    error_t error_code;
    const char *meaning;
};

/* Private define ------------------------------------------------------------*/
struct error_list_t error_list[] = {
    {Error_NoError,             "No error."},
    {Error_ServoInit,           "Servo can't be initiated."},
    {Error_ServoCommand,        "Servo command problems."},
    {Error_UnimplementedError,  "Error not implemented."} // end of list
};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/******************************************************************************/
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void error_Handler(char *file, int line, error_t error_code)
{

  printf("ERROR in file %s on line %d and reason %s\r\n", file, line, error_ToStr(error_code));
   
  for(;;)
  {
  }

}

/******************************************************************************/
/**
  @brief Returns an human-readable string based on the given cd_error code

  @param error_code value to be translated to the equivalent string
  @returns pointer to an standard NULL-terminated C string explain the meaning of the code

*/
const char* error_ToStr(error_t error_code) {

    uint8_t i;

    i=0;

    while ((error_list[i].error_code != error_code) && (error_list[i].error_code != Error_UnimplementedError)) {
        i++;
    }

    return(error_list[i].meaning);
}


/* Private functions ---------------------------------------------------------*/
/* End of file ****************************************************************/
