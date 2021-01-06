/**
    @file servo.h
    @brief Servo handling
*/

#ifndef SERVO_H
#define SERVO_H

error_t servo_Init(void);
error_t servo_SetPosition(int16_t tenth_degree);

#endif
/**** End of file ***/
