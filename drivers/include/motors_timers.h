#ifndef MOTORS_TIMERS_H
#define MOTORS_TIMERS_H


#include "stm32f4xx.h"

void motor_init(); // It can set up timer 2 ch1 for motor 1 and timer 2 ch2 for motor 2
void set_M1_dir(int dir); // set motor 1 direction
void set_M2_dir(int dir); // set motor 2 direction
void set_M1_speed(uint32_t duty_cycle); // set motor 1 speed
void set_M2_speed(uint32_t duty_cycle); // set motor 2 speed

#endif
