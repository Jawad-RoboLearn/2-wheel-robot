#ifndef MOTORS_TIMERS_H
#define MOTORS_TIMERS_H


#include "stm32f4xx.h"

// MOTOR
void motor_init(); // It can set up timer 2 ch1 for motor 1 and timer 2 ch2 for motor 2
void set_M1_dir(int dir); // set motor 1 direction
void set_M2_dir(int dir); // set motor 2 direction
void set_M1_speed_duty(uint32_t duty_cycle); // set motor 1 speed
void set_M2_speed_duty(uint32_t duty_cycle); // set motor 2 speed
float get_M1_speed_counts (uint32_t duty_cycle); //get desired speed in counts
float get_M2_speed_counts (uint32_t duty_cycle); //get desired speed in counts
#define MAX_SPEED1_COUNT = 500;
#define MAX_SPEED2_COUNT = 500;
// ENCODER
#define SYSCFG_EXTICR2_EXTI4       ((uint32_t)0x000F) // Mask for EXTI4
#define SYSCFG_EXTICR2_EXTI4_PA    ((uint32_t)0x0000) // Connect EXTI4 to PA4
#define SYSCFG_EXTICR2_EXTI5       ((uint32_t)0x000F) // Mask for EXTI5
#define SYSCFG_EXTICR2_EXTI5_PA    ((uint32_t)0x0000) // Connect EXTI5 to PA5
#define SYSCFG_EXTICR2_EXTI6_PA    ((uint32_t)0x0000) // Connect EXTI4 to PA6
#define SYSCFG_EXTICR2_EXTI7_PA    ((uint32_t)0x0000) // Connect EXTI4 to PA7

void GPIO_encoder_init(void);
void ext_interrupt_init(void);
void enc1A_IRQ_handler(void);
void enc1B_IRQ_handler(void);
void enc2A_IRQ_handler(void);
void enc2B_IRQ_handler(void);
void EXTI4_15_IRQHandler(void);
void init_speed_timer(void);
void timer3_speed_IRQhandler(void);
float get_speed1(void);
float get_speed2(void);

volatile int32_t encoder1_count = 0;
volatile int32_t encoder2_count = 0;
volatile int32_t last_encoder1_count = 0;
volatile int32_t last_encoder2_count = 0;
volatile float current_speed1 = 0.0; 
volatile float current_speed2 = 0.0; 
float dt = 0.1; //100ms or 10 hz is set
#endif
