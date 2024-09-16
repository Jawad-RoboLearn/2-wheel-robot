#include "motors_timers.h"


void motor_init()
{
    // timer 2 ch 1 for first motor
    // timer 2 ch 2 for second motor


    // Enable clock access to timer 2: APB1 enable
    RCC->APB1ENR |= (1U<<0); // bit 0 is for timer 2 enable
    // set prescalar value: 10Khz PWM
    // PSC = sys_clk / timer_freq = 16000000/10000 = 1600
    TIM2->PSC = 1600 - 1; // -1 because start from 0
    // set autoreload value
    // Motor operates at 100Hz so PWM period = 1/100 = 10ms
    // ARR = timer_freq * PWM period = 100
    TIM2->ARR = 100 - 1; //
    
    // Configure PWM mode 1 on channel 1 for TIM2
    // pwm mode 1 = 0110: active while the counter is less than CCR value. 0110 
    // PWM mode bits OC1M is 3 bits wide and start at bit position 4
    TIM2->CCMR1 |= (6 << 4) // 6 is 0110 and 4 is OC1M start bit position
    TIM2->CCMR1 |= (1 << 3) // 3 is The OC1PE bit position

    // enable output on ch1
    TIM2->CCER |= (1U << 0); // CC1E 


    // Configure PWM mode 1 on channel 2 for TIM2
    // pwm mode 1 = 0110: active while the counter is less than CCR value. 0110 
    // PWM mode bits OC1M is 3 bits wide and start at bit position 4
    TIM2->CCMR1 |= (6 << 12) // 6 is 0110 and 12 is OC2M start bit position
    TIM2->CCMR1 |= (1 << 11) // 1 is The OC2PE bit position

    // enable output on ch2
    TIM2->CCER |= (1U << 4); //CC2E

    // enable timer/start timer
    TIM2->CR1 |= (1U<<0);



    // GPIOA SETTINGS FOR Direction
    // PA0 AND PA1
    RCC->AHB1ENR |= (1U<<0); // DO ONLY HERE ONE TIME
    //GPIO output mode as 01
    GPIOA->MODER &= ~(1U<<1); // PA0
    GPIOA->MODER |= (1U<<0);
    GPIOA->MODER &= ~(1U<<3); // PA1
    GPIOA->MODER |= (1U<<2);
    GPIOA->MODER &= ~(1U<<5); // PA2
    GPIOA->MODER |= (1U<<4);
    GPIOA->MODER &= ~(1U<<7); // PA3
    GPIOA->MODER |= (1U<<6);
}



void set_M1_dir(int dir)
{
    if (dir == 1)
    {
        GPIOA->ODR |= (1<<0); // IN1 high
        GPIOA->ODR &= ~(1<<1); //IN2 low
    }
    else
    {
        GPIOA->ODR &= ~(1<<0); // IN1 low
        GPIOA->ODR |= (1<<1); //IN2 high
    }
}

void set_M2_dir(int dir)
{
    if (dir == 1)
    {
        GPIOA->ODR |= (1<<2); // IN1 high
        GPIOA->ODR &= ~(1<<3); //IN2 low
    }
    else
    {
        GPIOA->ODR &= ~(1<<2); // IN1 low
        GPIOA->ODR |= (1<<3); //IN2 high
    }
}

void set_M1_speed(uint32_t duty_cycle)
{
    // limit
    if (duty_cycle > 100)
    {
        duty_cycle = 100;
    }
    if (duty_cycle < 0)
    {
        duty_cycle = 0;
    }

    TIM2->CCR1 = duty_cycle;
    
}
void set_M1_speed(uint32_t duty_cycle)
{
    // limit
    if (duty_cycle > 100)
    {
        duty_cycle = 100;
    }
    if (duty_cycle < 0)
    {
        duty_cycle = 0;
    }

    TIM2->CCR2 = duty_cycle;
    
}

