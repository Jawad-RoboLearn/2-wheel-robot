#include "motors_timers.h"

// Init GPIO and Timer settings
void motor_init()
{

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

    // timer 3 init anc callback for speed
    init_speed_timer();
}

// set Motor 1 desired direction
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

// set Motor 2 desired direction
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

// set Motor 1 desired speed
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

// set Motor 2 desired speed
void set_M2_speed(uint32_t duty_cycle)
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

// Encoder GPIO init
void GPIO_encoder_init(void)
{
    // PA clock is already init in motors_timers.c

    // set PA4 and PA5 for motor 1 encoder AB inputs
    GPIOA->MODER &= ~(1U<<8); // PA4
    GPIOA->MODER &= ~(1U<<9);
    GPIOA->MODER &= ~(1U<<10); // PA5
    GPIOA->MODER &= ~(1U<<11);

     // set PA6 and PA7 for motor 2 encoder AB inputs
    GPIOA->MODER &= ~(1U<<12); // PA6
    GPIOA->MODER &= ~(1U<13);
    GPIOA->MODER &= ~(1U<<14); // PA7
    GPIOA->MODER &= ~(1U<<15);

    // PULL UP
    // PA4
    GPIOA->PUPDR &= ~(1U<<9);
    GPIOA->PUPDR |= (1U<<8);
    // PA5
    GPIOA->PUPDR &= ~(1U<11);
    GPIOA->PUPDR |= (1U<<10);
    // PA6
    GPIOA->PUPDR &= ~(1U<<13);
    GPIOA->PUPDR |= (1U<<12);
    // PA7
    GPIOA->PUPDR &= ~(1U<15);
    GPIOA->PUPDR |= (1U<<14);

}

// external interrupt init for encoders
void ext_interrupt_init(void)
{
    // ENABLE SYSCFGEN
    RCC->APB2ENR |= (1U<<14); 

    // CONFIGURE PA4 for EXTI4
    //EXTICR[1] for PA4-7
    //SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4; // CLEAR PREV
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PA; //0000 for PA4

    //SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI5; // CLEAR PREV
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PA; //0000 for PA5

    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PA; //0000 for PA6
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PA; //0000 for PA7

    // unMASK A4 A5 A6 A7
    EXTI->IMR |= (1U<<4);
    EXTI->IMR |= (1U<<5);
    EXTI->IMR |= (1U<<6);
    EXTI->IMR |= (1U<<7);

    // TRIGGER FALL EDGE
    EXTI->FTSR |= (1U<<4);
    EXTI->FTSR |= (1U<<5);
    EXTI->FTSR |= (1U<<6);
    EXTI->FTSR |= (1U<<7);

    //nvic config
    NVIC_SetPriority(EXTI4_15_IRQn, 2); // Set priority
    NVIC_EnableIRQ(EXTI4_15_IRQn);      // Enable interrupts
}

// ENCA Motor 1 interrupt handler
void enc1A_IRQ_handler(void)
{ // ENCA OF MOTOR1 is connected to PA4
    if (GPIOA->IDR & (1U<<4)) // ENCA is HIGH
    {
        encoder1_count--; //ACLKWISE
    }
    else
    {
        encoder1_count++; //CLCKWISE
    }
}

// ENCB Motor 1 interrupt handler
void enc1B_IRQ_handler(void)
{ // ENCB OF MOTOR1 is connected to PA5
    if (GPIOA->IDR & (1U<<5)) // ENCB is HIGH
    {
        encoder1_count++; //CLKWISE
    }
    else
    {
        encoder1_count--; //ACLCKWISE
    }
}

// ENCA Motor 2 interrupt handler
void enc2A_IRQ_handler(void)
{ // ENCA OF MOTOR1 is connected to PA6
    if (GPIOA->IDR & (1U<<6)) // ENCA is HIGH
    {
        encoder2_count--; //ACLKWISE
    }
    else
    {
        encoder2_count++; //CLCKWISE
    }
}

// ENCB Motor 2 interrupt handler
void enc2B_IRQ_handler(void)
{ // ENCB OF MOTOR2 is connected to PA7
    if (GPIOA->IDR & (1U<<7)) // ENCB is HIGH
    {
        encoder2_count++; //CLKWISE
    }
    else
    {
        encoder2_count--; //ACLCKWISE
    }
}

// MANAGE MOTOR 1 ENCODER INTERRUPTS
void EXTI4_15_IRQHandler(void)
{
    // check for PA4 interrupt
    if (EXTI->PR & (1U<<4))
    {
        enc1A_IRQ_handler();
        EXTI->PR |= (1U<<4); // CLEAR PENDING BIT
    }

    // check for PA5 interrupt
    if (EXTI->PR & (1U<<5))
    {
        enc1B_IRQ_handler();
        EXTI->PR |= (1U<<5); // CLEAR PENDING BIT
    }

     // check for PA6 interrupt
    if (EXTI->PR & (1U<<6))
    {
        enc2A_IRQ_handler();
        EXTI->PR |= (1U<<6); // CLEAR PENDING BIT
    }

    // check for PA7 interrupt
    if (EXTI->PR & (1U<<7))
    {
        enc2B_IRQ_handler();
        EXTI->PR |= (1U<<7); // CLEAR PENDING BIT
    }
}


void init_speed_timer(void)
{
    // ENABLE TIMER3 CLOCK
    RCC->ABP1ENR |= (1U<<1);

    TIM3->PSC = 1600 - 1; // -1 because start from 0
    // every 100ms aligned with timer2
    TIM3->ARR = 100 - 1; //

    // ENABLE UPDATE INTERRUPT
    TIM3->DIER |= (1U<<0);

    // enable timer/start timer
    TIM3->CR1 |= (1U<<0);

    NVIC_SetPriority(TIM3_IRQn, 1); // Set priority
    NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt
}

void timer3_speed_IRQhandler(void)
{
    // CHK IF UPDATE FLAG IS SET
    if (TIM3->SR & (1U<<0))
    {
        TIM3->SR &= ~(1U<<0); // CLEAR FLAG

        //CALC SPEED
        current_speed1 = (encoder1_count - last_encoder1_count)/dt;
        last_encoder1_count = encoder1_count;

        current_speed2 = (encoder2_count - last_encoder2_count)/dt;
        last_encoder2_count = encoder2_count;
    }
}

float get_speed1(void)
{
    return current_speed1;
}

float get_speed2(void)
{
    return current_speed2;
}
