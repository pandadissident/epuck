#include <stdlib.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <gpio.h>
#include <motor.h>

#define TIMER_CLOCK         84000000
#define TIMER_FREQ          100000 // [Hz]
#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]

#define MOTOR_RIGHT_A	GPIOE, 13
#define MOTOR_RIGHT_B	GPIOE, 12
#define MOTOR_RIGHT_C	GPIOE, 14
#define MOTOR_RIGHT_D	GPIOE, 15

#define MOTOR_LEFT_A	GPIOE, 9
#define MOTOR_LEFT_B	GPIOE, 8
#define MOTOR_LEFT_C	GPIOE, 11
#define MOTOR_LEFT_D	GPIOE, 10

#define MOTOR_RIGHT_TIMER		TIM6
#define MOTOR_RIGHT_TIMER_EN	RCC_APB1ENR_TIM6EN
#define MOTOR_RIGHT_IRQHandler	TIM6_DAC_IRQHandler
#define MOTOR_RIGHT_IRQ			TIM6_DAC_IRQn

#define MOTOR_LEFT_TIMER		TIM7
#define MOTOR_LEFT_TIMER_EN		RCC_APB1ENR_TIM7EN
#define MOTOR_LEFT_IRQ			TIM7_IRQn
#define MOTOR_LEFT_IRQHandler	TIM7_IRQHandler

#define SPEED_CONTROL       0
#define POSITION_CONTROL    1

//some static global variables
static int16_t right_speed = 0; 			    // in [step/s]
static int16_t left_speed = 0; 				    // in [step/s]
static int16_t counter_step_right = 0;          // in [step]
static int16_t counter_step_left = 0; 		    // in [step]
static int16_t position_to_reach_right = 0;	    // in [step]
static int16_t position_to_reach_left = 0;	    // in [step]
static uint8_t position_right_reached = 0;
static uint8_t position_left_reached = 0;
static uint8_t state_motor = 0;

//tables containing the steps for the motors
static const uint8_t step_halt[NB_OF_PHASES] = {0, 0, 0, 0};
static const uint8_t step_table[NSTEP_ONE_EL_TURN][NB_OF_PHASES] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1},
};

void motor_init(void)
{
    // motor GPIO configuration
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    // right motor
    gpio_config_output_pushpull(MOTOR_RIGHT_A);
	gpio_config_output_pushpull(MOTOR_RIGHT_B);
    gpio_config_output_pushpull(MOTOR_RIGHT_C);
    gpio_config_output_pushpull(MOTOR_RIGHT_D);

    // left motor
    gpio_config_output_pushpull(MOTOR_LEFT_A);
    gpio_config_output_pushpull(MOTOR_LEFT_B);
    gpio_config_output_pushpull(MOTOR_LEFT_C);
    gpio_config_output_pushpull(MOTOR_LEFT_D);


    // Motor right timer configuration
    // Enable Motor right timer clock
    RCC->APB1ENR |= MOTOR_RIGHT_TIMER_EN;

    // Enable Motor right timer interrupt vector
    NVIC_EnableIRQ(MOTOR_RIGHT_IRQ);

    // Configure Motor right timer
    // final timer clock  = timer clock / (prescaler + 1)
    MOTOR_RIGHT_TIMER->PSC = (TIMER_CLOCK / TIMER_FREQ) - 1;    
    MOTOR_RIGHT_TIMER->ARR = 0;
    MOTOR_RIGHT_TIMER->DIER = TIM_DIER_UIE;  // enable update interrupt
    MOTOR_RIGHT_TIMER->CR1 = TIM_CR1_CEN;    // enable timer


    // Motor left timer configuration
    // Enable Motor left timer clock
    RCC->APB1ENR |= MOTOR_LEFT_TIMER_EN;

    // Enable Motor left timer interrupt vector
    NVIC_EnableIRQ(MOTOR_LEFT_IRQ);

    // Configure Motor left timer
    // final timer clock  = timer clock / (prescaler + 1)
    MOTOR_LEFT_TIMER->PSC = (TIMER_CLOCK / TIMER_FREQ) - 1;
    MOTOR_LEFT_TIMER->ARR = 0;
    MOTOR_LEFT_TIMER->DIER = TIM_DIER_UIE;  // enable update interrupt
    MOTOR_LEFT_TIMER->CR1 = TIM_CR1_CEN;    // enable timer
}

static void right_motor_update(const uint8_t *out)
{
    // Right motor
    out[0] ? gpio_set(MOTOR_RIGHT_A) : gpio_clear(MOTOR_RIGHT_A);
    out[1] ? gpio_set(MOTOR_RIGHT_B) : gpio_clear(MOTOR_RIGHT_B);
    out[2] ? gpio_set(MOTOR_RIGHT_C) : gpio_clear(MOTOR_RIGHT_C);
    out[3] ? gpio_set(MOTOR_RIGHT_D) : gpio_clear(MOTOR_RIGHT_D);
}

static void left_motor_update(const uint8_t *out)
{
    // Left motor
    out[0] ? gpio_set(MOTOR_LEFT_A) : gpio_clear(MOTOR_LEFT_A);
    out[1] ? gpio_set(MOTOR_LEFT_B) : gpio_clear(MOTOR_LEFT_B);
    out[2] ? gpio_set(MOTOR_LEFT_C) : gpio_clear(MOTOR_LEFT_C);
    out[3] ? gpio_set(MOTOR_LEFT_D) : gpio_clear(MOTOR_LEFT_D);
}

void motor_stop(void)
{
	//Set to 0 each phase
	left_motor_update(step_halt);
    right_motor_update(step_halt);

	//Reset CMP
	MOTOR_RIGHT_TIMER->ARR = 0;
	MOTOR_LEFT_TIMER->ARR = 0;
}

uint8_t motor_position_reached(void)
{
    if(state_motor == POSITION_CONTROL && position_right_reached && position_left_reached){
        return POSITION_REACHED;
    }else{
        return POSITION_NOT_REACHED;
    }
}

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
	//reinit global variable
	counter_step_left = 0;
	counter_step_right = 0;

    position_right_reached = 0;
    position_left_reached = 0;

	//Set global variable with position to reach in step
	position_to_reach_left = position_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = -position_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;

	motor_set_speed(speed_r, speed_l);

	//flag for position control, will erase flag for speed control only
	state_motor = POSITION_CONTROL;
}

void motor_set_speed(float speed_r, float speed_l)
{
	int speed_r_step_s,speed_l_step_s;

    // Limit motor speed
    if (speed_r > MOTOR_SPEED_LIMIT) {
        speed_r = MOTOR_SPEED_LIMIT;
    } else if (speed_r < -MOTOR_SPEED_LIMIT) {
        speed_r = -MOTOR_SPEED_LIMIT;
    }
    if (speed_l > MOTOR_SPEED_LIMIT) {
        speed_l = MOTOR_SPEED_LIMIT;
    } else if (speed_l < -MOTOR_SPEED_LIMIT) {
        speed_l = -MOTOR_SPEED_LIMIT;
    }

    //transform the speed from cm/s into step/s
    speed_r_step_s = -speed_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	speed_l_step_s = speed_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;

    right_speed = speed_r_step_s;
    left_speed = speed_l_step_s;

    //flag for speed control
	state_motor = SPEED_CONTROL;

    // Timer reload takes 1 cycle, this is why -1
    MOTOR_RIGHT_TIMER->ARR = (TIMER_FREQ / abs(speed_r_step_s))-1;
    MOTOR_LEFT_TIMER->ARR = (TIMER_FREQ / abs(speed_l_step_s))-1;
}

// Motor right Interrupt Service Routine
void MOTOR_RIGHT_IRQHandler(void)
{
    /*
    *
    *   BEWARE !!
    *   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
    *
    *   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
    *   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
    *
    *   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
    *
    */

    static uint8_t i = 0;

    // Check if position is reached
    if ((abs(counter_step_right) > abs(position_to_reach_right)) && state_motor == POSITION_CONTROL) {
    	counter_step_right = 0;
        position_right_reached = 1;
    	right_motor_update(step_halt);
    	MOTOR_RIGHT_TIMER->ARR = 0;
    }
    else {
        if (right_speed > 0) {
            i = (i + 1) & 3;
            right_motor_update(step_table[i]);
            counter_step_right++;
        } else if (right_speed < 0) {
            i = (i - 1) & 3;
            right_motor_update(step_table[i]);
            counter_step_right--;
        } else {
            right_motor_update(step_halt);
        }
    }

    // Clear interrupt flag
    MOTOR_RIGHT_TIMER->SR &= ~TIM_SR_UIF;
    MOTOR_RIGHT_TIMER->SR;	// Read back in order to ensure the effective IF clearing
}

// Motor left Interrupt Service Routine
void MOTOR_LEFT_IRQHandler(void)
{

    /*
    *
    *   BEWARE !!
    *   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
    *
    *   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
    *   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
    *
    *   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
    *
    */

    static uint8_t i = 0;

    // Check if position is reached
    if ((abs(counter_step_left) > abs(position_to_reach_left))  && state_motor == POSITION_CONTROL) {
    	counter_step_left = 0;
        position_left_reached = 1;
    	left_motor_update(step_halt);
    	MOTOR_LEFT_TIMER->ARR = 0;
    }
    else {
        if (left_speed > 0) {
            i = (i + 1) & 3;
            left_motor_update(step_table[i]);
            counter_step_left++;
        } else if (left_speed < 0) {
            i = (i - 1) & 3;
            left_motor_update(step_table[i]);
            counter_step_left--;
        } else {
            left_motor_update(step_halt);
        }
    }

    // Clear interrupt flag
    MOTOR_LEFT_TIMER->SR &= ~TIM_SR_UIF;
    MOTOR_LEFT_TIMER->SR;	// Read back in order to ensure the effective IF clearing
}

