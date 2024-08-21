/*
    Created on: 2024.08.20
        Author: Seongjin Hur
    
      function: Make Three-Phase PWM signals with STM32 MCU
                Using TIM1 CH1, CH2, CH3 for each phase

     variables (should be declared in main.c)
             t: time variable for the sine wave
            Ts: Sampling Period (1/freq)
                
*/

//include header
#include "pwm.h"

//define Constants
#define CLK_FREQ 64
#define ARR_VALUE 500

//functions
void PWMfreq(double freq) //freq[kHz], upto 84kHz
{
    int PSC = 0;
    __HAL_TIM_SET_AUTORELOAD(&htim1, ARR_VALUE - 1); //set ARR to 500-1

    PSC = (int)round(CLK_FREQ*2/ freq);
    __HAL_TIM_SET_PRESCALER(&htim1, PSC - 1);
}

void PWMduty(double sinfreq)
{
    int CRR1 = 0, CRR2 = 0, CRR3 = 0;
    double omega = 2 * M_PI * sinfreq;
    
    // Phase A (0 degree)
    CRR1 = (int)(250 * (sin(omega * t) + 1));

    // Phase B (120 degree phase shift)
    CRR2 = (int)(250 * (sin(omega * t - (2 * M_PI / 3)) + 1));

    // Phase C (240 degree phase shift)
    CRR3 = (int)(250 * (sin(omega * t + (2 * M_PI / 3)) + 1));


    // Update the PWM compare registers for each phase
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CRR1);  // Phase A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, CRR2);  // Phase B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, CRR3);  // Phase C

    t = t + Ts;
}

void setup_complementary_pwm()
{
    // Assuming TIM1 is already configured for PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}