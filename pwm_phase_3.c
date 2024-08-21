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
#include "pwm_phase_3.h"

//define Constants
#define CLK_FREQ 64
#define ARR_VALUE 500

//functions
void PWMfreq(double freq) //freq[kHz]
{
    uint16_t PSC = 0;
    __HAL_TIM_SET_AUTORELOAD(&htim1, ARR_VALUE - 1); //set ARR to 500-1

    PSC = (uint16_t)round(CLK_FREQ*2/ freq);
    __HAL_TIM_SET_PRESCALER(&htim1, PSC - 1);
}

void PWMduty(double sinfreq)
{
    uint16_t CRR1 = 0;
    uint16_t CRR2 = 0;
    uint16_t CRR3 = 0;
    double omega = 2 * M_PI * sinfreq;
    
    // Phase A (0 degree)
    CRR1 = (uint16_t)(ARR_VALUE*0.45 * (sin(omega * t)) + ARR_VALUE*0.5 - 1);

    // Phase B (120 degree phase shift)
    CRR2 = (uint16_t)(ARR_VALUE*0.45 * (sin(omega * t - (2 * M_PI / 3))) + ARR_VALUE*0.5 - 1);

    // Phase C (240 degree phase shift)
    CRR3 = (uint16_t)(ARR_VALUE*0.45 * (sin(omega * t + (2 * M_PI / 3))) + ARR_VALUE*0.5 - 1);


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