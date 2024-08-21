/*
    Created on: 2024.08.20
        Author: Seongjin Hur
    
      function: Make Three-Phase PWM signals with STM32 MCU
                Using TIM1 CH1, CH2, CH3 for each phase

*/

#ifndef __PWM_PHASE_3_H__
#define __PWM_PHASE_3_H__

//include header
#include "math.h"

//define Constant
#ifndef M_PI 
#define M_PI 3.14159265358979
#endif

//function prototype

//PWM configuration
void PWMfreq(double PWMfreq);
void PWMduty(double sinefreq);

//Set PWM signals
void setup_complementary_pwm(void);


#endif