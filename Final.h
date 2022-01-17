#ifndef __FINAL_H__
#define __FINAL_H__
#include "manage.h"
#define IR_THRESH	  1000
#define RC_front		0.5
#define RC_left			1.5	
#define RC_behind		2.5

extern int state;
extern int play_pause_program, printFlag;

extern PWM_t pwm_RC, pwm_UT, pwm_Left_H, pwm_Left_L, pwm_Right_H, pwm_Right_L;
extern TIM_t tim4_UT;

// sensor variables	--------------//
extern int IR_left, IR_right;
extern int UT_distance;
extern float RC_angle;
//--------------------------------//

extern int del, ris, fal, EOCcnt, check, park_state, UT_prev, cnt, TIM9cnt;
extern char direction;



//*****************************************//
//**		main operation
//*****************************************//

void lane_tracker(void);

void park_vehicle(void);

void sensor_testing(void);



//*****************************************//
//**		sub operation
//*****************************************//

void drive(char type);








//*****************************************//
//**		settings
//*****************************************//

void setup_final(void);

void RC_motor_setup(void);

void DC_motor_setup(void);

void DC_motor(float left, float right);

void IR_setup(void);

void UT_setup(void);

void EXTI15_10_IRQHandler();

void TIM4_IRQHandler(void);

void TIM1_BRK_TIM9_IRQHandler();

void ADC_IRQHandler(void);

void wait_BT(void);

void USART1_IRQHandler();

























#endif