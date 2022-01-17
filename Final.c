#include "manage.h"

int state = 0; // 0: test, 1: start algorithm
int play_pause_program = 1, printFlag = 0;

PWM_t pwm_RC, pwm_UT, pwm_Left_H, pwm_Left_L, pwm_Right_H, pwm_Right_L;
TIM_t tim4_UT;

// sensor variables	--------------//
int IR_left, IR_right;
int UT_distance;
float RC_angle = 0.5;
//--------------------------------//


// other variables	--------------//
int del =0, ris, fal, EOCcnt = 0, check = 1, park_state = 0, UT_prev, cnt = 0, TIM9cnt = 0;
int BTflag = 1, laneFlag = 1;
char direction = 's';
uint8_t BTdata;
//--------------------------------//


void lane_tracker(void)
{	
	if(cnt < 500)
	{
		PWM_pulsewidth_ms(&pwm_RC, RC_front);
	}
	else if(UT_distance > 20 || UT_distance < 0)
	{	
		drive(direction);
	}
	else
	{
		drive('X');  // stop vehicle
		state = 2;   // move to state 2(parking)
		park_state = 0;
		cnt = 0;
	}
	
}


void park_vehicle(void)
{
	switch(park_state)
	{
		case 0: // rotate UT
			if(cnt < 1200 && cnt > 500)
			{
				PWM_pulsewidth_ms(&pwm_RC, RC_left);
				UT_prev = UT_distance;
				park_state = 1;
			}
			break;
		
		case 1: // find edge
			if( abs(UT_distance - UT_prev) < 15)
			{
				UT_prev = UT_distance;
				drive('b');
				cnt = 0;
			}
			else if(cnt > 315)
			{
				park_state = 2;
				cnt = 0;
			}
			break;
			
		case 2: // turn 90 degrees
				if(cnt < 700)
				{
					DC_motor(0,-0.8);
				}
				else if(cnt < 1300)
				{
					drive('x');
					PWM_pulsewidth_ms(&pwm_RC, RC_behind);
				}
				else
				{
					park_state = 3;
					cnt = 0;
				}
				break;
			
		case 3: // go inside the parking lot
			if(UT_distance > 15)
			{
				drive('b');
			}
			else
			{
				drive('x');
				park_state = 4;
				cnt = 0;
			}
			break;
			
		case 4: // 90 degree again
			if(cnt < 800)
				DC_motor(0.8,-0.8);
			else
			{
				drive('x');
				state = 0;
				park_state = 0;
			}
			break;
	}
}



void sensor_testing(void)
{

	PWM_pulsewidth_ms(&pwm_RC, RC_behind);
	
}


void drive(char type)
{
	if(type == 'l' || type == 'L')
	{
		DC_motor(0.4,1);
	}
	else if(type == 's')
	{
		DC_motor(1,1);
	}
	else if(type == 'S')
	{
		DC_motor(0.9,0.9);
	}
	else if (type == 'r' || type == 'R')
	{
		DC_motor(1,0.3);
	}
	else if(type == 'x' || type == 'X')
	{
		DC_motor(0,0);
	}
	else if(type == 'b' || type == 'B')
	{
		DC_motor(-0.9, -0.9);
	}
}


void setup_final(void)
{
	RCC_PLL_init();	
	SysTick_init();
	EXTI_Init(2,13,1,0);
	UART2_init(9600, POL);
	
	BT_init(9600); 			  // BlueTooth
	
	RC_motor_setup();		  // RC motor
	
	UT_setup();				  // Ultrasonic Sensor
	
	DC_motor_setup();		  // Wheels
	
	IR_setup();			      // IR sensor
	
	TIM9_init(1);			  // event handler
	
}

void RC_motor_setup(void)
{
	PWM_init(&pwm_RC, GPIOA, 6); 
	PWM_period_ms(&pwm_RC,25);
	PWM_pulsewidth_ms(&pwm_RC, RC_angle);
}

void DC_motor_setup(void)
{
	PWM_init(	&pwm_Left_H		, 	GPIOA, 8	);
	PWM_init(	&pwm_Left_L		, 	GPIOA, 9	);
	PWM_init(	&pwm_Right_H	, 	GPIOA, 10	);
	PWM_init(	&pwm_Right_L	, 	GPIOA, 11	);
	
	PWM_period_ms(	&pwm_Left_H		,	1);
	PWM_period_ms(	&pwm_Left_L		,	1);
	PWM_period_ms(	&pwm_Right_H	,	1);
	PWM_period_ms(	&pwm_Right_L	,	1);
	
	PWM_duty(	&pwm_Left_H		, 0);
	PWM_duty(	&pwm_Left_L		, 0);
	PWM_duty(	&pwm_Right_H	, 0);
	PWM_duty(	&pwm_Right_L	, 0);
}

void DC_motor(float left, float right)
{
	if(left > 0)
	{
		PWM_duty(	&pwm_Left_H		, left);
		PWM_duty(	&pwm_Left_L		, 0);
	}
	else if(left < 0)
	{
		PWM_duty(	&pwm_Left_H		, 0);
		PWM_duty(	&pwm_Left_L		, -left);
	}
	else
	{
		PWM_duty(	&pwm_Left_H		, 0);
		PWM_duty(	&pwm_Left_L		, 0);
	}
	
	if(right > 0)
	{
		PWM_duty(	&pwm_Right_H	, right);
		PWM_duty(	&pwm_Right_L	, 0);
	}
	else if(right < 0)
	{
		PWM_duty(	&pwm_Right_H	, 0);
		PWM_duty(	&pwm_Right_L	, -right);
	}
	else
	{
		PWM_duty(	&pwm_Right_H	, 0);
		PWM_duty(	&pwm_Right_L	, 0);
	}
}


void IR_setup(void)
{
	TIM5_init(1);

	GPIO_init(GPIOA, 0, ANALOG);
	GPIO_init(GPIOA, 1, ANALOG);

	ADC_TIM5_TRG_init(GPIOA, 0, GPIOA, 1);

	NVIC_SetPriority(ADC_IRQn, 9); 	   
	
	NVIC_EnableIRQ(ADC_IRQn);

}

void UT_setup(void)
{
	// trigger
	PWM_init(&pwm_UT,GPIOC,8); 
	PWM_period_ms(&pwm_UT, 50);
	PWM_pulsewidth_ms(&pwm_UT,10);
	
	// echo
	CAP_init(&tim4_UT, GPIOB,8); 
	TIM_period_us(4,10);	
	NVIC_SetPriority(TIM4_IRQn, 0);							// Set the priority of TIM2 interrupt request
	NVIC_EnableIRQ(TIM4_IRQn);
	
}





void EXTI15_10_IRQHandler() // pause button
{
	if((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13)
	{
		if(state>0)
			state = 0;
		else
		{
			state = 1;
			cnt = 0;
		}
		play_pause_program = !play_pause_program;
		
	}
	EXTI->PR |= EXTI_PR_PR13;
}


void TIM4_IRQHandler(void)	// UT read
{	
	if(TIM4->SR & TIM_SR_UIF)
	{ 
		del++;//  <-- overflow counting
		TIM4->SR &= ~TIM_SR_UIF; 							// clear update interrupt flag
	}
	if((TIM4->SR & TIM_SR_CC3IF) != 0)	// rising edge
	{ 
		ris  = TIM4->CCR3;
		del = 0;
		TIM4->SR &= ~TIM_SR_CC3IF;
	}
	if((TIM4->SR & TIM_SR_CC4IF) != 0)	// falling edge
	{
		fal  = TIM4->CCR4;
		UT_distance = (double)(fal - ris + del*TIM4->ARR);
		UT_distance = UT_distance * (TIM4->PSC + 1)/84;
		UT_distance = UT_distance/58.0;
		del = 0;
		TIM4->SR &= ~TIM_SR_CC4IF;
	}
}

void TIM1_BRK_TIM9_IRQHandler()	// Tim9
{
	if(TIM9->SR & TIM_SR_UIF)
	{
		cnt++;
		TIM9cnt = TIM9cnt > 1000 ? 0 : TIM9cnt + 1;
		if(TIM9cnt == 0)
			printFlag = 1;

		TIM9->SR &= ~1;
	}
}

void ADC_IRQHandler(void){
   if((ADC1->SR & ADC_SR_OVR) == ADC_SR_OVR){
     ADC1->SR&=~ADC_SR_OVR;
   }

   if(ADC1->SR & ADC_SR_JEOC )
	 {
		 
		 IR_left = ADC1->JDR1;
		 IR_left = IR_left < IR_THRESH ? 1: 0;
		 
		 
		 IR_right = ADC1->JDR2;
		 IR_right = IR_right < IR_THRESH ? 1: 0;
		 
		 if(state == 1)
		 {
			 direction = !IR_left && IR_right ? 'r' : (IR_left && !IR_right ? 'l' : 's');
			 drive(direction);
		 }
		  
		 ADC1->SR &= ~(ADC_SR_JEOC);
	 }
}

void wait_BT(void)
{
	drive('x');

}

void USART1_IRQHandler(){
   if(USART1->SR & USART_SR_RXNE)
	 {
		 //Bluetooth connection
		 BTdata = BT_read();
		
		 if(BTdata == (uint8_t)'s')
		 {
			 BT_write((uint8_t*) "BT sent ",8);
			 BT_write(&BTdata,1);
			 BT_write((uint8_t*) "\r\n",2);
			 state = 1;
			 cnt = 0;
		 }
		 else if(BTdata == (uint8_t)'p')
		 {
			 BT_write((uint8_t*) "BT sent ",8);
			 BT_write(&BTdata,1);
			 BT_write((uint8_t*) "\r\n",2);
			 state = 2;
		 }
		 else if(BTdata == (uint8_t)'q')
		 {
			 BT_write((uint8_t*) "BT sent ",8);
			 BT_write(&BTdata,1);
			 BT_write((uint8_t*) "\r\n",2);
			 state = 0;
		 }
		 
		 

      USART1->SR &= ~USART_SR_RXNE; //clear bit
   }
}

