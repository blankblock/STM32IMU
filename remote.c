

#include "config.h"
#include "remote.h"

#define RC_PWM_IC_MAX			1940
#define RC_PWM_IC_MIN			1100
#define RC_PWM_IC_MID			1520

#define CAP31_PIN				(GPIOB->IDR & GPIO_Pin_4)		//PB4 --> Timer3 ch1
#define CAP31_POL_RISING		(TIM3->CCER &= ~TIM_CCER_CC1P)
#define CAP31_POL_FALLING		(TIM3->CCER |= TIM_CCER_CC1P)
#define CAP32_PIN				(GPIOB->IDR & GPIO_Pin_5)		//PB5 --> Timer3 ch2
#define CAP32_POL_RISING		(TIM3->CCER &= ~TIM_CCER_CC2P)
#define CAP32_POL_FALLING		(TIM3->CCER |= TIM_CCER_CC2P)

#define CAP33_PIN				(GPIOB->IDR & GPIO_Pin_0)		//PB0 --> Timer3 ch3
#define CAP33_POL_RISING		(TIM3->CCER &= ~TIM_CCER_CC3P)
#define CAP33_POL_FALLING		(TIM3->CCER |= TIM_CCER_CC3P)
#define CAP34_PIN				(GPIOB->IDR & GPIO_Pin_1)		//PB1 --> Timer3 ch4
#define CAP34_POL_RISING		(TIM3->CCER &= ~TIM_CCER_CC4P)
#define CAP34_POL_FALLING		(TIM3->CCER |= TIM_CCER_CC4P)

volatile uint16_t rcPWM_IC[4];
TIM_ICInitTypeDef  TIM_ICInitStructure;
TIM_ICInitTypeDef RC_PWM_CH1_ICStructure;
TIM_ICInitTypeDef RC_PWM_CH2_ICStructure;
TIM_ICInitTypeDef RC_PWM_CH3_ICStructure;
TIM_ICInitTypeDef RC_PWM_CH4_ICStructure;
volatile uint16_t rcPwm_ch1_up = 0;
volatile uint16_t rcPwm_ch2_up = 0;
volatile uint16_t rcPwm_ch3_up = 0;
volatile uint16_t rcPwm_ch4_up = 0;
volatile float rcInput[4];

void TIM3_IRQHandler(void)
{
	uint16_t tim_cnt = TIM3->CNT;
	uint16_t tim_capture = 0;
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		/* Clear TIM3 Capture compare interrupt pending bit */
		if(CAP31_PIN)
		{
			CAP31_POL_FALLING;
			rcPwm_ch1_up = TIM3->CCR1;
		}else
		{
			CAP31_POL_RISING;
			rcPWM_IC[0] = TIM3->CCR1 - rcPwm_ch1_up;
		}
	}
	if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		/* Clear TIM3 Capture compare interrupt pending bit */
		if(CAP32_PIN)
		{
			CAP32_POL_FALLING;
			rcPwm_ch2_up = TIM3->CCR2;
		}else
		{
			CAP32_POL_RISING;
			rcPWM_IC[1] = TIM3->CCR2 - rcPwm_ch2_up;
		}
	}
	if((TIM3->SR& TIM_IT_CC3)&&(TIM3->DIER & TIM_IT_CC3))	//TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		/* Clear TIM3 Capture compare interrupt pending bit */
		if(CAP33_PIN)
		{
			CAP33_POL_FALLING;
			rcPwm_ch3_up = TIM3->CCR3;
		}else
		{
			CAP33_POL_RISING;
			rcPWM_IC[2] = TIM3->CCR3 - rcPwm_ch3_up;
		}
	}
	if((TIM3->SR& TIM_IT_CC4)&&(TIM3->DIER & TIM_IT_CC4))
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		/* Clear TIM3 Capture compare interrupt pending bit */
		if(CAP34_PIN)
		{
			CAP34_POL_FALLING;
			rcPwm_ch4_up = TIM3->CCR4;
		}else
		{
			CAP34_POL_RISING;
			rcPWM_IC[3] = TIM3->CCR4 - rcPwm_ch4_up;
			Remote_Update();
		}
		
	}
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
         TIM_ClearITPendingBit(TIM3, TIM_IT_Update);    // Counter overflow, reset interrupt
         //if(T2_Overrun<127){T2_Overrun++;}
    }

}

void Remote_Update(void)
{
	int i = 0;
	for(i=0;i<4;i++)
	{
//		if(rcPWM_IC[i] > RC_PWM_IC_MAX) rcInput[i] = 1.0;
//		else if(rcPWM_IC[i] < RC_PWM_IC_MIN) rcInput[i] = -1.0;
		if(rcPWM_IC[i] < RC_PWM_IC_MAX && rcPWM_IC[i] > RC_PWM_IC_MIN )
		{
			//If the input pwm is in middle range
			if(rcPWM_IC[i] < RC_PWM_IC_MID + 20 && rcPWM_IC[i] > RC_PWM_IC_MID - 20) 
			{
				rcInput[i] = 0.0;
			}
			else 
			{
				rcInput[i] = (float)(rcPWM_IC[i] - RC_PWM_IC_MID) / (float)(RC_PWM_IC_MAX - RC_PWM_IC_MID);
			}
		}
	}
}

void InitICStructure(TIM_TypeDef* TIMx, TIM_ICInitTypeDef *ICInitType,uint16_t channel, uint16_t Polarity)
{
	ICInitType->TIM_Channel = channel;
	ICInitType->TIM_ICFilter = 0x0;
	ICInitType->TIM_ICPolarity = Polarity;
	ICInitType->TIM_ICPrescaler = TIM_ICPSC_DIV1;
	ICInitType->TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIMx, ICInitType);
	//TIM_PWMIConfig(TIMx, ICInitType);
}
void Remote_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM3_TimeBaseStructure;
	TIM_DeInit(TIM3);
	  /* Time Base configuration */
	TIM3_TimeBaseStructure.TIM_Prescaler = 71;		//1 usec for 72MHz clock
	TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM3_TimeBaseStructure.TIM_Period = 65535;
	TIM3_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM3_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM3_TimeBaseStructure);
	  /* TIM3 configuration: Input Capture mode ---------------------
	 The external signal is connected to TIM3 CH1 and CH2 pin (PA.06 & 07)  
	 The Rising edge is used as active edge,
	 The TIM3 CCR2 is used to compute the frequency value 
	------------------------------------------------------------ */
	InitICStructure(TIM3, &RC_PWM_CH1_ICStructure, TIM_Channel_1, TIM_ICPolarity_Rising);
	InitICStructure(TIM3, &RC_PWM_CH2_ICStructure, TIM_Channel_2, TIM_ICPolarity_Rising);
	InitICStructure(TIM3, &RC_PWM_CH3_ICStructure, TIM_Channel_3, TIM_ICPolarity_Rising);
	InitICStructure(TIM3, &RC_PWM_CH4_ICStructure, TIM_Channel_4, TIM_ICPolarity_Rising);

	/* TIM enable counter */
	TIM_Cmd(TIM3, ENABLE);
	
	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);

}