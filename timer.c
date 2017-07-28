/**
 * @file timer.c
 * @brief
 * @author KHY 2014/3/3
 */

#include "timer.h"
#include "config.h"

#define MOTOR_PWM_FREQ		(8000)

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t timerCnt;
static uint8_t timer2msCnt;
static uint8_t timer10msCnt;
static uint8_t timer50msCnt = 0;
static uint16_t timer100msCnt = 0;
static uint16_t timer200msCnt = 0;

volatile uint8_t task_2msRdy = 0;
volatile uint8_t task_10msRdy = 0;
volatile uint8_t task_50msRdy = 0;
volatile uint8_t task_100msRdy = FALSE;
volatile uint8_t task_200msRdy = FALSE;

uint16_t TimerPeriod = 0;
TimerType m_Timer;


/**
  * @brief  Timer2 interrupt service routine called every 10us 
  * @param  None
  * @retval None
*/
void TIM1_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
		timerCnt++;
		m_Timer.ms++;
		timer2msCnt++;
		timer10msCnt++;
		timer50msCnt++;
		timer100msCnt++;
		timer200msCnt++;
		if(timer2msCnt>5)
		{
			timer2msCnt = 0;
			task_2msRdy = TRUE;
		}
		if(timer10msCnt>9)
		{
			task_10msRdy = TRUE;
			timer10msCnt = 0;
		}
		if(timer50msCnt > 49)
		{
			task_50msRdy = TRUE;
			timer50msCnt = 0;
		}
		if(timer100msCnt > 999)
		{
#ifdef OMOCOPTER
		//LED_TOGGLE();
#else
		//LED_TOGGLE_G();
#endif
			task_100msRdy = TRUE;
			timer100msCnt = 0;
		}
		if(timer200msCnt>199)
		{
			task_200msRdy = TRUE;
			timer200msCnt = 0;
		}
		if(timerCnt > 499)
		{
			timerCnt = 0;
		}
		if(m_Timer.ms>999)
		{
			m_Timer.ms = 0;
			m_Timer.ss++;
			if(m_Timer.ss>59){
				m_Timer.ss = 0;
				m_Timer.mm++;
			}
		}
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}



/**
 * @fn TIMER_Configuration(PWM_Frequency pwmPeriod)
 * @brief Initialize all timer relate peripherals including PWM generation, Input capture, General Timer
 * @param [in] pwmFrequency pwm Frequency set
 */
void TIMER_Configuration(void)
{
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  int motorPwmPeriod = 0;
  m_Timer.ms = 0;
  m_Timer.ss = 0;
  m_Timer.mm = 0;

	/* TIM1 Configuration ---------------------------------------------------
   Generate 7 PWM signals with 4 different duty cycles:
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
   and Connectivity line devices and to 24 MHz for Low-Density Value line and
   Medium-Density Value line devices
   
   The objective is to generate 7 PWM signal at 17.57 KHz:
     - TIM1_Period = (SystemCoreClock / 17570) - 1
   The channel 1 and channel 1N duty cycle is set to 50%
   The channel 2 and channel 2N duty cycle is set to 37.5%
   The channel 3 and channel 3N duty cycle is set to 25%
   The channel 4 duty cycle is set to 12.5%
   The Timer pulse is calculated as follows:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
  ----------------------------------------------------------------------- */
  
  
  /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
	TIM_DeInit(TIM1);
	// Timer2 Clock Initialization
	TIM_TimeBaseStructure.TIM_Prescaler = TIM1_PRESCALER;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM1_INT_PERIOD_US;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
		// Clear update flag
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
		// Clear update interrupt bit
	TIM_ClearITPendingBit(TIM1,TIM_FLAG_Update);
		// Enable update interrupt
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
		// Enable timer counting
	TIM_Cmd(TIM1,ENABLE);
	
	TIM_DeInit(TIM2);
	  TimerPeriod = (SystemCoreClock / MOTOR_PWM_FREQ ) - 1;
	  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			///Polarity Low: Signal Low->High, Polarity High: Signal High->Low
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	/* TIM1 counter enable */
	TIM_Cmd(TIM2, ENABLE);
	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
	
	// Timer4 Clock Initialization
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Prescaler = TIM2_PRESCALER;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM4_INT_PERIOD_US;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		// Clear update flag
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
		// Clear update interrupt bit
	TIM_ClearITPendingBit(TIM4,TIM_FLAG_Update);
		// Enable update interrupt
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
		// Enable timer counting
	TIM_Cmd(TIM4,ENABLE);
}
