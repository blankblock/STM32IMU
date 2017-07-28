
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "utils.h"
#include "matrix.h"
#include "i2c.h"
#include "hmc5883.h"
#include "mpu9150.h"
#include "bmi055_imu.h"
#include "lps25h.h"
#include "l3g4200.h"
#include "uart.h"
#include "timer.h"
#include "remote.h"
#include "utill.h"
#include <math.h>
#include "kalman.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern char TxBuffer1[USART1_Tx_BufferSize];
extern volatile uint8_t task_2msRdy;
extern volatile uint8_t task_10msRdy;
extern volatile uint8_t task_50msRdy;
extern volatile uint8_t task_100msRdy;
extern volatile uint8_t task_200msRdy;


extern MPU9150_Typedef mpu9150;
extern LPS25_Typedef lps25;
extern stGyro_Typedef stGyro;
extern char RxBuffer1[USART1_Rx_BufferSize];
extern uint8_t Rx1_Ready;
extern HMC5883L_Typedef compass;
extern volatile uint16_t rcPWM_IC[4];
extern volatile float rcInput[4];

static uint16_t CompassRead_TimerCnt = 0;
static uint16_t PressureRead_TimerCnt = 0;

float dt;					//timer interval
float roll, pitch, yaw;		// Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer
float gyroAngleX, gyroAngleY, gyroAngleZ;	//Angle calculated using gyro only
float gyroRateX, gyroRateY, gyroRateZ;
float compAngleX, compAngleY, compAngleZ;	//Angle calculated using a complementary filter
float kalAngleX, kalAngleY, kalAngleZ;		//Angle calculated using a Kalman filter

KalmanFilterTypedef kalmanX;
KalmanFilterTypedef kalmanY;
KalmanFilterTypedef kalmanZ;

float outputThro, outputRoll, outputPitch, outputYaw;
float outputFL, outputFR, outputRL, outputRR;
PID_Gain_Typedef pidPitch;
PID_Gain_Typedef pidRoll;
PID_Gain_Typedef pidYaw;

uint8_t displayMode;
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)


struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

char rx_new = 0;
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from their
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};


/* Private functions ---------------------------------------------------------*/
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
void GPIO_Configuration(void);
void RCC_Configuration(void);
void NVIC_Configuration(void);
void TIM4_IRQHandler(void);
void upatePitchRoll(void);
void updateYaw(void);
void PID_Init(PID_Gain_Typedef *gainType, float p, float i, float d);

void updatePitchRoll()
{
	roll = atan2(mpu9150.accelData[1], mpu9150.accelData[2]) * RAD_TO_DEG;
	pitch = atan(-mpu9150.accelData[0] 
				 / sqrt(mpu9150.accelData[0]*mpu9150.accelData[0] +
						mpu9150.accelData[2]*mpu9150.accelData[2])) * RAD_TO_DEG;
}
void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
//  magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
//  magZ *= -1;
//
//  magX *= magGain[0];
//  magY *= magGain[1];
//  magZ *= magGain[2];
//
//  magX -= magOffset[0];
//  magY -= magOffset[1];
//  magZ -= magOffset[2];

  float rollAngle = kalAngleX * DEG_TO_RAD;
  float pitchAngle = kalAngleY * DEG_TO_RAD;

  float Bfy = compass.magGauss[2] * sin(rollAngle) - compass.magGauss[1] * cos(rollAngle);
  float Bfx = compass.magGauss[0] * cos(pitchAngle) 
	  	+ compass.magGauss[1] * sin(pitchAngle) * sin(rollAngle) 
		+ compass.magGauss[2] * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
  yaw *= -1;
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) 
	{
		/* Update all the IMU values */
		stGyro_ReadGyro();
		mpu9150_ReadAccel();
		/* Roll and pitch estimation */
		updatePitchRoll();
		gyroRateX = stGyro.degS[0];
		gyroRateY = stGyro.degS[1];
		gyroRateZ = stGyro.degS[2];
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanSetAngle(&kalmanX, roll);
			compAngleX = roll;
			kalAngleX = roll;
			gyroAngleX = roll;
		} else
			kalAngleX = kalmanGetAngle(&kalmanX, roll, gyroRateX, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleX) > 90)
			gyroRateY = -gyroRateY; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleY = kalmanGetAngle(&kalmanY, pitch, gyroRateY, dt);
		/*
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanSetAngle(pitch);
			compAngleY = pitch;
			kalAngleY = pitch;
			gyroAngleY = pitch;
		} else
			kalAngleY = kalmanGetAngle(pitch, stGyro.degS[1], dt); // Calculate the angle using a Kalman filter
		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleX = kalmanGetAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
		*/
		
		/* Yaw estimation */
		updateYaw();
		// This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
		if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
			kalmanSetAngle(&kalmanZ, yaw);
			compAngleZ = yaw;
			kalAngleZ = yaw;
			gyroAngleZ = yaw;
		} else
			kalAngleZ = kalmanGetAngle(&kalmanZ, yaw, gyroRateZ, dt); // Calculate the angle using a Kalman filter

		if(++CompassRead_TimerCnt>TIMER_INTERVAL_COMPASS)
		{
			LED_TOGGLE();
			HMC5883_ReadData();
			CompassRead_TimerCnt = 0;
		}
		if(++PressureRead_TimerCnt>TIMER_INTERVAL_PRESSURE)
		{
			lps25_ReadPress();
			mpu9150_ReadTemp();
			PressureRead_TimerCnt = 0;
		}
		outputThro = (rcInput[2] + 1.0)/2.0;
		outputPitch = rcInput[1]*0.4;
		outputRoll = rcInput[0]*0.4;
		outputYaw = rcInput[3];
		outputFL = outputThro + outputRoll + outputPitch - kalAngleX * pidRoll.Kp + kalAngleY * pidPitch.Kp;
		outputFR = outputThro - outputRoll + outputPitch + kalAngleX * pidRoll.Kp + kalAngleY * pidPitch.Kp;
		outputRL = outputThro + outputRoll - outputPitch - kalAngleX * pidRoll.Kp - kalAngleY * pidPitch.Kp;
		outputRR = outputThro - outputRoll - outputPitch + kalAngleX * pidRoll.Kp - kalAngleY * pidPitch.Kp;
		if(outputFL  < 0.0) outputFL = 0.0;
		if(outputFR  < 0.0) outputFR = 0.0;
		if(outputRL  < 0.0) outputRL = 0.0;
		if(outputRR  < 0.0) outputRR = 0.0;
		
		MOTOR_CCR_FL = (uint16_t)(outputFL * PWM_OUT_MAX);
		MOTOR_CCR_FR = (uint16_t)(outputFR * PWM_OUT_MAX);
		MOTOR_CCR_RL = (uint16_t)(outputRL * PWM_OUT_MAX);
		MOTOR_CCR_RR = (uint16_t)(outputRR * PWM_OUT_MAX);
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}

void PID_Init(PID_Gain_Typedef *gainType, float p, float i, float d)
{
	gainType->Kp = p;
	gainType->Ki = i;
	gainType->Kd = d;
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	int result;
	displayMode = 0;
	AFIO->MAPR|= (0x02<<24);	//010: JTAG-DP Disabled and SW-DP Enabled
	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();
	I2C_Configure();
	USART_Config(USART_115200bps);
	outputThro = 0.0;
	outputPitch = 0.0;
	outputRoll = 0.0;
	outputYaw = 0.0;
	outputFL = 0.0;
	outputFR = 0.0;
	outputRL = 0.0;
	outputRR = 0.0;
	/* Add your application code here
	*/
	Delay_us(60000);
	HMC5883_Config();
	Delay_us(6000);
	mpu9150_Config();
	Delay_us(6000);
	lps25_Config();
	Delay_us(6000);
	stGyro_Config();
	mpu9150_SelfTest();
	TIMER_Configuration();
	Remote_Config();
	PID_Init(&pidPitch, 0.05, 0.002, 0.0);
	PID_Init(&pidRoll, 0.05, 0.002, 0.0);
	PID_Init(&pidYaw, 0.05, 0.002, 0.0);
	kalmanInit(&kalmanX);
	kalmanInit(&kalmanY);
	kalmanInit(&kalmanZ);
	dt = (1.0/GYRO_ACCEL_UPDATE_HZ);		//timer interval is fixed at 500hz update rate led to 0.002 sec
	/* Infinite loop */
	while (1)
	{
		if(Rx1_Ready == TRUE)
		{
			switch(RxBuffer1[0])
			{
				case 'q':
					sprintf(TxBuffer1, "Press (g)yro, (a)ccel, (c)ompass, (b)arometer\r\n");
					USART_DMA_Send(TxBuffer1);
					sprintf(TxBuffer1, "Press (q)uit for exit at anytime.\r\n");
					USART_DMA_Send(TxBuffer1);
					sprintf(TxBuffer1, "Temp: %f\r\n", mpu9150.f_Temperature);
					USART_DMA_Send(TxBuffer1);
					displayMode = 0;
					break;
				case 'g':
					displayMode = 1;
					break;
				case 'a':
					displayMode = 2;
					break;
				case 'c':
					displayMode = 3;
					break;
				case 'b':
					displayMode = 4;
					break;
				case 'i':
					mpu9150_Ident();
					displayMode = 0;
					break;
				case 't':
					mpu9150_SelfTest();
					displayMode = 0;
					break;
				case 's':
					displayMode = 5;
					break;
				case 'k':
					displayMode = 6;
					break;
				case 'p':
					displayMode = 7;
				break;
			}
			Rx1_Ready = FALSE;
		}
		if(task_50msRdy == TRUE)
		{
			switch(displayMode)
			{
			case 0:

				break;
			case 1:
				sprintf(TxBuffer1, "$GYR,%.3f,%.3f,%.3f,\r\n", 
						mpu9150.gyroData[0], mpu9150.gyroData[1], mpu9150.gyroData[2]);
				USART_DMA_Send(TxBuffer1);
				break;
			case 2:
				sprintf(TxBuffer1, "$INV,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,\r\n", 
						mpu9150.accelData[0],mpu9150.accelData[1],mpu9150.accelData[2],
						mpu9150.gyroData[0],mpu9150.gyroData[1],mpu9150.gyroData[2]);
				USART_DMA_Send(TxBuffer1);
				break;
			case 3:
				sprintf(TxBuffer1, "$MAG,%.3f,%.3f,%.3f,%.3f,\r\n", 
						compass.magGauss[0], compass.magGauss[1], compass.magGauss[2], compass.heading);
				USART_DMA_Send(TxBuffer1);
				break;
			case 4:
				sprintf(TxBuffer1, "$BAR,%.3f,%.3f,\r\n", lps25.f_hPa, lps25.f_DegC);
				USART_DMA_Send(TxBuffer1);
				break;
			case 5:
				sprintf(TxBuffer1, "$GYR,%.3f,%.3f,%.3f,%d,\r\n",
					stGyro.degS[0], stGyro.degS[1], stGyro.degS[2], stGyro.temp_C);
				USART_DMA_Send(TxBuffer1);
				break;
			case 6:
				sprintf(TxBuffer1, "$KAL,%.3f,%.3f,%.3f,%.3f,%.3f,\r\n", 
						kalAngleX, roll, 
						kalAngleY, pitch,
						kalAngleZ);
				USART_DMA_Send(TxBuffer1);
				break;
			case 7:
				sprintf(TxBuffer1, "$PWM,%.3f,%.3f,%.3f,%.3f,\r\n", 
						rcInput[0],
						rcInput[1],
						rcInput[2],
						rcInput[3]);
				USART_DMA_Send(TxBuffer1);
				break;
		
			}
			
			task_50msRdy = false;
		}
		if(task_200msRdy == TRUE)
		{
			task_200msRdy = false;
		}
	}
}

/**
 * @fn RCC_Configuration(void)
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
	/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* TIM1, GPIOA, GPIOB, GPIOE and AFIO clocks enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOC , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#ifdef USE_DCMD_V120
	///Alternate functions are used for DCMD V1.20 and need to provie clocks.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

/**
 * @fn GPIO_Configuration(void)
  * @brief  Configure the TIM1 Pins.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//Configure for LED pins
#ifdef OMOCOPTER
	GPIO_InitStructure.GPIO_Pin = LED_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED_Port, &GPIO_InitStructure);
#elif defined	OMOCOPTER_V2
	GPIO_InitStructure.GPIO_Pin = LED_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED_Port, &GPIO_InitStructure);
#else
	GPIO_InitStructure.GPIO_Pin = LED_G_Pin| LED_R_Pin | LED_Y_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(LED_Port, &GPIO_InitStructure);
#endif
	//USART_TX_RX Pins
	GPIO_InitStructure.GPIO_Pin = USART1_Tx_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_Tx_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  USART1_Rx_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART1_Rx_Port, &GPIO_InitStructure);
	/* USART1 Remapping pins */
	//GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
	/* MOTOR2_PWM_Port Configuration: Channel 1/1N, 2/2N, 3/3N and 4 as alternate function push-pull */
	//Initialize Motor1,3,4 pin PA15, PA2, PA3
	/* Disable the Serial Wire Jtag Debug Port SWJ-DP */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = MOTOR1_PWM_Pin|MOTOR2_PWM_Pin|MOTOR3_PWM_Pin|MOTOR4_PWM_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR1_PWM_Port, &GPIO_InitStructure);
	//MOTOR1_PWM_Port->ODR &= ~(MOTOR1_PWM_Pin|MOTOR3_PWM_Pin|MOTOR4_PWM_Pin);
	//Initialize Motor2 pin PB.3
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* TIM2 Partial remapping pins for use PA2, PA3, PA15, PB3 */
	//GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); 
	
	/* RC_PWM_Port Configuration: Channel 1/1N, 2/2N, 3/3N and 4 as alternate function push-pull */
	//Release PB4 an normal GPIO port
	//AFIO->MAPR|= (0x02<<24);	//010: JTAG-DP Disabled and SW-DP Enabled
	GPIO_InitStructure.GPIO_Pin = RC1_PWM_Pin|RC2_PWM_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RC_PWM_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = RC3_PWM_Pin|RC4_PWM_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RC_PWM_Port, &GPIO_InitStructure);
	/* TIM3 Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); 
}
/**
 * @fn NVIC_Configuration(void)
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
			/* Enable the TIM1 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel =  TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel =  TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel =  TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {

  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
