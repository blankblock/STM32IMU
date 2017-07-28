#include "i2c.h"
#include "mpu9150.h"

MPU9150_Typedef mpu9150;
uint16_t gyro_x = 0;
uint16_t gyro_y = 0;
uint16_t gyro_z = 0;
int16_t accelData[3] = {0,0,0};
int16_t gyroData[3] = {0,0,0};
char bufferTx[50]; 

void mpu9150_Config()
{
	int i = 0;
	uint8_t ret;
	mpu9150.f_Temperature = 0.0;
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_1, 0x80);		//Reset the device
	Delay_us(8000);
	mpu9150_Init(&mpu9150.Config_User);
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_1, mpu9150.Config_User.PWR_MGMT1);
	ret = I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_SMPLRT_DIV, mpu9150.Config_User.SampleRate);
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_CONFIG, mpu9150.Config_User.Config_DLPF);
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_GYRO_CONFIG, (mpu9150.Config_User.GyroConfig));
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_ACCEL_CONFIG, (mpu9150.Config_User.AccelConfig));
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_USER_CTRL, 0x00);
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_I2C_MST_CTRL, 0x00);
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_INT_ENABLE, 0x00);
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_FIFO_EN, (0x07<<4));
	I2C_WriteReg(MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_2, 0xC7);
	mpu9150_Ident();
	return;
}
void mpu9150_Init(MPU9150_Conf *config)
{
	config->PWR_MGMT1 = 0x01;			//Use PLL with X axis gyroscope reference
	config->SampleRate = 7;//24;		//Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	mpu9150_AccelRangeSet(AccelRange_2g);
	mpu9150_GyroRangeSet(GyroRange_250degs);
	config->Config_DLPF = DLPF_260Hz;		//Disable DLPF setting 
}
void mpu9150_AccelRangeSet(MPU9150_AccelRange aRange)
{
	mpu9150.Config_User.AccelConfig = (aRange<<3);
	switch(aRange)
	{
		case AccelRange_2g:
			mpu9150.accelRange_g = 32768/2;
			break;
		case AccelRange_4g:
			mpu9150.accelRange_g = 32768/4;
			break;
		case AccelRange_8g:
			mpu9150.accelRange_g = 32768/8;
			break;
		case AccelRange_16g:
			mpu9150.accelRange_g = 32768/16;
			break;
	}
}
void mpu9150_GyroRangeSet(MPU9150_GyroRange gRange)
{
	mpu9150.Config_User.GyroConfig = (0x07<<5)|(gRange<<3);
	switch(gRange)
	{
		case GyroRange_250degs:
			mpu9150.gyroGain_LSB_dps = 131.0;
			break;
		case GyroRange_500degs:
			mpu9150.gyroGain_LSB_dps = 65.5;
			break;
		case GyroRange_1000degs:
			mpu9150.gyroGain_LSB_dps = 32.8;
			break;
		case GyroRange_2000degs:
			mpu9150.gyroGain_LSB_dps = 16.4;
			break;
	}
}
void mpu9150_Ident()
{
	mpu9150.Config_Return.PWR_MGMT1 = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_1);
	mpu9150.Config_Return.AccelConfig = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_2);
	mpu9150.Config_Return.Config_DLPF = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_CONFIG);
	mpu9150.Config_Return.GyroConfig = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_GYRO_CONFIG);
	mpu9150.Config_Return.I2C_MasterCTRL = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_I2C_MST_CTRL);
	mpu9150.Config_Return.SampleRate = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_SMPLRT_DIV);
	mpu9150.Config_Return.ADDR = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_WHO_AM_I);
	mpu9150.Config_Return.FIFO_Mode = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_FIFO_EN);
	sprintf(bufferTx, "ID: %2x, Gyro: %2x, SR: %2x, PM1: %2x, Cnf: %2x\r\n", 
			mpu9150.Config_Return.ADDR,
			mpu9150.Config_Return.AccelConfig,
			mpu9150.Config_Return.SampleRate,
			mpu9150.Config_Return.PWR_MGMT1,
			mpu9150.Config_Return.Config_DLPF);
	USART_DMA_Send(bufferTx);
	mpu9150_SelfTest();
}
void mpu9150_ReadGyro()
{
	uint16_t temp = 0;
	gyroData[0] = (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_GYRO_XOUT_H)<<8);
	gyroData[0]|= (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_GYRO_XOUT_L)&0xFF);
	gyroData[1] = (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_GYRO_YOUT_H)<<8);
	gyroData[1]|= (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_GYRO_YOUT_L)&0xFF);
	gyroData[2] = (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_GYRO_ZOUT_H)<<8);
	gyroData[2]|= (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_GYRO_ZOUT_L)&0xFF);
	temp = ((I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_FIFO_COUNTH)&0x3)<<8);
	temp |= (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_FIFO_COUNTL)&0xFF);
	mpu9150.fifoCount = temp;
}
void mpu9150_ReadAccel()
{
	int i = 0;
	accelData[0] = (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_ACCEL_XOUT_H)<<8);
	accelData[0]|= (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_ACCEL_XOUT_L)&0xFF);
	accelData[1] = (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_ACCEL_YOUT_H)<<8);
	accelData[1]|= (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_ACCEL_YOUT_L)&0xFF);
	accelData[2] = (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_ACCEL_ZOUT_H)<<8);
	accelData[2]|= (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_ACCEL_ZOUT_L)&0xFF);
	for(i=0;i<3;i++)
	{
		mpu9150.accelData[i] = (float)accelData[i] / mpu9150.accelRange_g;
	}
}
void mpu9150_ReadTemp()
{
	int16_t temp = 0;
	temp = (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_TEMP_OUT_H)<<8);
	temp |= (I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_TEMP_OUT_L)&0xFF);
	mpu9150.f_Temperature = (float)temp/340+35;
}
void mpu9150_SelfTest(void)
{
	int i;
	uint8_t temp = 0;
	for(i=0;i<3;i++)
	{
		temp = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_SELF_TEST_X+i);
		mpu9150.selfTestA[i] = ((temp&0x70)>>3);
		mpu9150.selfTestG[i] = (temp&0x0F);
	}
	temp = I2C_ReadReg(MPU9150_I2C_ADDRESS, MPU9150_SELF_TEST_A);
	mpu9150.selfTestA[0] |= ((temp&0x18)>>4);
	mpu9150.selfTestA[1] |= ((temp&0x0C)>>2);
	mpu9150.selfTestA[2] |= ((temp&0x03));
	sprintf(bufferTx, "SelfTest:xa:%4d ya:%4d za:%4d gx:%4d, gy:%4d, gz:%4d\r\n", 
			mpu9150.selfTestA[0],
			mpu9150.selfTestA[1],
			mpu9150.selfTestA[2],
			mpu9150.selfTestG[0],
			mpu9150.selfTestG[1],
			mpu9150.selfTestG[2]);
	USART_DMA_Send(bufferTx);
	
}