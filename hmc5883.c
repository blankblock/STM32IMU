
#include "hmc5883.h"
#include "config.h"
#include "i2c.h"
#include "utill.h"
#include "math.h"

int gGaussScale[8] = {
    1370,   //  0.9
    1090,   //  1.3
    820,    //  1.9
    660,    //  2.5
    440,    //  4.0
    390,    //  4.7
    330,    //  5.6
    230     //  8.1
    };
uint8_t hmcBuff[6] = {0,};
uint16_t mag_x = 0;
uint16_t mag_y = 0;
uint16_t mag_z = 0;
uint8_t confa = 0;
HMC5883L_Typedef compass;

void HMC5883_Config(void)
{
	int i = 0, ret = 0;
	//uint8_t tmp = 0;
	//compass.eGainConfig = GausGain_47;
	MHC5883L_CONFset(&compass, MagAvgSamples_0, MagOutRate_30Hz, MagMeasurementMode_Normal);
	HMC5883L_GainSet(&compass, GausGain_13);
	Delay_us(1000);	//Wait for 1ms
	ret = I2C_WriteReg(HMC_ADDR_W, HMC_REG_CONFIG_A, compass.u8_Config_A);
	ret =I2C_WriteReg(HMC_ADDR_W, HMC_REG_CONFIG_B, compass.u8_Config_B);
	ret = I2C_WriteReg(HMC_ADDR_W, HMC_REG_MODE, 0x00);		//Configure device for continuous measurement mode
	compass.u8_ConfSet_A = I2C_ReadReg(HMC_ADDR_R, HMC_REG_CONFIG_A);
	compass.u8_ConfSet_B = I2C_ReadReg(HMC_ADDR_R, HMC_REG_CONFIG_B);
	Delay_us(1000);
}

void MHC5883L_CONFset(HMC5883L_Typedef *mag, HMC58_AvgSamples avgs, HMC58_OutputRate outputRate, HMC58_MeasMode mode)
{
	mag->u8_Config_A = 0;
	mag->u8_Config_A = (avgs<<5)|((outputRate&0x07)<<2)|((mode)&0x03);
}
void HMC5883L_GainSet(HMC5883L_Typedef *mag, HMC58_GainConfig gain)
{
	mag->u8_Config_B = (gain<<5);
	switch(gain)
	{
		case GausGain_09:
			mag->gain_LSB_Gauss = 1370.0;
			break;
		case GausGain_13:
			mag->gain_LSB_Gauss = 1090.0;
			break;
		case GausGain_19:
			mag->gain_LSB_Gauss = 820.0;
			break;
		case GausGain_25:
			mag->gain_LSB_Gauss = 660.0;
			break;
		case GausGain_40:
			mag->gain_LSB_Gauss = 440.0;
			break;
		case GausGain_47:
			mag->gain_LSB_Gauss = 390.0;
			break;
		case GausGain_56:
			mag->gain_LSB_Gauss = 330.0;
			break;
		case GausGain_81:
			mag->gain_LSB_Gauss = 230.0;
			break;
	}
}
void HMC5883_ReadData(void)
{
	int i =0;
	I2C_ReadDMABytes(HMC_ADDR_R, 0x06, 6, hmcBuff);
	confa = I2C_WriteAddr(HMC_ADDR_W, HMC_REG_X_MSB);
	compass.magRaw[HMC5883_X_ORDER] = ((hmcBuff[0]<<8)|(hmcBuff[1])) * HMC5883_X_ORIENT;
	compass.magRaw[HMC5883_Z_ORDER] = ((hmcBuff[2]<<8)|(hmcBuff[3])) * HMC5883_Z_ORIENT;
	compass.magRaw[HMC5883_Y_ORDER] = ((hmcBuff[4]<<8)|(hmcBuff[5])) * HMC5883_Y_ORIENT;
	for(i=0;i<3;i++)
	{
		compass.magGauss[i] = (float)compass.magRaw[i] / compass.gain_LSB_Gauss;
	}
	compass.heading = atan2(compass.magGauss[0], compass.magGauss[1]) * RAD_TO_DEG;
}
