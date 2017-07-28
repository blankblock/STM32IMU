
#include "stm32f10x.h"
#include "i2c.h"
#include "l3g4200.h"
#include "config.h"

stGyro_Typedef stGyro;

void stGyro_Config(void)
{
	uint8_t tmp = 0;
	stGyro.i16_rawX = 0;
	stGyro.i16_rawY = 0;
	stGyro.i16_rawZ = 0;
	stGyro.reg_s.ctrl_reg1 = (STGYRO_REG_DR_400Hz<<6)|(0x02<<4)|(0x0F);	//Set 400Hz, Bandwidth 50, Normal Mode, Enable 3 axis
	tmp = I2C_ReadReg(STGYRO_SDA, STGYRO_REG_WHO_AM_I);
	I2C_WriteReg(STGYRO_SDA, STGYRO_REG_CTRL_REG1, stGyro.reg_s.ctrl_reg1);
	stGyro_SetDPS(StGyroRange_250dps);
	
}
void stGyro_SetDPS(stGyro_DPS dps)
{
	switch(dps)
	{
		case StGyroRange_250dps:
			stGyro.dps = 250.0;
			stGyro.reg_s.ctrl_reg4 = (0x00);
			break;
		case StGyroRange_500dps:
			stGyro.dps = 500.0;
			stGyro.reg_s.ctrl_reg4 = (0x01<<4);
			break;
		case StGyroRange_2000dps:
			stGyro.dps = 2000.0;
			stGyro.reg_s.ctrl_reg4 = (0x02<<4);
			break;
	}
	I2C_WriteReg(STGYRO_SDA, STGYRO_REG_CTRL_REG4, stGyro.reg_s.ctrl_reg4);
}
void stGyro_ReadGyro(void)
{
	stGyro.temp_C = I2C_ReadReg(STGYRO_SDA, STGYRO_REG_OUT_TEMP);
	stGyro.i16_rawX = (I2C_ReadReg(STGYRO_SDA, STGYRO_REG_OUT_X_H)<<8)|
					(I2C_ReadReg(STGYRO_SDA, STGYRO_REG_OUT_X_L)&0x0F);
	stGyro.i16_rawY = (I2C_ReadReg(STGYRO_SDA, STGYRO_REG_OUT_Y_H)<<8)|
					(I2C_ReadReg(STGYRO_SDA, STGYRO_REG_OUT_Y_L)&0x0F);
	stGyro.i16_rawZ = (I2C_ReadReg(STGYRO_SDA, STGYRO_REG_OUT_Z_H)<<8)|
					(I2C_ReadReg(STGYRO_SDA, STGYRO_REG_OUT_Z_L)&0x0F);
	stGyro.degS[STGYRO_X_ORDER] = (float)stGyro.i16_rawX / stGyro.dps * STGYRO_X_ORIENT;
	stGyro.degS[STGYRO_Y_ORDER] = (float)stGyro.i16_rawY / stGyro.dps * STGYRO_Y_ORIENT;
	stGyro.degS[STGYRO_Z_ORDER] = (float)stGyro.i16_rawZ / stGyro.dps * STGYRO_Z_ORIENT;
}