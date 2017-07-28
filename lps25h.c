
#include "stm32f10x.h"
#include "i2c.h"
#include "lps25h.h"
#include "config.h"

LPS25_Typedef lps25;

void lps25_Config(void)
{
	lps25.devID = LPS25_I2C_ADDRESS;
	lps25.i32_PressRaw = 0;
	lps25.i16_TempRaw = 0;
	lps25.Conf_RES = (LPS25_AVGP128<<2)|(LPS25_AVGT128);
	lps25.Cntrol_REG1 = (1<<7)|(LPS25_ODR_25Hz<<4);
	lps25.WhoAmI = I2C_ReadReg(lps25.devID, LPS25_WHO_AM_I);
	I2C_WriteReg(lps25.devID, LPS25_RES_CONF, lps25.Conf_RES);
	I2C_WriteReg(lps25.devID, LPS25_CTRL_REG1, lps25.Cntrol_REG1);
}


void lps25_Init(void)
{
	
}

void lps25_ReadPress(void)
{
	int32_t temp = 0;
	int16_t i16t = 0;
	temp = I2C_ReadReg(lps25.devID, LPS25_PRES_POUT_XL);
	temp |= (I2C_ReadReg(lps25.devID, LPS25_PRES_OUT_L)<<8);
	temp |= (I2C_ReadReg(lps25.devID, LPS25_PRES_OUT_H)<<16);
	lps25.i32_PressRaw = temp;
	lps25.f_hPa = (float)temp/4096;
	i16t = I2C_ReadReg(lps25.devID, LPS25_TEMP_OUT_L);
	i16t |= (I2C_ReadReg(lps25.devID, LPS25_TEMP_OUT_H)<<8);
	lps25.i16_TempRaw = i16t;
	lps25.f_DegC = (float)i16t/480 + 42.5;
	
}