/**
  ******************************************************************************
  * @file    stm32_eval_i2c_tsensor.c
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file provides a set of functions needed to manage the I2C SENS 
  *          temperature sensor mounted on STM32xx-EVAL board (refer to stm32_eval.h
  *          to know about the boards supporting this sensor). 
  *          It implements a high level communication layer for read and write 
  *          from/to this sensor. The needed STM32 hardware resources (I2C and 
  *          GPIO) are defined in stm32xx_eval.h file, and the initialization is 
  *          performed in SENS_LowLevel_Init() function declared in stm32xx_eval.c 
  *          file.
  *            
  *          Note:
  *          -----    
  *          This driver uses the DMA method to send and receive data on I2C bus,
  *          which allows higher efficiency and reliability  of the communication.
  *                 
  *          You can easily tailor this driver to any other development board, 
  *          by just adapting the defines for hardware resources and 
  *          SENS_LowLevel_Init() function.
  *
  *     +-----------------------------------------------------------------+
  *     |                        Pin assignment                           |                 
  *     +---------------------------------------+-----------+-------------+
  *     |  STM32 I2C Pins                       |   STSENS  |   Pin       |
  *     +---------------------------------------+-----------+-------------+
  *     | SENS_I2C_SDA_PIN/ SDA                 |   SDA     |    1        |
  *     | SENS_I2C_SCL_PIN/ SCL                 |   SCL     |    2        |
  *     | SENS_I2C_SMBUSALERT_PIN/ SMBUS ALERT  |   OS/INT  |    3        |
  *     | .                                     |   GND     |    4  (0V)  |
  *     | .                                     |   GND     |    5  (0V)  |
  *     | .                                     |   GND     |    6  (0V)  |
  *     | .                                     |   GND     |    7  (0V)  |
  *     | .                                     |   VDD     |    8  (3.3V)|
  *     +---------------------------------------+-----------+-------------+
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup Common
  * @{
  */
  
/** @addtogroup STM32_EVAL_I2C_TSENSOR
  * @brief      This file includes the SENS Temperature Sensor driver of 
  *             STM32-EVAL boards.
  * @{
  */ 

/** @defgroup STM32_EVAL_I2C_TSENSOR_Private_Types
  * @{
  */ 
/**
  * @}
  */

/** @defgroup STM32_EVAL_I2C_TSENSOR_Private_Defines
  * @{
  */ 
#define SENS_SD_SET                0x01 /*!< Set SD bit in the configuration register */
#define SENS_SD_RESET              0xFE /*!< Reset SD bit in the configuration register */
/**
  * @}
  */ 

/** @defgroup STM32_EVAL_I2C_TSENSOR_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 
  
/** @defgroup STM32_EVAL_I2C_TSENSOR_Private_Variables
  * @{
  */ 
  
__IO uint32_t  SENS_Timeout = SENS_LONG_TIMEOUT; 
/**
  * @}
  */ 

/** @defgroup STM32_EVAL_I2C_TSENSOR_Private_Function_Prototypes
  * @{
  */ 
static void I2C_DMA_Config(I2C_DMADirection_TypeDef Direction, uint8_t* buffer, uint8_t NumData);

/**
  * @}
  */ 


/** @defgroup STM32_EVAL_I2C_TSENSOR_Private_Functions
  * @{
  */ 



/**
  * @brief  Initializes the SENS_I2C.
  * @param  None
  * @retval None
  */
void I2C_Configure(void)
{
  I2C_InitTypeDef   I2C_InitStructure;
  
  I2C_LowLevel_Init();
  I2C_Cmd(SENS_I2C, DISABLE);
  I2C_DeInit(SENS_I2C);
  I2C_Cmd(SENS_I2C, ENABLE);
  /*!< SENS_I2C Init */
  I2C_InitStructure.I2C_Mode 				= I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle 			= I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 		= 0x00;
  I2C_InitStructure.I2C_Ack 				= I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed 			= SENS_I2C_SPEED;
  I2C_Init(SENS_I2C, &I2C_InitStructure);
  /*!< SENS_I2C Init */
  I2C_Cmd(SENS_I2C, ENABLE);
}

/**
  * @brief  Initializes the SENS_I2C..
  * @param  None
  * @retval None
  */
void I2C_LowLevel_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/*!< SENS_I2C Periph clock enable */
	RCC_APB1PeriphClockCmd(SENS_I2C_CLK, ENABLE);

	/*!< SENS_I2C_SCL_GPIO_CLK, SENS_I2C_SDA_GPIO_CLK 
	   and SENS_I2C_SMBUSALERT_GPIO_CLK Periph clock enable */
	RCC_APB2PeriphClockCmd(SENS_I2C_GPIO_CLK, ENABLE);

	/*!< Configure SENS_I2C pins: SCL */
	GPIO_InitStructure.GPIO_Pin = SENS_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SENS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

	/*!< Configure SENS_I2C pins: SDA */
	GPIO_InitStructure.GPIO_Pin = SENS_I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(SENS_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
}

void I2C_ResetSCL(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = SENS_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SENS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(SENS_I2C_SCL_GPIO_PORT, SENS_I2C_SCL_PIN);
	Delay_us(100);
	/*!< Configure SENS_I2C pins: SCL */
	GPIO_InitStructure.GPIO_Pin = SENS_I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(SENS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Configure the DMA Peripheral used to handle communication via I2C.
  * @param  None
  * @retval None
  */

static void I2C_DMA_Config(I2C_DMADirection_TypeDef Direction, uint8_t* buffer, uint8_t NumData)
{
  DMA_InitTypeDef DMA_InitStructure;
  
  RCC_AHBPeriphClockCmd(I2C_DMA_CLK, ENABLE);
  
  /* Initialize the DMA_PeripheralBaseAddr member */
  DMA_InitStructure.DMA_PeripheralBaseAddr = SENS_I2C_DR;
  /* Initialize the DMA_MemoryBaseAddr member */
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffer;
   /* Initialize the DMA_PeripheralInc member */
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  /* Initialize the DMA_MemoryInc member */
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  /* Initialize the DMA_PeripheralDataSize member */
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  /* Initialize the DMA_MemoryDataSize member */
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  /* Initialize the DMA_Mode member */
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  /* Initialize the DMA_Priority member */
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  /* Initialize the DMA_M2M member */
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  /* If using DMA for Reception */
  if (Direction == I2C_DMA_RX)
  {
    /* Initialize the DMA_DIR member */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    
    /* Initialize the DMA_BufferSize member */
    DMA_InitStructure.DMA_BufferSize = NumData;
    
    DMA_DeInit(I2C_DMA_RX_CHANNEL);
    
    DMA_Init(I2C_DMA_RX_CHANNEL, &DMA_InitStructure);
  }
   /* If using DMA for Transmission */
  else if (Direction == I2C_DMA_TX)
  { 
    /* Initialize the DMA_DIR member */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    
    /* Initialize the DMA_BufferSize member */
    DMA_InitStructure.DMA_BufferSize = NumData;
    
    DMA_DeInit(I2C_DMA_TX_CHANNEL);
    
    DMA_Init(I2C_DMA_TX_CHANNEL, &DMA_InitStructure);
  }
}
int I2C_WriteBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t byteNum, uint8_t *buf)
{
	uint32_t I2C_TimeOut = I2C_TIMEOUT;
	/* Send STRAT condition */
	I2C_GenerateSTART(SENS_I2C, ENABLE);
	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_MODE_SELECT))	{
		if(I2C_TimeOut--==0){	return -1;		}
	}
	/* Send DEVICE address for write */
	I2C_Send7bitAddress(SENS_I2C, slaveAddr, I2C_Direction_Transmitter);
	I2C_TimeOut = I2C_TIMEOUT;
	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(I2C_TimeOut--==0){	return -1;		}
	}
	/* Send the DEVICE's internal address to write to */
	I2C_SendData(SENS_I2C, regAddr);
	I2C_TimeOut = I2C_TIMEOUT;
	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))	{
		if(I2C_TimeOut--==0){	return -1;		}
	}
	/* While there is data to be sent */
	while(byteNum)  
	{
		/* Send the byte to be written */
		I2C_SendData(SENS_I2C, *buf);
		I2C_TimeOut = I2C_TIMEOUT;
		while (!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
			if(I2C_TimeOut--==0)	{	return -7;	}
		}
		/* Point to the next location where the byte read will be saved */
		buf++; 
		/* Decrement the read bytes counter */
		byteNum--;
	}
	/* Send STOP condition */
	I2C_GenerateSTOP(SENS_I2C, ENABLE);
	while(I2C_GetFlagStatus(SENS_I2C, I2C_FLAG_STOPF));
	if(I2C_TimeOut == 0)
		return ERROR;
	else
		return SUCCESS;
}
int I2C_WriteReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t RegValue)
{
	uint32_t I2C_TimeOut = I2C_TIMEOUT;
	/* Send STRAT condition */
	I2C_GenerateSTART(SENS_I2C, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_MODE_SELECT))	{
		if(I2C_TimeOut--==0){	return -1;		}
	}
	/* Send EEPROM address for write */
	I2C_Send7bitAddress(SENS_I2C, slaveAddr, I2C_Direction_Transmitter);
	I2C_TimeOut = I2C_TIMEOUT;
	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))	{
		if(I2C_TimeOut--==0){	return -2;		}
	}
	
	/* Send the EEPROM's internal address to write to */
	I2C_SendData(SENS_I2C, regAddr);
	I2C_TimeOut = I2C_TIMEOUT;
	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))	{
		if(I2C_TimeOut--==0){	return -3;		}
	}
	/* Send the byte to be written */
	I2C_SendData(SENS_I2C, RegValue); 
	I2C_TimeOut = I2C_TIMEOUT;
	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(I2C_TimeOut--==0){	return -4;		}
	}
	/* Send STOP condition */
	I2C_GenerateSTOP(SENS_I2C, ENABLE);
	while(I2C_GetFlagStatus(SENS_I2C, I2C_FLAG_STOPF));
	if(I2C_TimeOut == 0)
		return ERROR;
	else
		return SUCCESS;
}
/**
  * @brief  Read the specified register from the LM75.
  * @param  RegName: specifies the LM75 register to be read.
  *              This member can be one of the following values:  
  *                  - LM75_REG_TEMP: temperature register
  *                  - LM75_REG_TOS: Over-limit temperature register
  *                  - LM75_REG_THYS: Hysteresis temperature register
  * @retval LM75 register value.
  */
uint16_t I2C_DMAReadReg(uint8_t devAddr, uint8_t RegName)
{   
  uint8_t LM75_BufferRX[2] ={0,0};
  uint16_t tmp = 0;    
  uint32_t  I2C_TimeOut = I2C_TIMEOUT; 
   /* Test on BUSY Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_BUSY)) 
  {
    if((I2C_TimeOut--) == 0) return -1;
  }
  
  /* Configure DMA Peripheral */
  I2C_DMA_Config(I2C_DMA_RX, (uint8_t*)LM75_BufferRX, 2);  
  
  /* Enable DMA NACK automatic generation */
  I2C_DMALastTransferCmd(SENS_I2C, ENABLE);
  
  /* Enable the I2C peripheral */
  I2C_GenerateSTART(SENS_I2C, ENABLE);
  
  /* Test on SB Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_SB)) 
  {
    if((I2C_TimeOut--) == 0) -2;
  }
  
  /* Send device address for write */
  I2C_Send7bitAddress(SENS_I2C, devAddr, I2C_Direction_Receiver);
  
  /* Test on ADDR Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) 
  {
    if((I2C_TimeOut--) == 0) return -3;
  }
  
  /* Send the device's internal address to write to */
  I2C_SendData(SENS_I2C, RegName);  
  
  /* Test on TXE FLag (data sent) */
  I2C_TimeOut = I2C_TIMEOUT;
  while ((!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_BTF)))  
  {
    if((I2C_TimeOut--) == 0) return -4;
  }
  
  /* Send START condition a second time */  
  I2C_GenerateSTART(SENS_I2C, ENABLE);
  
  /* Test on SB Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_SB)) 
  {
    if((I2C_TimeOut--) == 0) return -5;
  }
  
  /* Send LM75 address for read */
  //I2C_Send7bitAddress(SENS_I2C, devAddr, I2C_Direction_Receiver);
  
  /* Test on ADDR Flag */
//  I2C_TimeOut = I2C_TIMEOUT;
//  while (!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))   
//  {
//    if((I2C_TimeOut--) == 0) return -6;
//  }
  
  /* Enable I2C DMA request */
  I2C_DMACmd(SENS_I2C,ENABLE);
  
  /* Enable DMA RX Channel */
  DMA_Cmd(I2C_DMA_RX_CHANNEL, ENABLE);
  
  /* Wait until DMA Transfer Complete */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!DMA_GetFlagStatus(I2C_DMA_RX_TCFLAG))
  {
    if((I2C_TimeOut--) == 0) return -7;
  }        
  
  /* Send STOP Condition */
  I2C_GenerateSTOP(SENS_I2C, ENABLE);
  
  /* Disable DMA RX Channel */
  DMA_Cmd(I2C_DMA_RX_CHANNEL, DISABLE);
  
  /* Disable I2C DMA request */  
  I2C_DMACmd(SENS_I2C,DISABLE);
  
  /* Clear DMA RX Transfer Complete Flag */
  DMA_ClearFlag(I2C_DMA_RX_TCFLAG);
  
  /*!< Store SENS_I2C received data */
  tmp = (uint16_t)(LM75_BufferRX[0] << 8);
  tmp |= LM75_BufferRX[1];
  
  /* return a Reg value */
  return (uint16_t)tmp;  
}

int I2C_WriteAddr(uint8_t devId, uint8_t addr)
{
	uint32_t I2C_TimeOut = I2C_TIMEOUT;
	uint8_t tmp = 0;
	/* While the bus is busy */
	while(I2C_GetFlagStatus(SENS_I2C, I2C_FLAG_BUSY))
	{
		if(I2C_TimeOut--==0){	return -1;		}
	}
	/* Send START condition */
	I2C_GenerateSTART(SENS_I2C, ENABLE);
	I2C_AcknowledgeConfig(SENS_I2C, ENABLE);
	/* Test on EV5 and clear it */
	I2C_TimeOut = I2C_TIMEOUT;
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(I2C_TimeOut--==0)	{	return -2;	}
	}
	/* Send EEPROM address for write */
	I2C_Send7bitAddress(SENS_I2C, devId, I2C_Direction_Transmitter);
	
	/* Test on EV6 and clear it */
	I2C_TimeOut = I2C_TIMEOUT;
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if(I2C_TimeOut--==0)	{	return -3;		}
	}

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(SENS_I2C, ENABLE);

	/* Send the EEPROM's internal address to write to */
	I2C_SendData(SENS_I2C, addr);  

	/* Test on EV8 and clear it */
	I2C_TimeOut = I2C_TIMEOUT;
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if(I2C_TimeOut--==0){	return -4;	}
	}

	/* Send STRAT condition a second time */  
	I2C_GenerateSTART(SENS_I2C, ENABLE);
	/* Test on EV5 and clear it */
	I2C_TimeOut = I2C_TIMEOUT;
	while(!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if(I2C_TimeOut--==0)	{	return -5;	}
	}
	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(SENS_I2C, ENABLE);
	I2C_GenerateSTOP(SENS_I2C, ENABLE);
	while(I2C_GetFlagStatus(SENS_I2C, I2C_FLAG_STOPF));
	return tmp;
}



int I2C_ReadReg(uint8_t devAddress, uint8_t RegistAddr)
{
	uint32_t I2C_TimeOut = I2C_TIMEOUT;
	uint8_t tmp = 0;
	uint8_t bufferRx[2] = {0,0};
	/* While the bus is busy */
	while(I2C_GetFlagStatus(SENS_I2C, I2C_FLAG_BUSY))
	{
		if(I2C_TimeOut--==0){	return -1;		}
	}
	/* Configure DMA Peripheral */
  I2C_DMA_Config(I2C_DMA_RX, (uint8_t*)bufferRx, 2);  
  
  /* Enable DMA NACK automatic generation */
  I2C_DMALastTransferCmd(SENS_I2C, ENABLE);
  
  /* Enable the I2C peripheral */
  I2C_GenerateSTART(SENS_I2C, ENABLE);
  
  /* Test on SB Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_SB)) 
  {
    if((I2C_TimeOut--) == 0) return -2;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Send device address for write */
  I2C_Send7bitAddress(SENS_I2C, devAddress, I2C_Direction_Transmitter);
  /* Test on ADDR Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) 
  {
    if((I2C_TimeOut--) == 0) return -3;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Send the device's internal address to write to */
  I2C_SendData(SENS_I2C, RegistAddr);  
  
  /* Test on TXE FLag (data sent) */
  I2C_TimeOut = I2C_TIMEOUT;
  while ((!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_BTF)))  
  {
    if((I2C_TimeOut--) == 0) return -4;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Send START condition a second time */  
  I2C_GenerateSTART(SENS_I2C, ENABLE);
  
  /* Test on SB Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_SB)) 
  {
    if((I2C_TimeOut--) == 0) return -5;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Send LM75 address for read */
  I2C_Send7bitAddress(SENS_I2C, devAddress, I2C_Direction_Receiver);
  
  /* Test on ADDR Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))   
  {
    if((I2C_TimeOut--) == 0) return -6;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Enable I2C DMA request */
  I2C_DMACmd(SENS_I2C,ENABLE);
  
  /* Enable DMA RX Channel */
  DMA_Cmd(I2C_DMA_RX_CHANNEL, ENABLE);
  
  /* Wait until DMA Transfer Complete */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!DMA_GetFlagStatus(I2C_DMA_RX_TCFLAG))
  {
    if((I2C_TimeOut--) == 0) return -7;//LM75_TIMEOUT_UserCallback();
  }        
  
  /* Send STOP Condition */
  I2C_GenerateSTOP(SENS_I2C, ENABLE);
  
  /* Disable DMA RX Channel */
  DMA_Cmd(I2C_DMA_RX_CHANNEL, DISABLE);
  
  /* Disable I2C DMA request */  
  I2C_DMACmd(SENS_I2C,DISABLE);
  
  /* Clear DMA RX Transfer Complete Flag */
  DMA_ClearFlag(I2C_DMA_RX_TCFLAG);
	
	return bufferRx[0];
}

int I2C_ReadDMABytes(uint8_t devAddress, uint8_t RegistAddr, int nBytes, uint8_t *buf)
{
	uint32_t I2C_TimeOut = I2C_TIMEOUT;
	uint8_t tmp = 0;
	//uint8_t bufferRx[2] = {0,0};
	/* While the bus is busy */
	while(I2C_GetFlagStatus(SENS_I2C, I2C_FLAG_BUSY))
	{
		if(I2C_TimeOut--==0){	return -1;		}
	}
	/* Configure DMA Peripheral */
  I2C_DMA_Config(I2C_DMA_RX, buf, nBytes);  
  
  /* Enable DMA NACK automatic generation */
  I2C_DMALastTransferCmd(SENS_I2C, ENABLE);
  
  /* Enable the I2C peripheral */
  I2C_GenerateSTART(SENS_I2C, ENABLE);
  
  /* Test on SB Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_SB)) 
  {
    if((I2C_TimeOut--) == 0) return -2;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Send device address for write */
  I2C_Send7bitAddress(SENS_I2C, devAddress, I2C_Direction_Transmitter);
  /* Test on ADDR Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) 
  {
    if((I2C_TimeOut--) == 0) return -3;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Send the device's internal address to write to */
  I2C_SendData(SENS_I2C, RegistAddr);  
  
  /* Test on TXE FLag (data sent) */
  I2C_TimeOut = I2C_TIMEOUT;
  while ((!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_BTF)))  
  {
    if((I2C_TimeOut--) == 0) return -4;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Send START condition a second time */  
  I2C_GenerateSTART(SENS_I2C, ENABLE);
  
  /* Test on SB Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_GetFlagStatus(SENS_I2C,I2C_FLAG_SB)) 
  {
    if((I2C_TimeOut--) == 0) return -5;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Send LM75 address for read */
  I2C_Send7bitAddress(SENS_I2C, devAddress, I2C_Direction_Receiver);
  
  /* Test on ADDR Flag */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!I2C_CheckEvent(SENS_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))   
  {
    if((I2C_TimeOut--) == 0) return -6;//LM75_TIMEOUT_UserCallback();
  }
  
  /* Enable I2C DMA request */
  I2C_DMACmd(SENS_I2C,ENABLE);
  
  /* Enable DMA RX Channel */
  DMA_Cmd(I2C_DMA_RX_CHANNEL, ENABLE);
  
  /* Wait until DMA Transfer Complete */
  I2C_TimeOut = I2C_TIMEOUT;
  while (!DMA_GetFlagStatus(I2C_DMA_RX_TCFLAG))
  {
    if((I2C_TimeOut--) == 0) return -7;//LM75_TIMEOUT_UserCallback();
  }        
  
  /* Send STOP Condition */
  I2C_GenerateSTOP(SENS_I2C, ENABLE);
  
  /* Disable DMA RX Channel */
  DMA_Cmd(I2C_DMA_RX_CHANNEL, DISABLE);
  
  /* Disable I2C DMA request */  
  I2C_DMACmd(SENS_I2C,DISABLE);
  
  /* Clear DMA RX Transfer Complete Flag */
  DMA_ClearFlag(I2C_DMA_RX_TCFLAG);
	
	return 1;
}


/**
  * @brief  DeInitializes the SENS_I2C.
  * @param  None
  * @retval None
  */
void SENS_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /*!< Disable SENS_I2C */
  I2C_Cmd(SENS_I2C, DISABLE);
  /*!< DeInitializes the SENS_I2C */
  I2C_DeInit(SENS_I2C);
  
  /*!< SENS_I2C Periph clock disable */
  RCC_APB1PeriphClockCmd(SENS_I2C_CLK, DISABLE);
    
  /*!< Configure SENS_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = SENS_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SENS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure SENS_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = SENS_I2C_SDA_PIN;
  GPIO_Init(SENS_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure SENS_I2C pin: SMBUS ALERT */
//  GPIO_InitStructure.GPIO_Pin = SENS_I2C_SMBUSALERT_PIN;
//  GPIO_Init(SENS_I2C_SMBUSALERT_GPIO_PORT, &GPIO_InitStructure);
}
/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
