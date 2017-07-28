/**
 * @file uart.c
 * @brief RS-232 communications
 */
#include "config.h"
#include "uart.h"


/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
DMA_InitTypeDef DMA_USART_Tx_InitStructure;
DMA_InitTypeDef DMA_USART_Rx_InitStructure;
char TxBuffer1[USART1_Tx_BufferSize] = "STAR-IMU V0.1 2014/4/7\r\n";
char RxBuffer1[USART1_Rx_BufferSize];
char RxBuffer2[USART1_Rx_BufferSize];
volatile uint8_t RxCounter1 = 0x00;
volatile uint8_t RxCounter2 = 0;
uint8_t RxState1 = 0;
uint8_t Rx1_Ready = 0;
uint8_t RxLength1 = 0;

int getStrLen(char *s)
{
  int l = 0;
  while(*s!='\0') {
	l++;
	s++;
  }
  return l;
}

void USART_DMA_Send(char *s)
{
  	DMA_DeInit(USART1_Tx_DMA_Channel);  
	DMA_USART_Tx_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
	DMA_USART_Tx_InitStructure.DMA_MemoryBaseAddr = (uint32_t)s;
	DMA_USART_Tx_InitStructure.DMA_BufferSize = getStrLen(s);
	DMA_Init(USART1_Tx_DMA_Channel, &DMA_USART_Tx_InitStructure);
    /* Enable USARTy TX DMA1 Channel */
  	DMA_Cmd(USART1_Tx_DMA_Channel, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	/* Wait until USARTy TX DMA1 Channel Transfer Complete */
	while (DMA_GetFlagStatus(USART1_Tx_DMA_FLAG) == RESET)
	{
	}
}
void USART_Config(USART_BAUD baud)
{
	/* USARTy and USARTz configured as follow:
	- BaudRate = 230400 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	switch(baud)
	{
	case USART_9600bps:
		USART_InitStructure.USART_BaudRate = 9600;
		break;
	case USART_19200bps:
		USART_InitStructure.USART_BaudRate = 19200;
		break;
	case USART_38400bps:
		USART_InitStructure.USART_BaudRate = 38400;
		break;
	case USART_57600bps:
		USART_InitStructure.USART_BaudRate = 57600;
		break;
	case USART_115200bps:
		USART_InitStructure.USART_BaudRate = 115200;
		break;
	default:
		USART_InitStructure.USART_BaudRate = 9600;
		break;
	}
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Configure USARTy */
	USART_Init(USART1, &USART_InitStructure);
	/* Enable USART1 Receive interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	/* Enable USART1 Transmission Complete interrupts */
	USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	/* Enable the USARTy */
	USART_Cmd(USART1, ENABLE);

	USART_DMA_Init();
	/* Enable USARTy DMA TX & RX request */
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	/* Enable USARTy TX DMA1 Channel */
	DMA_Cmd(USART1_Tx_DMA_Channel, ENABLE);
	/* Enable USARTy RX DMA1 Channel */
	//DMA_Cmd(USART1_Rx_DMA_Channel, ENABLE);
	DMA_ITConfig(USART1_Rx_DMA_Channel, DMA1_IT_GL5, ENABLE);
	/* Wait until USARTy TX DMA1 Channel Transfer Complete */
	while (DMA_GetFlagStatus(USART1_Tx_DMA_FLAG) == RESET)
	{
	}
}
void USART_DMA_Init(void)
{
	//Refer to DMA request mapping
	/* USARTy TX DMA1 Channel (triggered by USARTy Tx event) Config */
	DMA_DeInit(USART1_Tx_DMA_Channel);  
	DMA_USART_Tx_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
	DMA_USART_Tx_InitStructure.DMA_MemoryBaseAddr = 	(uint32_t)TxBuffer1;
	DMA_USART_Tx_InitStructure.DMA_DIR = 				DMA_DIR_PeripheralDST;
	DMA_USART_Tx_InitStructure.DMA_BufferSize = 		TxBufferSize1;
	DMA_USART_Tx_InitStructure.DMA_PeripheralInc = 		DMA_PeripheralInc_Disable;
	DMA_USART_Tx_InitStructure.DMA_MemoryInc = 			DMA_MemoryInc_Enable;
	DMA_USART_Tx_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_USART_Tx_InitStructure.DMA_MemoryDataSize = 	DMA_MemoryDataSize_Byte;
	DMA_USART_Tx_InitStructure.DMA_Mode = 				DMA_Mode_Normal;
	DMA_USART_Tx_InitStructure.DMA_Priority = 			DMA_Priority_VeryHigh;
	DMA_USART_Tx_InitStructure.DMA_M2M = 				DMA_M2M_Disable;
	DMA_Init(USART1_Tx_DMA_Channel, &DMA_USART_Tx_InitStructure);

	/* USARTy RX DMA1 Channel (triggered by USARTy Rx event) Config */
//	DMA_DeInit(USART1_Rx_DMA_Channel);  
//	DMA_USART_Rx_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
//	DMA_USART_Rx_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RxBuffer1;
//	DMA_USART_Rx_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//	DMA_USART_Rx_InitStructure.DMA_BufferSize = USART1_Rx_BufferSize;
//	DMA_USART_Rx_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_USART_Rx_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_USART_Rx_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_USART_Rx_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_USART_Rx_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_USART_Rx_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	DMA_USART_Rx_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_Init(USART1_Rx_DMA_Channel, &DMA_USART_Rx_InitStructure);
}


/*******************************************************************************
*	@brief		TX String Function.
*	@param[in]	UART Port, String
*	@return		void
*******************************************************************************/
void TX_String(USART_TypeDef* usart_port, char *pstr)
{
	while(*pstr)
	{
		while(USART_GetFlagStatus(usart_port, USART_FLAG_TXE) == RESET);
		USART_SendData(usart_port, *pstr++);
	}
	
}
/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	uint16_t u16tmp = 0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		//u16tmp = USART_ReceiveData(USART1);
		//RxBuffer2[RxCounter2++] = u16tmp;
		//RxBuffer1[RxCounter1++] = USART_ReceiveData(USART1);
		if(Rx1_Ready!=TRUE)
		{
			RxBuffer1[0] = USART_ReceiveData(USART1);
			Rx1_Ready = TRUE;
			/*
			switch(RxState1)
			{
				case 0:
					//Detect of '$' character
					if(u16tmp == 0x24) {
						RxState1 = 1;
						RxCounter1 = 0;
						RxBuffer1[0] = u16tmp;
					}
					else {
						RxState1 = 0;
						RxCounter1 = 0;
					}
					break;
				case 1:
					//Detect of 'End of character'
					if(u16tmp == 0x0D || u16tmp == 0x0A)
					{
						RxState1 = 0;
						Rx1_Ready = SET;
						RxLength1 = RxCounter1;
						RxCounter1 = 0;
					}
					else {
						//Buffer overflow occured!! or new message arrived before terminating the message
						if(RxCounter1 > USART1_Rx_BufferSize-1 || u16tmp ==0x02 || u16tmp == 0x24)
						{
							RxCounter1 = 0;
							RxState1 = 0;
						}
						else{
							RxBuffer1[RxCounter1] = u16tmp;
						}
						RxCounter1++;
					}
					break;
			}
			*/
		}
		if(RxCounter1>USART1_Rx_BufferSize-1) RxCounter1 = 0;
		
	}
	/*	USART1 Transmission complete interrupt is pending*/
	else if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_TC);
	}
	else if(USART_GetITStatus(USART1,USART_IT_TXE)== RESET)
	{
	  USART_ClearITPendingBit(USART1, USART_IT_TXE);
	}
	else if(USART_GetITStatus(USART1,USART_IT_ERR)== RESET)
	{
	  USART_ClearITPendingBit(USART1, USART_IT_TXE);
	}
}
