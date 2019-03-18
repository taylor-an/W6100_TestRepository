#include "Config.h"
#include "W6100RelFunctions.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"



void sysinit(void* arg)
{
	intr_kind temp;
	
	/*
	 */
	temp = IK_DEST_UNREACH;

	if(ctlwizchip(CW_INIT_WIZCHIP,arg) == -1)
	{
		printf("W6100 initialized fail\r\n");
	}

	if(ctlwizchip(CW_SET_INTRMASK,&temp) == -1)
	{
		printf("W6100 interrupt\r\n");
	}
}

uint8_t spiReadByte(void)
{
	while (SPI_I2S_GetFlagStatus(Open_SPIx, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(Open_SPIx, 0xff);
	while (SPI_I2S_GetFlagStatus(Open_SPIx, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(Open_SPIx);
}

void spiWriteByte(uint8_t byte)
{
	while (SPI_I2S_GetFlagStatus(Open_SPIx, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(Open_SPIx, byte);
	while (SPI_I2S_GetFlagStatus(Open_SPIx, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(Open_SPIx);
}
void spiReadBurst(uint8_t* pbuf,datasize_t len)
{
	uint8_t i;
	for(i=0; i<len; i++){
		while (SPI_I2S_GetFlagStatus(Open_SPIx, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(Open_SPIx, 0xff);
		while (SPI_I2S_GetFlagStatus(Open_SPIx, SPI_I2S_FLAG_RXNE) == RESET);
		pbuf[i] = SPI_I2S_ReceiveData(Open_SPIx);
	}
}

void spiWriteBurst(uint8_t* pbuf, datasize_t len)
{
	uint8_t i;
	for(i = 0; i<len; i++){
		while (SPI_I2S_GetFlagStatus(Open_SPIx, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(Open_SPIx, pbuf[i]);
		while (SPI_I2S_GetFlagStatus(Open_SPIx, SPI_I2S_FLAG_RXNE) == RESET);
		SPI_I2S_ReceiveData(Open_SPIx);
	}
}

//(*bus_wb)(uint32_t addr, iodata_t wb);
void busWriteByte(uint32_t addr, iodata_t data)
{
	(*((volatile uint8_t*)(_WIZCHIP_IO_BASE_+1))) = (uint8_t)((addr &0xFF00)>>8);
	(*((volatile uint8_t*)(_WIZCHIP_IO_BASE_+2))) = (uint8_t)((addr) & 0x00FF);
	(*((volatile uint8_t*)(_WIZCHIP_IO_BASE_+3))) = data;
}

//iodata_t (*bus_rb)(uint32_t addr);
iodata_t busReadByte(uint32_t addr)
{
	(*((volatile uint8_t*)(_WIZCHIP_IO_BASE_+1))) = (uint8_t)((addr &0xFF00)>>8);
	(*((volatile uint8_t*)(_WIZCHIP_IO_BASE_+2))) = (uint8_t)((addr) & 0x00FF);
	return  (*((volatile uint8_t*)(_WIZCHIP_IO_BASE_+3)));
}

 void csEnable(void)
{
	GPIO_ResetBits(GPIO_CS_PORT, GPIO_PIN_CS);
}

 void csDisable(void)
{
	GPIO_SetBits(GPIO_CS_PORT, GPIO_PIN_CS);
}

 void resetAssert(void)
{
	GPIO_ResetBits(WIZReset_Port, WIZReset_Pin);
}

//inline void resetDeassert(void)
 void resetDeassert(void)
{
	GPIO_SetBits(WIZReset_Port, WIZReset_Pin);
}

void EnterCris(void)
{
	NVIC_DisableIRQ(EXTI15_10_IRQn);
}

void ExitCris(void)
{
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}
//void W6100Reset(void)
//{
//	int i,j,k;
//	k=0;
//	GPIO_ResetBits(WIZReset_Port,WIZReset_Pin);
//	TickDelay(10);
//	
//	GPIO_SetBits(WIZReset_Port,WIZReset_Pin);
//}
