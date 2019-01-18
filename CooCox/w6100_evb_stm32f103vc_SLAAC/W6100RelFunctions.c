#include "W6100_evb.h"
#include "W6100RelFunctions.h"
#include "stm32f10x_conf.h"

void W6100Initialze(void)
{
	intr_kind temp;
	unsigned char W6100_AdrSet[2][4] = {{2,2,2,2},{2,2,2,2}};
	/*
	 */
	temp = IK_DEST_UNREACH;

	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)W6100_AdrSet) == -1)
	{
		printf("W6100 initialized fail.\r\n");
	}

	if(ctlwizchip(CW_SET_INTRMASK,&temp) == -1)
	{
		printf("W6100 interrupt\r\n");
	}
	printf("interrupt mask: %02x\r\n",getIMR());

	do{//check phy status.
		if(ctlwizchip(CW_GET_PHYLINK,(void*)&temp) == -1){
			printf("Unknown PHY link status.\r\n");
		}
	}while(temp == PHY_LINK_OFF);
}

uint8_t spiReadByte(void)
{
	while (SPI_I2S_GetFlagStatus(W6100_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(W6100_SPI, 0xff);
	while (SPI_I2S_GetFlagStatus(W6100_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(W6100_SPI);
}

void spiWriteByte(uint8_t byte)
{
	while (SPI_I2S_GetFlagStatus(W6100_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(W6100_SPI, byte);
	while (SPI_I2S_GetFlagStatus(W6100_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_I2S_ReceiveData(W6100_SPI);
}

inline void csEnable(void)
{
	GPIO_ResetBits(W6100_CS_PORT, W6100_CS_PIN);
}

inline void csDisable(void)
{
	GPIO_SetBits(W6100_CS_PORT, W6100_CS_PIN);
}

inline void resetAssert(void)
{
	GPIO_ResetBits(W6100_RESET_PORT, W6100_RESET_PIN);
}

inline void resetDeassert(void)
{
	GPIO_SetBits(W6100_RESET_PORT, W6100_RESET_PIN);
}

void EnterCris(void)
{
	NVIC_DisableIRQ(EXTI9_5_IRQn);
}

void ExitCris(void)
{
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void W6100Reset(void)
{
	int i,j,k;
	k=0;
	GPIO_ResetBits(W6100_RESET_PORT,W6100_RESET_PIN);
	//CoTickDelay(10);
	GPIO_SetBits(W6100_RESET_PORT,W6100_RESET_PIN);
}
