#include <stdio.h>
#include <string.h>

#include "stm32f4xx_conf.h"
#include "halConfiguration.h"
#include "usart.h"

#include "Types.h"
#include "netconfig.h"
#include "W6100RelFunctions.h"
#include "loopback.h"
#include "socket.h"

#include "AddressAutoConfig.h"

#define chip 6100
#define version 0.1

uint8 ch_status[MAX_SOCK_NUM] = { 0, }; /** 0:close, 1:ready, 2:connected */
CONFIG_MSG Config_Msg;
CHCONFIG_TYPE_DEF Chconfig_Type_Def;

uint8 txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};
uint8 rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};

uint8 Enable_DHCP    = 0;    //OFF;
uint16 i;
uint8_t MO_flag;

//////////////////////////////////////////////////////////////////////
/*******************MAC ADDRESS**************************/
//////////////////////////////////////////////////////////////////////
uint8 WIZ_MAC[6]     = {0x00, 0x08, 0xdc, 0xFE, 0x57, 0x99}; //MAC Address

//////////////////////////////////////////////////////////////////////
/*******************IPv6  ADDRESS**************************/
//////////////////////////////////////////////////////////////////////
//uint8 WIZ_LLA[16] = {0xFE, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//					 0x02, 0x08, 0xdc, 0xFF, 0xFE, 0xFE, 0x57, 0x99};

uint16_t WIZ_Port = 5000;

unsigned char W6100_AdrSet[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};

uint32_t presentTime = 0;
uint8 data_buf [TX_RX_MAX_BUF_SIZE]; // TX Buffer for applications

extern uint8_t DNS6_Address[16];
	
void Main_MENU(void);

void setMAC(uint8_t * WIZ_MAC);
void DADNS(void);
void AUTORS(void);
void Set_network(void);
void getAutoConfigAddress(void);
	
int main(void)
{
	GPIO_Configuration();
	USART_Configuration();
	NVIC_Configuration();
	SetSysTick_config(1000);

	#if _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SPI_
	reg_wizchip_spi_cbfunc(spiReadByte, spiWriteByte);
	reg_wizchip_cs_cbfunc(csEnable,csDisable);
	reg_wizchip_cris_cbfunc(EnterCris, ExitCris);
	#else
	reg_wizchip_cs_cbfunc(csEnable,csDisable);
	reg_wizchip_cris_cbfunc(EnterCris, ExitCris);
	#endif

	#if _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SPI_
	Set_SPI_Mode();
	#else
	Set_Indirect_Mode();
	#endif
	Interface_init();
	delay_ms(60);

	sysinit( W6100_AdrSet);
	
	while(!wizphy_getphylink());
	setMAC(WIZ_MAC);//Duplicate_Address_Detection(WIZ_MAC);MO_flag = Address_Auto_Config_RA(7);
//	setMAC();
	Duplicate_Address_Detection(WIZ_MAC);
	MO_flag = Address_Auto_Config_RA(7);
	printf("MO_flag : %x \r\n", MO_flag);
//		
//	Address_Auto_Config_Run(7, MO_flag);
	
	/*	
    //Set RTR and RCR register
    setRTR(2000);
    setRCR(5);

	while(!wizphy_getphylink()); // wait link

	DADNS();
	AUTORS();
	//Set_network();*/
	
	while(1)
	{
		Main_MENU();
		//loopback_tcps(0, 5000, data_buf, AF_INET6);
	}
}

void Main_MENU(void)
{   
    uint8_t num = 0;
    uint8_t uart_menu;
    uint8 tmp_array[16];
	char str[12];
    uint8_t MACA[6];
    
    printf("\r\n");
	printf("\r\n");
    printf("////-----------------------------------------------////\r\n");
    printf("////-----------------------------------------------////\r\n");
    printf("////WIZnet CO.LTD W%d TEST FIRMWARE Version %.2f ////\r\n", chip, version);
    printf("////-----------------------------------------------////\r\n");
    printf("////-----MENU--------------------------------------////\r\n");
    printf("////-----------------------------------------------////\r\n");
    printf("////--%2d : set MAC Address ------------------------////\r\n", num++);
    printf("////--%2d : Duplicate_Address_Detection ------------////\r\n", num++);
    printf("////--%2d : Address_Auto_Config_RA -----------------////\r\n", num++);
    printf("////--%2d : Address_Auto_Config_stateless_DHCP -----////\r\n", num++);
	printf("////--%2d : Address_Auto_Config_stateful_DHCP ------////\r\n", num++);
    printf("////--%2d : get My Address -------------------------////\r\n", num++);
	printf("////--%2d : loopback_tcps --------------------------////\r\n", num++);
    printf("////-----------------------------------------------////\r\n");
    printf("////-----------------------------------------------////\r\n");
    printf("////-----------------------------------------------////\r\n");
    printf("\r\n");
    printf("\r\n");
    printf("\r\n");
    printf("INPUT COMMAND : ");
    
    while(1)
    {
        uart_menu = UartGetc();
        if( uart_menu != '\0')
            break;
    }
    
    if ( uart_menu=='0' )
    {
        printf("%c is Set MAC!!\r\n", uart_menu);
		setMAC(WIZ_MAC);
	}
    
    else if ( uart_menu=='1' )
    {
        printf("%c is Duplicate_Address_Detection TEST!!\r\n", uart_menu);
        Duplicate_Address_Detection(WIZ_MAC);
    }
    
    else if( uart_menu=='2' )
    {
        printf("%c is Address_Auto_Config_RA TEST!!\r\n", uart_menu);
        MO_flag = Address_Auto_Config_RA(7);
		if(MO_flag == SLAAC_DHCP6)
		{
			printf("you need get DNS address. menu is number 3 stateless DHCPv6 \r\n");
		}
		else if(MO_flag == SFAAC_DHCP6)
		{
			printf("you need get IP,DNS address. menu is number 4 stateful DHCPv6 \r\n");
		}
		else
		{
			getGUAR(tmp_array);
			printf("your Global IP is %x%x:%x%x:%x%x:%x%x:%x%x:%x%x:%x%x:%x%x \r\n", tmp_array[ 0], tmp_array[ 1], tmp_array[ 2], tmp_array[ 3], 
																					 tmp_array[ 4], tmp_array[ 5], tmp_array[ 6], tmp_array[ 7], 
																				     tmp_array[ 8], tmp_array[ 9], tmp_array[10], tmp_array[11], 
																					 tmp_array[12], tmp_array[13], tmp_array[14], tmp_array[15]);
		}
    }
    else if ( uart_menu=='3' )
    {
        printf("%c is Address_Auto_Config_Stateless_DHCP TEST!!\r\n", uart_menu);
        Address_Auto_Config_SLDHCP(7);
    }
	
    else if ( uart_menu=='4' )
    {
        printf("%c is Address_Auto_Config_Stateful_DHCP TEST!!\r\n", uart_menu);
        Address_Auto_Config_SFDHCP(7);
    }
	
	else if ( uart_menu=='5' )
    {
        printf("%c is check your address setting TEST!!\r\n", uart_menu);
        getAutoConfigAddress();
    }
	
    else if ( uart_menu=='6' )
    {
        printf("%c is loopback_tcs TEST!!\r\n", uart_menu);
		while(1)
        {
			//loopback_tcps(0, 5000, data_buf, AF_INET6);
			loopback_udps(0, data_buf, 5000, AF_INET6);
		}
    }
	
    else
    {
        printf("%c is COMMAND ERROR!!\r\n", uart_menu);
    }
}

void setMAC(uint8_t * WIZ_MAC)
{
	uint8_t tmp[6];
	NETCFG_UNLOCK();
	setSHAR(WIZ_MAC);
	NETCFG_LOCK();
	getSHAR(tmp);
	printf("Mac address : %.2x:%.2x:%.2x:%.2x:%.2x:%.2x \r\n",tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5]);
}

void getAutoConfigAddress(void)
{
	uint8_t tmp[16];
	
	getSHAR(tmp);
	printf("Mac address : %.2x:%.2x:%.2x:%.2x:%.2x:%.2x \r\n",tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5]);
	getLLAR(tmp);
	printf("your Link Local IP is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", tmp[ 0], tmp[ 1], tmp[ 2], tmp[ 3], 
																												 tmp[ 4], tmp[ 5], tmp[ 6], tmp[ 7], 
																												 tmp[ 8], tmp[ 9], tmp[10], tmp[11], 
																												 tmp[12], tmp[13], tmp[14], tmp[15]);
	getGUAR(tmp);
	printf("your Global IP is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", tmp[ 0], tmp[ 1], tmp[ 2], tmp[ 3], 
																											 tmp[ 4], tmp[ 5], tmp[ 6], tmp[ 7], 
																											 tmp[ 8], tmp[ 9], tmp[10], tmp[11], 
																											 tmp[12], tmp[13], tmp[14], tmp[15]);
	getGA6R(tmp);
	printf("your Gateway IP is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", tmp[ 0], tmp[ 1], tmp[ 2], tmp[ 3], 
																											  tmp[ 4], tmp[ 5], tmp[ 6], tmp[ 7], 
																											  tmp[ 8], tmp[ 9], tmp[10], tmp[11], 
																											  tmp[12], tmp[13], tmp[14], tmp[15]);	
	
	printf("your DNSv6 is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", DNS6_Address[ 0], DNS6_Address[ 1], DNS6_Address[ 2], DNS6_Address[ 3], 
																										 DNS6_Address[ 4], DNS6_Address[ 5], DNS6_Address[ 6], DNS6_Address[ 7], 
																										 DNS6_Address[ 8], DNS6_Address[ 9], DNS6_Address[10], DNS6_Address[11], 
																										 DNS6_Address[12], DNS6_Address[13], DNS6_Address[14], DNS6_Address[15]);
}