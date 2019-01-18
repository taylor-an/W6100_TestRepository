#include <stdio.h>
#include <stdlib.h>
#include "HAL_Config.h"
#include "HALInit.h"
#include "wizchip_conf.h"
#include "inttypes.h"
#include "W6100RelFunctions.h"



wiz_NetInfo gWIZNETINFO = { .mac = {0x00,0x08,0xdc,0x57,0x57,0x20},
							.ip = {192,168,0,4},
							.sn = {255, 255, 255, 0},
							.gw = {192, 168, 0, 1},
							.dns = {168, 126, 63, 1},
							.dhcp = NETINFO_STATIC,
							.lla = {0xfe,0x80, 0x00,0x00,
								 0x00,0x00, 0x00,0x00,
								 0x02,0x08, 0xdc,0xff,
								 0xfe,0x57, 0x57,0x61},
				            .gua={0x20,0x01,0x02,0xb8,
								 0x00,0x10,0x00,0x01,
								 0x02,0x08,0xdc,0xff,
								 0xfe,0x57,0x57,0x61},
				            .sn6={0xff,0xff,0xff,0xff,
								 0xff,0xff,0xff,0xff,
								 0x00,0x00,0x00, 0x00,
								 0x00,0x00,0x00,0x00},
				            .gw6={0xfe, 0x80, 0x00,0x00,
								  0x00,0x00,0x00,0x00,
								  0x02,0x00, 0x87,0xff,
								  0xfe,0x08, 0x4c,0x81},
							};


uint8_t WIZ_Dest_IP[4] = {192, 168, 0, 232};                  //DST_IP Address


uint8_t DestIP6_L[16] = {0xfe,0x80, 0x00,0x00,
						  0x00,0x00, 0x00,0x00,
                          0x31,0x71,0x98,0x05,
                          0x70,0x24,0x4b,0xb1
						};

uint8_t DestIP6_G[16] = {0x20,0x01,0x02,0xb8,
                          0x00,0x10,0x00,0x01,
                          0x31,0x71,0x98,0x05,
                          0x70,0x24,0x4b,0xb1
                         };

uint8_t Router_IP[16]= {0xff,0x02,0x00,0x00,
                          0x00,0x00,0x00,0x00,
                          0x00,0x00,0x00,0x00,
                          0x00,0x00,0x00,0x02
                         };

uint8_t data_buf[2048];
void print_network_information(void);
void delay(unsigned int count);


int main(void)
{
	volatile int i;
	volatile int j,k;

	gpioInitialize();
	usartInitialize();
	timerInitialize();
	printf("System start.\r\n");




#if _WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SPI_
	/* SPI method callback registration */
	reg_wizchip_spi_cbfunc(spiReadByte, spiWriteByte);
	/* CS function register */
	reg_wizchip_cs_cbfunc(csEnable,csDisable);
#else
	/* Indirect bus method callback registration */
	//reg_wizchip_bus_cbfunc(busReadByte, busWriteByte);
#endif

#if _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_BUS_INDIR_
	FSMCInitialize();
#else
	spiInitailize();
#endif
	resetAssert();
	delay(10);
	resetDeassert();
	delay(10);

	NETUNLOCK();
	wizchip_setnetinfo(&gWIZNETINFO);
	W6100Initialze();


	printf("VERSION(%x) = %.2x \r\n", VER,getVER());
	print_network_information();


	while(1){
		loopback_tcps(0,5000, data_buf, AF_INET6);
		loopback_tcps(1,5001, data_buf, AF_INET);

	}
}

void delay(unsigned int count)
{
	int temp;
	temp = count + TIM2_gettimer();
	while(temp > TIM2_gettimer()){}
}


void print_network_information(void)
{
	wizchip_getnetinfo(&gWIZNETINFO);
	printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\n\r",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
	printf("IP address : %d.%d.%d.%d\n\r",gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
	printf("SN Mask	   : %d.%d.%d.%d\n\r",gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
	printf("Gate way   : %d.%d.%d.%d\n\r",gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
	printf("DNS Server : %d.%d.%d.%d\n\r",gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
	printf("LLA  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO.lla[0],gWIZNETINFO.lla[1],gWIZNETINFO.lla[2],gWIZNETINFO.lla[3],\
									gWIZNETINFO.lla[4],gWIZNETINFO.lla[5],gWIZNETINFO.lla[6],gWIZNETINFO.lla[7],\
									gWIZNETINFO.lla[8],gWIZNETINFO.lla[9],gWIZNETINFO.lla[10],gWIZNETINFO.lla[11],\
									gWIZNETINFO.lla[12],gWIZNETINFO.lla[13],gWIZNETINFO.lla[14],gWIZNETINFO.lla[15]);
	printf("GUA  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\n\r",gWIZNETINFO.gua[0],gWIZNETINFO.gua[1],gWIZNETINFO.gua[2],gWIZNETINFO.gua[3],\
									gWIZNETINFO.gua[4],gWIZNETINFO.gua[5],gWIZNETINFO.gua[6],gWIZNETINFO.gua[7],\
									gWIZNETINFO.gua[8],gWIZNETINFO.gua[9],gWIZNETINFO.gua[10],gWIZNETINFO.gua[11],\
									gWIZNETINFO.gua[12],gWIZNETINFO.gua[13],gWIZNETINFO.gua[14],gWIZNETINFO.gua[15]);
	printf("SN6  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\n\r",gWIZNETINFO.sn6[0],gWIZNETINFO.sn6[1],gWIZNETINFO.sn6[2],gWIZNETINFO.sn6[3],\
									gWIZNETINFO.sn6[4],gWIZNETINFO.sn6[5],gWIZNETINFO.sn6[6],gWIZNETINFO.sn6[7],\
									gWIZNETINFO.sn6[8],gWIZNETINFO.sn6[9],gWIZNETINFO.sn6[10],gWIZNETINFO.sn6[11],\
									gWIZNETINFO.sn6[12],gWIZNETINFO.sn6[13],gWIZNETINFO.sn6[14],gWIZNETINFO.sn6[15]);
	printf("GW6  : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",gWIZNETINFO.gw6[0],gWIZNETINFO.gw6[1],gWIZNETINFO.gw6[2],gWIZNETINFO.gw6[3],\
									gWIZNETINFO.gw6[4],gWIZNETINFO.gw6[5],gWIZNETINFO.gw6[6],gWIZNETINFO.gw6[7],\
									gWIZNETINFO.gw6[8],gWIZNETINFO.gw6[9],gWIZNETINFO.gw6[10],gWIZNETINFO.gw6[11],\
									gWIZNETINFO.gw6[12],gWIZNETINFO.gw6[13],gWIZNETINFO.gw6[14],gWIZNETINFO.gw6[15]);


}
