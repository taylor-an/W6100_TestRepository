#include <stdio.h>
#include <string.h>

#include "stm32f4xx_conf.h"
#include "halConfiguration.h"
#include "usart.h"

#include "Types.h"
#include "netconfig.h"
#include "W6100RelFunctions.h"
#include "loopback.h"
#include "dhcpv6.h"

#define MY_MAX_DHCP_RETRY   3

uint8 ch_status[MAX_SOCK_NUM] = { 0, }; /** 0:close, 1:ready, 2:connected */
CONFIG_MSG Config_Msg;
CHCONFIG_TYPE_DEF Chconfig_Type_Def;

uint8 txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};
uint8 rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};

uint8 Enable_DHCP    = 0;    //OFF;
uint16 i;

//////////////////////////////////////////////////////////////////////
/*******************MAC ADDRESS**************************/
//////////////////////////////////////////////////////////////////////
uint8 WIZ_MAC[6]     = {0x00, 0x08, 0xdc, 0xFE, 0x57, 0x99}; //MAC Address

//////////////////////////////////////////////////////////////////////
/*******************IPv4  ADDRESS**************************/
//////////////////////////////////////////////////////////////////////
uint8 WIZ_IP[4]      = {192, 168, 77, 99};                   //IP Address
uint8 WIZ_realIP[4]      = {222,98,173,209};
uint8 WIZ_SubNet[4]  = {255, 255, 255, 0};                   //SubnetMask Address

#ifdef FOR_TEST_PPPoE
uint8 WIZ_GateWay[4] = {192,168,55,1};                    //Gateway Address
#else
uint8 WIZ_GateWay[4] = {222,98,173,254};                    //Gateway Address
#endif

uint16 WIZ_My_PORT = 15000;                                 //DST_IP port

uint8 WIZ_Dest_IP_virtual[4] = {192, 168, 177, 177};                  //DST_IP Address
uint8 WIZ_Dest_IP_real[4] = {222, 98, 173, 200};                  //DST_IP Address
uint8 WIZ_Dest_IP_Google[4] = {216, 58, 200, 174};                  //DST_IP Address
uint8 WIZ_Dest_IP_Test[4] = {192,168,177,200};


uint8 mcastipv4_0[4] ={239,1,2,3};
uint8 mcastipv4_1[4] ={239,1,2,4};
uint8 mcastipv4_2[4] ={239,1,2,5};
uint8 mcastipv4_3[4] ={239,1,2,6};

uint16 WIZ_Dest_PORT = 15000;                                 //DST_IP port

//////////////////////////////////////////////////////////////////////
/*******************IPv6  ADDRESS**************************/
//////////////////////////////////////////////////////////////////////

uint8_t Zero_IP[16] = {0x00, };

uint8 WIZ_LLA[16] = {0xfe,0x80, 0x00,0x00,
                     0x00,0x00, 0x00,0x00,
                     0x02,0x08, 0xdc,0xff,
                     0xfe,0x57, 0x57, 0x57
                    };

uint8 WIZ_GUA[16] = {0,};//{0x20,0x01,0x02,0xb8,
//                     0x00,0x10,0x00,0x01,
//                     0x02,0x08, 0xdc,0xff,
//                     0xfe,0x57, 0x57, 0x57
//                    };

uint8 WIZ_6SubNet[16] = {0xff,0xff,0xff,0xff,
                         0xff,0xff,0xff,0xff,
                         0x00,0x00,0x00, 0x00,
                         0x00,0x00,0x00,0x00
                        };

uint8 WIZ_GateWay6[16] = {0xfe, 0x80, 0x00,0x00,
                          0x00,0x00,0x00,0x00,
                          0x02,0x00, 0x87,0xff,
                          0xfe,0x08, 0x4c,0x81
                         };

uint8_t DestIP6_L[16] = { 0xFE, 0x80, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00,
                                          0xd2, 0x50, 0x99, 0xff,
                                          0xfe, 0x62, 0xb0, 0xc6  };

uint8_t DestIP6_G[16] = { 0x20, 0x01, 0x02, 0xb8,
                                         0x00, 0x10, 0x00, 0x01,
                                          0xd2, 0x50, 0x99, 0xff,
                                          0xfe, 0x62, 0xb0, 0xc6  };

uint8_t DestIP6_GOOGLE[16] = { 0x24, 0x04, 0x68, 0x00,
                                                 0x40, 0x0a, 0x08, 0x06,
                                                 0x00, 0x00, 0x00, 0x00,
                                                 0x00, 0x00, 0x20, 0x04 };

uint8_t MultiIP6_SMA[16] = {0xFF, 0x02, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x01,
                                            0xFF, 0x62, 0xb0, 0xc6 };

uint8_t MultiIP6_ANM[16] = {0xFF, 0x02, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00, 0x01 };

uint8_t MultiIP6_MGA_FF02[16] = {0xFF, 0x02, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0xAB, 0xCD, 0xEF };

uint8_t MultiIP6_MGA_FF0E[16] = {0xFF, 0x0E, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0xAB, 0xCD, 0xEF};

uint8_t test_buf[2048];
uint32_t my_dhcp_retry = 0;
															 
uint32 Multicast_Port = 15000;
unsigned char W6100_AdrSet[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};

uint32_t presentTime = 0;
uint8 data_buf [TX_RX_MAX_BUF_SIZE]; // TX Buffer for applications

void Set_network(void);
void DAD_TEST(uint8_t *TargetAddr);

int main(void)
{
	uint8_t tmp[8];
    uint32_t toggle = 1;

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

	Set_network();
    sysinit( W6100_AdrSet);

	DAD_TEST(WIZ_LLA);

	DHCP_init(2, test_buf);

	while(1)
	{
        switch(DHCP_run())
        {
            case DHCP_IP_ASSIGN:
            case DHCP_IP_CHANGED:
                /* If this block empty, act with default_ip_assign & default_ip_update  */
                //
                // This example calls the registered 'my_ip_assign' in the two case.
                //
                // Add to ...
                //
                //
                toggle = 1;
                if(toggle)
                {
                    getGAR(tmp);  printf("> DHCP GW : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                    getSUBR(tmp); printf("> DHCP SN : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                    getSIPR(tmp); printf("> DHCP IP : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                    toggle = 0;
                    close(0); /* 
								If renewal IP address was defferent previous IP address, 
								socket becomes to disconnect or close for new connection.
								*/
                }  						
                break;
            case DHCP_IP_LEASED:
                //
                if(toggle)
                {
                    getSHAR(tmp);	printf("MAC ADDRESS : %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\r\n",tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5]);    
                    getGAR(tmp);  printf("> DHCP GW : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                    getSUBR(tmp); printf("> DHCP SN : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                    getSIPR(tmp); printf("> DHCP IP : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                    toggle = 0;
                }
                // TO DO YOUR NETWORK APPs.
                loopback_tcps(0, 5000, test_buf, AF_INET);
                break;

            case DHCP_FAILED:
                /* ===== Example pseudo code =====  */
                // The below code can be replaced your code or omitted.
                // if omitted, retry to process DHCP
                my_dhcp_retry++;
                if(my_dhcp_retry > MY_MAX_DHCP_RETRY)
                {	
#if DEBUG_MODE != DEBUG_NO
                    printf(">> DHCP %d Failed\r\n",my_dhcp_retry);
#endif
                    my_dhcp_retry = 0;
                    DHCP_stop();      // if restart, recall DHCP_init()
                }
                break;
            default:
                break;
        }	


    }
}

void Set_network(void)
{
    uint8 tmp_array[16];
    uint8 i;

    //delay_ms(1000);

    // MAC ADDRESS
    for (i = 0 ; i < 6; i++) Config_Msg.Mac[i] = WIZ_MAC[i];
    // Local IP ADDRESS
    Config_Msg.Lip[0] = WIZ_IP[0]; Config_Msg.Lip[1] = WIZ_IP[1]; Config_Msg.Lip[2] = WIZ_IP[2]; Config_Msg.Lip[3] = WIZ_IP[3];
    // GateWay ADDRESS
    Config_Msg.Gw[0] = WIZ_GateWay[0]; Config_Msg.Gw[1] = WIZ_GateWay[1]; Config_Msg.Gw[2] = WIZ_GateWay[2]; Config_Msg.Gw[3] = WIZ_GateWay[3];
    // Subnet Mask ADDRESS
    Config_Msg.Sub[0] = WIZ_SubNet[0]; Config_Msg.Sub[1] = WIZ_SubNet[1]; Config_Msg.Sub[2] = WIZ_SubNet[2]; Config_Msg.Sub[3] = WIZ_SubNet[3];

		//WIZCHIP_READ(WZ_RTR0);
		//printf("\r\n before NETCFGLOCK : %x\r\n", WIZCHIP_READ(WZ_NETCFG_LOCKR));
		//delay_ms(1000);
    // Network Config Register Unlock
    //printf("%.2x \r\n", getVERR());
	//_CS_Low;
	//_CS_High;
	//_CS_Low;
	//_CS_High;
		printf("VERSION(%x) = %.2x \r\n", VER,getVER());
		//NETCFG_UNLOCK();
	  NETCFG_UNLOCK();
		//delay_ms(1000);
		//printf("\r\n after NETCFGLOCK : %x\r\n", WIZCHIP_READ(WZ_NETCFG_LOCKR));

    //-- Set MAC Address
    setSHAR(Config_Msg.Mac);

    //-- Set Addresses for IPv4 address
    setSUBR(Config_Msg.Sub);
    setGAR(Config_Msg.Gw);
    setSIPR(Config_Msg.Lip);

    //-- Set Addresses for IPv6 address
    //setGW6R(WIZ_GateWay6);
    //setLLAR(WIZ_LLA);
		//setLLAR(DestIP6_L);
    //setGUAR(WIZ_GUA);
    setSUB6R(WIZ_6SubNet);
    setGA6R(WIZ_GateWay6);

    // Set DHCP
    Config_Msg.DHCP = Enable_DHCP;

    //Set RTR and RCR register
    setRTR(2000);
    setRCR(5);

    setSLRTR(2000);
    setSLRCR(5);
    //Init. TX & RX Memory size
    //sysinit(txsize, rxsize);

    printf("\r\n----------------------------------------- \r\n");
    printf("W6100E01-M4                       \r\n");
    printf("Network Configuration Information \r\n");
    printf("----------------------------------------- ");


    getSHAR(tmp_array);
    printf("\r\nMAC : %.2X.%.2X.%.2X.%.2X.%.2X.%.2X", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3],tmp_array[4],tmp_array[5]);

    getSIPR (tmp_array);
    printf("\r\nIP : %d.%d.%d.%d", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3]);

    getSUBR(tmp_array);
    printf("\r\nSN : %d.%d.%d.%d", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3]);

    getGAR(tmp_array);
    printf("\r\nGW : %d.%d.%d.%d\r\n", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3]);

    getGA6R(tmp_array);
    printf("\r\nGW6 : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X",
                                   tmp_array[0], tmp_array[1], tmp_array[2], tmp_array[3],
                                   tmp_array[4], tmp_array[5], tmp_array[6], tmp_array[7],
                                   tmp_array[8], tmp_array[9], tmp_array[10],tmp_array[11],
                                   tmp_array[12],tmp_array[13],tmp_array[14],tmp_array[15]);

    getLLAR(tmp_array);
    printf("\r\nLLA : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X",
                                   tmp_array[0], tmp_array[1], tmp_array[2], tmp_array[3],
                                   tmp_array[4], tmp_array[5], tmp_array[6], tmp_array[7],
                                   tmp_array[8], tmp_array[9], tmp_array[10],tmp_array[11],
                                   tmp_array[12],tmp_array[13],tmp_array[14],tmp_array[15]);
    getGUAR(tmp_array);
    printf("\r\nGUA : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X",
                                   tmp_array[0], tmp_array[1], tmp_array[2], tmp_array[3],
                                   tmp_array[4], tmp_array[5], tmp_array[6], tmp_array[7],
                                   tmp_array[8], tmp_array[9], tmp_array[10],tmp_array[11],
                                   tmp_array[12],tmp_array[13],tmp_array[14],tmp_array[15]);

    getSUB6R(tmp_array);
    printf("\r\n6SM : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X",
                                   tmp_array[0], tmp_array[1], tmp_array[2], tmp_array[3],
                                   tmp_array[4], tmp_array[5], tmp_array[6], tmp_array[7],
                                   tmp_array[8], tmp_array[9], tmp_array[10],tmp_array[11],
                                   tmp_array[12],tmp_array[13],tmp_array[14],tmp_array[15]);


     printf("\r\nNETCFGLOCK : %x\r\n", WIZCHIP_READ(NETLCKR));
}

void DAD_TEST(uint8_t *TargetAddr)
{
    uint8_t flags;
    uint8_t tmp_array[16];
        
    setSLPIP6R(TargetAddr); 
    
    getSLPIP6R(tmp_array);
    printf("Target IP address: %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n", 
            tmp_array[0], tmp_array[1], tmp_array[2], tmp_array[3], tmp_array[4], tmp_array[5], tmp_array[6], tmp_array[7],
			tmp_array[8], tmp_array[9], tmp_array[10], tmp_array[11], tmp_array[12], tmp_array[13], tmp_array[14], tmp_array[15]);
	   
    
    setSLRTR(0x2000);
    //setSLRCR(0x05);
	//printf("getSLRTR:%X\r\n",getSLRTR());
	//printf("getSLRCR:%X\r\n",getSLRCR());
    
	WIZCHIP_WRITE( SLIMR, 0x84); // Timeout & DAD_NS
	WIZCHIP_WRITE( SLCR,  0x04); //SLCMD_DAD_NS
    
    printf("WZ_SLIMR : %x\r\n",WIZCHIP_READ(SLIMR));
    printf("WZ_SLCR : %x\r\n",WIZCHIP_READ(SLCR));
    
    do
	{
		flags = WIZCHIP_READ( SLIR );
        printf(".");
	}
	while(flags == 0);
    printf("\r\n");
    
    printf("flags : %x\r\n",flags);
    
    WIZCHIP_WRITE(SLIRCLR, 0xFF); // SLIR Clear
    printf("WZ_SLIMR : %x\r\n",WIZCHIP_READ(SLIMR));
    
    if((flags&SLIR_TIOUT)==SLIR_TIOUT)
    {
        printf("DAD Success!!\r\n");
		NETCFG_UNLOCK();
        setLLAR(TargetAddr);
        
        printf("\r\n----------------------------------------- \r\n");         		
        printf("W6100E01-M4                       \r\n");        
        printf("Network Configuration Information \r\n");        
        printf("----------------------------------------- \r\n");         		
        
        getSHAR(tmp_array);
        printf("MAC : %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\r\n", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3],tmp_array[4],tmp_array[5]);
        
        getSIPR (tmp_array);
        printf("IP  : %d.%d.%d.%d\r\n", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3]);
        
        getSUBR(tmp_array);
        printf("SN  : %d.%d.%d.%d\r\n", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3]);
        
        getGAR(tmp_array);
        printf("GW  : %d.%d.%d.%d\r\n", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3]);

        getLLAR(tmp_array);
        printf("LLA : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n", 
                tmp_array[0], tmp_array[1], tmp_array[2], tmp_array[3], tmp_array[4], tmp_array[5], tmp_array[6], tmp_array[7],
                tmp_array[8], tmp_array[9], tmp_array[10], tmp_array[11], tmp_array[12], tmp_array[13], tmp_array[14], tmp_array[15]);
        
        getGUAR(tmp_array);
        printf("GUA : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n", 
                tmp_array[0], tmp_array[1], tmp_array[2], tmp_array[3], tmp_array[4], tmp_array[5], tmp_array[6], tmp_array[7],
                tmp_array[8], tmp_array[9], tmp_array[10], tmp_array[11], tmp_array[12], tmp_array[13], tmp_array[14], tmp_array[15]);
    }
    else if((flags&SLIR_NS)==SLIR_NS)
    {
        printf("DAD Failed!!\r\n");
        
        printf("\r\n----------------------------------------- \r\n");         		
        printf("W6100E01-M4                       \r\n");        
        printf("Network Configuration Information \r\n");        
        printf("----------------------------------------- \r\n");         		
        
        getSHAR(tmp_array);
        printf("MAC : %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\r\n", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3],tmp_array[4],tmp_array[5]);
        
        getSIPR (tmp_array);
        printf("IP  : %d.%d.%d.%d\r\n", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3]);
        
        getSUBR(tmp_array);
        printf("SN  : %d.%d.%d.%d\r\n", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3]);
        
        getGAR(tmp_array);
        printf("GW  : %d.%d.%d.%d\r\n", tmp_array[0],tmp_array[1],tmp_array[2],tmp_array[3]);

        getLLAR(tmp_array);
        printf("LLA : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n", 
                tmp_array[0], tmp_array[1], tmp_array[2], tmp_array[3], tmp_array[4], tmp_array[5], tmp_array[6], tmp_array[7],
                tmp_array[8], tmp_array[9], tmp_array[10], tmp_array[11], tmp_array[12], tmp_array[13], tmp_array[14], tmp_array[15]);
        
        getGUAR(tmp_array);
        printf("GUA : %.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n", 
                tmp_array[0], tmp_array[1], tmp_array[2], tmp_array[3], tmp_array[4], tmp_array[5], tmp_array[6], tmp_array[7],
                tmp_array[8], tmp_array[9], tmp_array[10], tmp_array[11], tmp_array[12], tmp_array[13], tmp_array[14], tmp_array[15]);
    }
}
