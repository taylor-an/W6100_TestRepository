#include <stdio.h>
#include "loopback.h"
#include "socket.h"
#include "wizchip_conf.h"

#if LOOPBACK_MODE == LOOPBACK_MAIN_NOBLCOK

static uint16_t j=0;
static uint16_t any_port = 	50000;

int32_t loopback_tcps(uint8_t sn, uint16_t port, uint8_t* buf, uint8_t loopback_mode)
    {
       int32_t ret;
       uint16_t size = 0, sentsize=0;
       int8_t status,inter;
       uint8_t tmp;
       uint32_t recevied_size, remained_size;
	   uint8_t arg_tmp8;


    #ifdef _LOOPBACK_DEBUG_
       uint8_t dst_ip[16];
       uint16_t dst_port;
    #endif
       getsockopt(sn, SO_STATUS,&status);
       switch(status)
       {
          case SOCK_ESTABLISHED :
        	  ctlsocket(sn,CS_GET_INTERRUPT,&inter);
             if(inter & Sn_IR_CON)
             {
    #ifdef _LOOPBACK_DEBUG_
            	 getsockopt(sn,SO_DESTIP,dst_ip);
    			if(loopback_mode == AS_IPV6)
                {

                    printf("Peer IP : %.2X%.2X:%.2X%.2X:%2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X:%.2X%.2X\r\n",
                    dst_ip[0], dst_ip[1], dst_ip[2], dst_ip[3], dst_ip[4], dst_ip[5], dst_ip[6], dst_ip[7],
                    dst_ip[8], dst_ip[9], dst_ip[10], dst_ip[11], dst_ip[12], dst_ip[13], dst_ip[14], dst_ip[15]);
                }
                else
                {
                    //getSn_DIPR(sn,dst_ip);
                    printf("Peer IP : %.3d.%.3d.%.3d.%.3d\r\n",
                    dst_ip[0], dst_ip[1], dst_ip[2], dst_ip[3]);
                }
    			 getsockopt(sn,SO_DESTPORT,&dst_port);
    			 printf("Peer Port : %d\r\n", dst_port);
    #endif
					 arg_tmp8 = Sn_IR_CON;
    			 ctlsocket(sn,CS_CLR_INTERRUPT,&arg_tmp8);
             }
			  getsockopt(sn,SO_RECVBUF,&recevied_size);
			  if(recevied_size > 0){
				  if(recevied_size > DATA_BUF_SIZE) recevied_size = DATA_BUF_SIZE;
				  ret = recv(sn, buf, recevied_size);

				  if(ret <= 0) return ret;      // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.
				  recevied_size = (uint16_t) ret;
				  sentsize = 0;

					while(recevied_size != sentsize)
					{
						ret = send(sn, buf+sentsize, recevied_size-sentsize);
						if(ret < 0)
						{
							close(sn);
							return ret;
						}
						sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
					}
			 }
             break;
          case SOCK_CLOSE_WAIT :
    #ifdef _LOOPBACK_DEBUG_
             //printf("%d:CloseWait\r\n",sn);
    #endif
        	 getsockopt(sn, SO_RECVBUF, &recevied_size);
    		 if(recevied_size > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
             {
    			if(recevied_size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
    			ret = recv(sn, buf, recevied_size);

    			if(ret <= 0) return ret;      // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.
    			recevied_size = (uint16_t) ret;
    			sentsize = 0;

    			while(recevied_size != sentsize)
    			{
    				ret = send(sn, buf+sentsize, recevied_size-sentsize);
    				if(ret < 0)
    				{
    					close(sn);
    					return ret;
    				}
    				sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
    			}
             }
             if((ret = disconnect(sn)) != SOCK_OK) return ret;
    #ifdef _LOOPBACK_DEBUG_
             printf("%d:Socket Closed\r\n", sn);
    #endif
             break;
          case SOCK_INIT :
    #ifdef _LOOPBACK_DEBUG_
        	 printf("%d:Listen, TCP server loopback, port [%d]\r\n", sn, port);
    #endif
             if( (ret = listen(sn)) != SOCK_OK) return ret;
             break;
          case SOCK_CLOSED:
    #ifdef _LOOPBACK_DEBUG_
             printf("%d:TCP server loopback start\r\n",sn);
    #endif
        	 switch(loopback_mode){
        	   case AS_IPV4:
        		    tmp = socket(sn,Sn_MR_TCP4,any_port++,0x00);
        	   break;
        	   case AS_IPV6:
        		   tmp = socket(sn,Sn_MR_TCP6,any_port++,0x00);
               break;
        	   case AS_IPDUAL:
        		   tmp = socket(sn,Sn_MR_TCPD,any_port++,0x00);
        	   break;
        	   default:
        	   break;
        	 }
    		 if(tmp != sn)    /* reinitialize the socket */
             {
    #ifdef _LOOPBACK_DEBUG_
                 printf("%d : Fail to create socket.\r\n",sn);
    #endif
                 return SOCKERR_SOCKNUM;
             }
    #ifdef _LOOPBACK_DEBUG_
            printf("%d:Socket opened\r\n",sn);
    #endif
             break;
          default:
             break;
       }
       return 1;
    }


int32_t loopback_tcpc(uint8_t sn, uint8_t* buf, uint8_t* destip, uint16_t destport, uint8_t loopback_mode)
    {
       int32_t ret; // return value for SOCK_ERRORs
       uint16_t size = 0, sentsize=0;
       uint8_t status,inter,addr_len;
       uint32_t recevied_size, remained_size;
       uint8_t tmp;
	   uint8_t arg_tmp8;
		wiz_IPAddress destinfo;


       // Socket Status Transitions
       // Check the W6100 Socket n status register (Sn_SR, The 'Sn_SR' controlled by Sn_CR command or Packet send/recv status)
       getsockopt(sn,SO_STATUS,&status);
       switch(status)
       {
          case SOCK_ESTABLISHED :
        	 ctlsocket(sn,CS_GET_INTERRUPT,&inter);
             if(inter & Sn_IR_CON)	// Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
             {
    #ifdef _LOOPBACK_DEBUG_
    			printf("%d:Connected to - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
    #endif
				arg_tmp8 = Sn_IR_CON;
    		ctlsocket(sn,CS_CLR_INTERRUPT,&arg_tmp8);// this interrupt should be write the bit cleared to '1'

             }

             //////////////////////////////////////////////////////////////////////////////////////////////
             // Data Transaction Parts; Handle the [data receive and send] process
             //////////////////////////////////////////////////////////////////////////////////////////////
             getsockopt(sn,SO_RECVBUF,&recevied_size);
    		 if(recevied_size > 0) // Sn_RX_RSR: Socket n Received Size Register, Receiving data length
             {
    			if(recevied_size > DATA_BUF_SIZE) recevied_size = DATA_BUF_SIZE; // DATA_BUF_SIZE means user defined buffer size (array)
    			ret = recv(sn, buf, recevied_size); // Data Receive process (H/W Rx socket buffer -> User's buffer)

    			if(ret <= 0) return ret; // If the received data length <= 0, receive failed and process end
    			recevied_size = (uint16_t) ret;
    			sentsize = 0;

    			// Data sentsize control
    			while(recevied_size != sentsize)
    			{
    				ret = send(sn, buf+sentsize, recevied_size-sentsize); // Data send process (User's buffer -> Destination through H/W Tx socket buffer)
    				if(ret < 0) // Send Error occurred (sent data length < 0)
    				{
    					close(sn); // socket close
    					return ret;
    				}
    				sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
    			}
             }
    		 //////////////////////////////////////////////////////////////////////////////////////////////
             break;

          case SOCK_CLOSE_WAIT :
    #ifdef _LOOPBACK_DEBUG_
             //printf("%d:CloseWait\r\n",sn);
    #endif
        	 getsockopt(sn,SO_RECVBUF,&recevied_size);
    	     if((recevied_size = getSn_RX_RSR(sn)) > 0) // Sn_RX_RSR: Socket n Received Size Register, Receiving data length
             {
    			if(recevied_size > DATA_BUF_SIZE) recevied_size = DATA_BUF_SIZE; // DATA_BUF_SIZE means user defined buffer size (array)
    			ret = recv(sn, buf, recevied_size); // Data Receive process (H/W Rx socket buffer -> User's buffer)

    			if(ret <= 0) return ret; // If the received data length <= 0, receive failed and process end
    			recevied_size = (uint16_t) ret;
    			sentsize = 0;

    			// Data sentsize control
    			while(recevied_size != sentsize)
    			{
    				ret = send(sn, buf+sentsize, recevied_size-sentsize); // Data send process (User's buffer -> Destination through H/W Tx socket buffer)
    				if(ret < 0) // Send Error occurred (sent data length < 0)
    				{
    					close(sn); // socket close
    					return ret;
    				}
    				sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
    			}
             }
             if((ret=disconnect(sn)) != SOCK_OK) return ret;
    #ifdef _LOOPBACK_DEBUG_
             printf("%d:Socket Closed\r\n", sn);
    #endif
             break;

          case SOCK_INIT :
    #ifdef _LOOPBACK_DEBUG_
        	 printf("%d:Try to connect to the %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
    #endif					
        	 getsockopt(sn,SO_DESTIP,&destinfo);
					addr_len = destinfo.len;
              if(addr_len == 16){
                  ret = connect(sn, destip, destport, 16); /* Try to connect to TCP server(Socket, DestIP, DestPort) */
              }
              else{
                 ret = connect(sn, destip, destport, 4);
              }
        	  if( ret != SOCK_OK) return ret;	//	Try to TCP connect to the TCP server (destination)
          break;

          case SOCK_CLOSED:
        	 switch(loopback_mode){
        	   case AS_IPV4:
        		    tmp = socket(sn,Sn_MR_TCP4,any_port++,0x00);
        	   break;
        	   case AS_IPV6:
        		   tmp = socket(sn,Sn_MR_TCP6,any_port++,0x00);
               break;
        	   case AS_IPDUAL:
        		   tmp = socket(sn,Sn_MR_TCPD,any_port++,0x00);
        	   break;
        	   default:
        	   break;
        	 }

   		     if(tmp != sn){    /* reinitialize the socket */
    #ifdef _LOOPBACK_DEBUG_
                 printf("%d : Fail to create socket.\r\n",sn);
    #endif
                 return SOCKERR_SOCKNUM;
             }

    #ifdef _LOOPBACK_DEBUG_
        	 //printf("%d:TCP client loopback start\r\n",sn);
             //printf("%d:Socket opened\r\n",sn);
    #endif
             break;
          default:
             break;
       }
       return 1;
    }
int32_t loopback_udps(uint8_t sn, uint8_t* buf, uint16_t port, uint8_t loopback_mode){
           int8_t status;
           static uint8_t destip[16] = {0,};
           static uint16_t destport;
           uint8_t pack_info;
           uint8_t addr_len;
           uint16_t ret;
           uint32_t recevied_size, remained_size;
           uint16_t size, sentsize;

           getsockopt(sn, SO_MODE,&status);
           switch(status)
           {
           case SOCK_UDP4:
           case SOCK_UDP6:
           case SOCK_UDPD:
           	getsockopt(sn, SO_RECVBUF, &recevied_size);
        	if(recevied_size > DATA_BUF_SIZE) recevied_size = DATA_BUF_SIZE;
                 if(recevied_size>0)
                 {
               	     ret = recvfrom(sn, buf, recevied_size, (uint8_t*)&destip, (uint16_t*)&destport, &addr_len);

               	     if(ret <= 0)
               	    	 return ret;
               	     recevied_size = (uint16_t) ret;
       				 sentsize = 0;
       				 while(sentsize != recevied_size){
       					ret = sendto(sn, buf+sentsize, recevied_size-sentsize, destip, destport, addr_len);
       					if(ret < 0)
       					{
       						 return ret;
       					}
       					sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
       				 }
                 }
                 break;
           case SOCK_CLOSED:
           	switch(loopback_mode)
               {
               case AS_IPV4:
                   socket(sn,Sn_MR_UDP4, port, 0x00);
                   break;
               case AS_IPV6:
                   socket(sn,Sn_MR_UDP6, port,0x00);
                   break;
               case AS_IPDUAL:
                   socket(sn,Sn_MR_UDPD, port, 0x00);
                   break;
                }
           	   printf("%d:Opened, UDP loopback, port [%d]\r\n", sn, port);

           }

       }


#endif
