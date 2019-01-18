#include "usart.h"


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configure Open_USART 
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void USART_Configuration(void)
{                                               
  USART_InitTypeDef USART_InitStructure; 
/*
     USARTx configured as follow:
         - BaudRate = 115200 baud  
         - Word Length = 8 Bits
         - One Stop Bit
         - No parity
         - Hardware flow control disabled (RTS and CTS signals)
         - Receive and transmit    
 */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(Open_USART, &USART_InitStructure);
  /* Enable the Open_USART Transmit interrupt: this interrupt is generated when the 
     Open_USART transmit data register is empty */
  //USART_ITConfig(Open_USART,USART_IT_RXNE,ENABLE);
  USART_Cmd(Open_USART, ENABLE);

}


/* Use no semihosting */
#if 1
//#pragma import(__use_no_semihosting)
struct __FILE
{  
    int handle;
};
FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}
#endif

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(Open_USART, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(Open_USART, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

uint8_t UartGetc(void)
{
    while( USART_GetFlagStatus(Open_USART, USART_FLAG_RXNE)==RESET);
    
    return USART_ReceiveData(Open_USART);
}

int UartGets(uint8_t *str, int size)
{
    uint8_t ch;
    
    while(size)
    {
        ch = UartGetc();
        if(ch != '\n') (*str) = ch;
        if(ch == '\r') break;
        
        str++;
        size--;
    }
    *str = 0;
    
    return size;
}




/* Use no semihosting */
#if 0
void USART_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel                   = Open_USART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
#endif
