#include "halConfiguration.h"
#include "usart.h"


static __IO uint32_t TimingDelay;

void Interface_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    delay_ms(60); // rst signal wait...
    if ( GPIO_ReadOutputDataBit(SPI_ENABLE_Port, SPI_ENABLE_Pin) == Bit_RESET )  // SPI Mode
    {
        SPI_InitTypeDef SPI_InitStruct;
        /*
        *  Open_SPIx_SCK -> PB3 , Open_SPIx_MOSI -> PB4, Open_SPIx_SCK -> MISO
        */
        
        RCC_AHB1PeriphClockCmd(SPI_RCC_AHBPeriph,ENABLE);
        
				//printf("spi mode \r\n");
			
				GPIO_PinAFConfig(Open_SPIx_SCK_GPIO_PORT,  Open_SPIx_SCK_SOURCE,  Open_SPIx_MOSI_AF);
        GPIO_PinAFConfig(Open_SPIx_MISO_GPIO_PORT, Open_SPIx_MISO_SOURCE, Open_SPIx_MOSI_AF);
        GPIO_PinAFConfig(Open_SPIx_MOSI_GPIO_PORT, Open_SPIx_MOSI_SOURCE, Open_SPIx_MOSI_AF);

				GPIO_InitStructure.GPIO_Pin   = Open_SPIx_SCK_PIN | Open_SPIx_MISO_PIN | Open_SPIx_MOSI_PIN;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
				GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
				GPIO_Init(Open_SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

        RCC_AHB1PeriphClockCmd(RCC_GPIO_CS,ENABLE);
				
				GPIO_InitStructure.GPIO_Pin   = GPIO_PIN_CS;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
				GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
				GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
				GPIO_Init(GPIO_CS_PORT, &GPIO_InitStructure);
				
				GPIO_SetBits(GPIO_CS_PORT,GPIO_PIN_CS);

        RCC_APB2PeriphClockCmd(Open_RCC_APB2Periph_SPIx,ENABLE);

        SPI_I2S_DeInit(Open_SPIx);
        SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;
        SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b; 
       // SPI_InitStruct.SPI_CPOL              = SPI_CPOL_High;
				SPI_InitStruct.SPI_CPOL              = SPI_CPOL_Low;
        //SPI_InitStruct.SPI_CPHA              = SPI_CPHA_2Edge;
				SPI_InitStruct.SPI_CPHA              = SPI_CPHA_1Edge;
        SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft ;
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
        SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;
        SPI_InitStruct.SPI_CRCPolynomial     = 7;
        SPI_Init(Open_SPIx, &SPI_InitStruct);
        SPI_Cmd(Open_SPIx, ENABLE);        
    }
    else
    {
				FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
        FSMC_NORSRAMTimingInitTypeDef  p;
        /* Enable GPIOs clock */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE |RCC_AHB1Periph_GPIOF| RCC_AHB1Periph_GPIOG, ENABLE);

        /* Enable FSMC clock */
        RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
				
				//printf("indirect mode \r\n");
			
        /* GPIOD configuration */
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

        GPIO_Init(GPIOD, &GPIO_InitStructure);

        /* GPIOE configuration */
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 ;

        GPIO_Init(GPIOE, &GPIO_InitStructure);

        /* GPIOF configuration */
        GPIO_PinAFConfig(GPIOF, GPIO_PinSource0 , GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOF, GPIO_PinSource1 , GPIO_AF_FSMC);
        //GPIO_PinAFConfig(GPIOF, GPIO_PinSource2 , GPIO_AF_FSMC);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  ;//| GPIO_Pin_2 ;
        GPIO_Init(GPIOF, &GPIO_InitStructure);

        /* NOE and NWE configuration */  
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        /* NE1 configuration */
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        /* NBL0, NBL1 configuration */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
//        GPIO_Init(GPIOE, &GPIO_InitStructure); 
				
				FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, DISABLE);

        /*-- FSMC Configuration ------------------------------------------------------*/
        p.FSMC_AddressSetupTime = 0x5;
				//p.FSMC_AddressSetupTime = 0xc;
        p.FSMC_AddressHoldTime = 0x0;
        p.FSMC_DataSetupTime = 0x0a;
				//p.FSMC_DataSetupTime = 0x20;
        p.FSMC_BusTurnAroundDuration = 0x0;
        p.FSMC_CLKDivision = 0x0;
        p.FSMC_DataLatency = 0x0;
        p.FSMC_AccessMode = FSMC_AccessMode_A;

        FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
        FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
        FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
        FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
        FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
        FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
        FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
        FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
        FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
        FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
        FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
        FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
        FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
        FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;
        FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;

        FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

        /*!< Enable FSMC Bank1_SRAM1 Bank */
        FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
    }
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*
      * USER's LED
      */
    RCC_AHB1PeriphClockCmd(
                            LED0_RCC_AHBPeriph | 
                            LED1_RCC_AHBPeriph | 
                            LED2_RCC_AHBPeriph | 
                            LED3_RCC_AHBPeriph | 
                            LED4_RCC_AHBPeriph | 
                            LED5_RCC_AHBPeriph | 
                            INT_RCC_AHBPeriph    |
                            SPI_RCC_AHBPeriph,
                            ENABLE
                          );
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG,ENABLE);
    
    /* Configure PF6 PF7 PF8 PF9 in output pushpull mode */
    GPIO_InitStructure.GPIO_Pin   = LED0_Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(LED0_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = LED1_Pin;
    GPIO_Init(LED1_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = LED2_Pin;
    GPIO_Init(LED2_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = LED3_Pin;
    GPIO_Init(LED3_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = LED4_Pin;
    GPIO_Init(LED4_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = LED5_Pin;
    GPIO_Init(LED5_Port, &GPIO_InitStructure);
    
    /*MCO2 ¼³Á¤*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_MCO);
    
    RCC_MCO2Config(RCC_MCO2Source_SYSCLK,RCC_MCO2Div_1);
    
    /*
    #define RCC_MCO2Source_SYSCLK            ((uint32_t)0x00000000)
    #define RCC_MCO2Source_PLLI2SCLK         ((uint32_t)0x40000000)
    #define RCC_MCO2Source_HSE               ((uint32_t)0x80000000)
    #define RCC_MCO2Source_PLLCLK            ((uint32_t)0xC0000000)
    */
    
    GPIO_InitStructure.GPIO_Pin   = INT_Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(INT_Port, &GPIO_InitStructure);

    /*
    *  Open_USART_TX -> PA9 , Open_USART_RX -PA10
    */
    RCC_AHB1PeriphClockCmd(Open_USART_TX_GPIO_CLK,ENABLE);
    RCC_AHB1PeriphClockCmd(Open_USART_RX_GPIO_CLK,ENABLE);
    RCC_APB2PeriphClockCmd(Open_USART_CLK,ENABLE);

    GPIO_PinAFConfig(Open_USART_TX_GPIO_PORT, Open_USART_TX_SOURCE, Open_USART_TX_AF);
    GPIO_PinAFConfig(Open_USART_RX_GPIO_PORT, Open_USART_RX_SOURCE, Open_USART_RX_AF);

    GPIO_InitStructure.GPIO_Pin   = Open_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(Open_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = Open_USART_RX_PIN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(Open_USART_RX_GPIO_PORT, &GPIO_InitStructure);
}

void EXTI_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(INT_RCC_AHBPeriph, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = INT_Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(INT_Port, &GPIO_InitStructure);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);
    
    EXTI_InitStructure.EXTI_Line   = EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode  = EXTI_Mode_Interrupt;
	  //EXTI_InitStructure.EXTI_Mode  = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure); 

    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = Open_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);    
}


void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel                   = Open_USART_IRQn|Open_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Delay Function.
  * @param  nTime: Delay time length of 1ms unit.
  * @retval None
  */
void Delay_Tick(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

  /* Setup SysTick Timer for 1 msec interrupts.
     ------------------------------------------
    1. The SysTick_Config() function is a CMSIS function which configure:
       - The SysTick Reload register with value passed as function parameter.
       - Configure the SysTick IRQ priority to the lowest value (0x0F).
       - Reset the SysTick Counter register.
       - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
       - Enable the SysTick Interrupt.
       - Start the SysTick Counter.
       - static __INLINE uint32_t SysTick_Config(uint32_t ticks)
    
    2. You can change the SysTick Clock source to be HCLK_Div8 by calling the
       SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8) just after the
       SysTick_Config() function call. The SysTick_CLKSourceConfig() is defined
       inside the misc.c file.

    3. You can change the SysTick IRQ priority by calling the
       NVIC_SetPriority(SysTick_IRQn,...) just after the SysTick_Config() function 
       call. The NVIC_SetPriority() is defined inside the core_cm3.h file.

    4. To adjust the SysTick time base, use the following formula:
                            
         Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
    
       - Reload Value is the parameter to be passed for SysTick_Config() function
       - Reload Value should not exceed 0xFFFFFF
   */
void SetSysTick_config(uint32_t div)
{
  if (SysTick_Config(SystemCoreClock / div))
  { 
    /* Capture error */ 
    while (1);
  }
}

static uint32_t my_time = 0;

uint32_t time_return(void) 
{
  return my_time;
}

__IO uint32_t Tick_Counter;
void Get_Tcik(void)
{
    if (Tick_Counter++ > 1000) { // 0.9m
    Tick_Counter = 0;
    my_time++;
  }
}

void delay_ms(uint32_t cnt)
{
    TimingDelay = cnt;
    while(TimingDelay != 0);
}


void Set_SPI_Mode()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(SPI_RCC_AHBPeriph,ENABLE);

    GPIO_InitStructure.GPIO_Pin   = SPI_ENABLE_Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    //GPIO_SetBits(SPI_ENABLE_Port, GPIO_InitStructure.GPIO_Pin);       // If PF14 is high, it will be spi mode
    GPIO_ResetBits(SPI_ENABLE_Port, GPIO_InitStructure.GPIO_Pin);
}

void Set_Indirect_Mode()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(SPI_RCC_AHBPeriph,ENABLE);

    GPIO_InitStructure.GPIO_Pin   = SPI_ENABLE_Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
		GPIO_SetBits(SPI_ENABLE_Port, GPIO_InitStructure.GPIO_Pin);
    //GPIO_ResetBits(SPI_ENABLE_Port, GPIO_InitStructure.GPIO_Pin);       // If PF14 is low, it will be indirect mode
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {

  }
}
#endif
