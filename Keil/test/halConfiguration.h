
#ifndef __BSP_H
#define __BSP_H
#include "stm32f4xx.h"

void Interface_init(void);

void Delay_Tick(__IO uint32_t nTime);
void SetSysTick_config(uint32_t div);
void GPIO_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void TimingDelay_Decrement(void);
uint32_t time_return(void);
void Get_Tcik(void);
void delay_ms(uint32_t cnt);

void Set_SPI_Mode(void);
void Set_Indirect_Mode(void);

#endif /*__BSP_H*/
