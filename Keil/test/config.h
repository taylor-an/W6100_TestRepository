#ifndef _CONFIG_H
#define _CONFIG_H

#include <stdio.h>
#include "stm32f4xx.h"

/**
 * @brief Definition for connected to SPI1
 */
/* SPIx Communication boards Interface */
//      NSS (PA15)
// SPI1 CLK (PB3)  
// SPI1 MOSI(PB4)
// SPI1 MISO(PB5)

#define Open_RCC_APB2Periph_SPIx            RCC_APB2Periph_SPI1

#define Open_SPIx                           SPI1
#define Open_SPIx_CLK                       RCC_APB2Periph_SPI1
#define Open_SPIx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define Open_SPIx_IRQn                      SPI1_IRQn
#define Open_SPIx_IRQHANDLER                SPI1_IRQHandler

#define Open_SPIx_SCK_PIN                   GPIO_Pin_3
#define Open_SPIx_SCK_GPIO_PORT             GPIOB
#define Open_SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define Open_SPIx_SCK_SOURCE                GPIO_PinSource3
#define Open_SPIx_SCK_AF                    GPIO_AF_SPI1

#define Open_SPIx_MISO_PIN                  GPIO_Pin_4
#define Open_SPIx_MISO_GPIO_PORT            GPIOB
#define Open_SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define Open_SPIx_MISO_SOURCE               GPIO_PinSource4
#define Open_SPIx_MISO_AF                   GPIO_AF_SPI1

#define Open_SPIx_MOSI_PIN                  GPIO_Pin_5
#define Open_SPIx_MOSI_GPIO_PORT            GPIOB
#define Open_SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define Open_SPIx_MOSI_SOURCE               GPIO_PinSource5
#define Open_SPIx_MOSI_AF                   GPIO_AF_SPI1


//#define RCC_GPIO_CS                         RCC_AHB1Periph_GPIOA
//#define RCC_AHBxPeriphClockCmd                RCC_AHB1PeriphClockCmd
//#define GPIO_PIN_CS                         GPIO_Pin_15
//#define GPIO_CS_PORT                        GPIOA
#define RCC_GPIO_CS                         RCC_AHB1Periph_GPIOD
#define GPIO_PIN_CS                         GPIO_Pin_7
#define GPIO_CS_PORT                        GPIOD
#define WIZReset_Port                     GPIOF
#define WIZReset_Pin                      GPIO_Pin_14
//#define WIZPDWN_Pin                       GPIO_Pin_15


/**
 * @brief Definition for COM port1, connected to USART1
 */  
#define Open_USART                        USART1
#define Open_USART_CLK                    RCC_APB2Periph_USART1

#define Open_USART_TX_PIN                 GPIO_Pin_9
#define Open_USART_TX_GPIO_PORT           GPIOA
#define Open_USART_TX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define Open_USART_TX_SOURCE              GPIO_PinSource9
#define Open_USART_TX_AF                  GPIO_AF_USART1


#define Open_USART_RX_PIN                 GPIO_Pin_10
#define Open_USART_RX_GPIO_PORT           GPIOA
#define Open_USART_RX_GPIO_CLK            RCC_AHB1Periph_GPIOA
#define Open_USART_RX_SOURCE              GPIO_PinSource10
#define Open_USART_RX_AF                  GPIO_AF_USART1

#define Open_USART_IRQn                   USART1_IRQn   
#define Open_EXTI_IRQn                     EXTI15_10_IRQn
/**
 * @brief Definition USER's LEDs
 */  
#define LED0_Port                       GPIOE
#define LED0_Pin                        GPIO_Pin_14
#define LED0_RCC_AHBPeriph              RCC_AHB1Periph_GPIOE

#define LED1_Port                       GPIOE
#define LED1_Pin                        GPIO_Pin_15
#define LED1_RCC_AHBPeriph              RCC_AHB1Periph_GPIOE

#define LED2_Port                       GPIOD
#define LED2_Pin                        GPIO_Pin_9
#define LED2_RCC_AHBPeriph              RCC_AHB1Periph_GPIOD

#define LED3_Port                       GPIOD
#define LED3_Pin                        GPIO_Pin_10
#define LED3_RCC_AHBPeriph              RCC_AHB1Periph_GPIOD

#define LED4_Port                       GPIOB
#define LED4_Pin                        GPIO_Pin_15
#define LED4_RCC_AHBPeriph              RCC_AHB1Periph_GPIOB

#define LED5_Port                       GPIOB
#define LED5_Pin                        GPIO_Pin_14
#define LED5_RCC_AHBPeriph              RCC_AHB1Periph_GPIOB

#define SPI_ENABLE_Port         GPIOF
#define SPI_ENABLE_Pin          GPIO_Pin_14
#define SPI_RCC_AHBPeriph   RCC_AHB1Periph_GPIOF



#define INT_Port         GPIOC
#define INT_Pin          GPIO_Pin_14
#define INT_RCC_AHBPeriph   RCC_AHB1Periph_GPIOC

#endif
