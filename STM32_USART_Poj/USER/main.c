#include "stm32f10x.h"
#include <stdio.h>
#ifndef PERIPH_BASE
#define PERIPH_BASE ((unsigned int)0x40000000)
#endif
#define APB2PERIPH_BASE (PERIPH_BASE + 0x10000)

#define GPIOA_BASE (APB2PERIPH_BASE + 0x0800)
#define GPIOA_CRL *(unsigned int*)(GPIOA_BASE+0x00)
#define GPIOA_CRH *(unsigned int*)(GPIOA_BASE+0x04)
#define GPIOA_IDR *(unsigned int*)(GPIOA_BASE+0x08)
#define GPIOA_ODR *(unsigned int*)(GPIOA_BASE+0x0C)
#define GPIOA_BSRR *(unsigned int*)(GPIOA_BASE+0x10)
#define GPIOA_BRR *(unsigned int*)(GPIOA_BASE+0x14)
#define GPIOA_LCKR *(unsigned int*)(GPIOA_BASE+0x18)

#define GPIOB_BASE (APB2PERIPH_BASE + 0x0C00)
#define GPIOB_CRL *(unsigned int*)(GPIOB_BASE+0x00)
#define GPIOB_CRH *(unsigned int*)(GPIOB_BASE+0x04)
#define GPIOB_IDR *(unsigned int*)(GPIOB_BASE+0x08)
#define GPIOB_ODR *(unsigned int*)(GPIOB_BASE+0x0C)
#define GPIOB_BSRR *(unsigned int*)(GPIOB_BASE+0x10)
#define GPIOB_BRR *(unsigned int*)(GPIOB_BASE+0x14)
#define GPIOB_LCKR *(unsigned int*)(GPIOB_BASE+0x18)

#define GPIOC_BCSE (APB2PERIPH_BASE + 0x1000)
#define GPIOC_CRL *(unsigned int*)(GPIOC_BASE+0x00)
#define GPIOC_CRH *(unsigned int*)(GPIOC_BASE+0x04)
#define GPIOC_IDR *(unsigned int*)(GPIOC_BASE+0x08)
#define GPIOC_ODR *(unsigned int*)(GPIOC_BASE+0x0C)
#define GPIOC_BSRR *(unsigned int*)(GPIOC_BASE+0x10)
#define GPIOC_BRR *(unsigned int*)(GPIOC_BASE+0x14)
#define GPIOC_LCKR *(unsigned int*)(GPIOC_BASE+0x18)

#define RCC_BASE (AHBPERIPH_BASE + 0x1000)
#define RCC_APB2ENR *(unsigned int*)(RCC_BASE+0x18)

#define DEBUG_USARTx USART1
#define DEBUG_USART_CLK RCC_APB2Periph_USART1
#define DEBUG_USART_APBxClkCmd RCC_APB2PeriphClockCmd
#define DEBUG_USART_BAUDRATE 115200

#define DEBUG_USART_GPIO_CLK (RCC_APB2Periph_GPIOA)
#define DEBUG_USART_GPIO_APBxClkCmd RCC_APB2PeriphClockCmd
#define DEBUG_USART_TX_GPIO_PORT GPIOA
#define DEBUG_USART_TX_GPIO_PIN GPIO_Pin_9
#define DEBUG_USART_RX_GPIO_PORT GPIOA
#define DEBUG_USART_RX_GPIO_PIN GPIO_Pin_10
#define DEBUG_USART_IRQ USART1_IRQn
#define DEBUG_USART_IRQHandler USART1_IRQHandler

static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
    
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(DEBUG_USARTx, &USART_InitStructure);

	
	NVIC_Configuration();

	
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(DEBUG_USARTx, ENABLE);
}

/***************** Send a Character **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	USART_SendData(pUSARTx,ch);

	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

/***************** Send a String **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
	do {
		Usart_SendByte( pUSARTx, *(str + k) );
	k++;
	} while (*(str + k)!='\0');
    
	while (USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET) {}
}

void delay_us(u32 nus)
{
 u32 temp;
 SysTick->LOAD = 9*nus;
 SysTick->VAL=0X00;
 SysTick->CTRL=0X01;
 do
 {
  temp=SysTick->CTRL;
 }while((temp&0x01)&&(!(temp&(1<<16))));
     SysTick->CTRL=0x00; 
    SysTick->VAL =0X00; 
}
void delay_ms(u16 nms)
{
 u32 temp;
 SysTick->LOAD = 9000*nms;
 SysTick->VAL=0X00;
 SysTick->CTRL=0X01;
 do
 {
  temp=SysTick->CTRL;
 }while((temp&0x01)&&(!(temp&(1<<16))));
    SysTick->CTRL=0x00; 
    SysTick->VAL =0X00; 
}


int main(void)
{
	USART_Config();
	
	while(1){
		Usart_SendString( DEBUG_USARTx,"Hello STM32!\n");
		delay_ms(1000);
	}
}







/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
