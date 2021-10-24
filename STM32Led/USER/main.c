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

// power 
// and set A4 B10 C13 as GPIO output
// and set Mode as GPIO_Mode_Out_PP
// and set output speed
void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructA;     //               
	GPIO_InitTypeDef GPIO_InitStructB;                    
	GPIO_InitTypeDef GPIO_InitStructC;                    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  // power the clock of GPIO A,B,C  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);  //
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);  //
	
	GPIO_InitStructA.GPIO_Mode    = GPIO_Mode_Out_PP;    
	GPIO_InitStructA.GPIO_Pin     = GPIO_Pin_3;           
	GPIO_InitStructA.GPIO_Speed   = GPIO_Speed_50MHz;    
	GPIO_Init(GPIOA,&GPIO_InitStructA);                   
	
	GPIO_InitStructB.GPIO_Mode    = GPIO_Mode_Out_PP;    
	GPIO_InitStructB.GPIO_Pin     = GPIO_Pin_0;           
	GPIO_InitStructB.GPIO_Speed   = GPIO_Speed_50MHz;    
	GPIO_Init(GPIOB,&GPIO_InitStructB);                   
	
	GPIO_InitStructC.GPIO_Mode    = GPIO_Mode_Out_PP;    
	GPIO_InitStructC.GPIO_Pin     = GPIO_Pin_13;           
	GPIO_InitStructC.GPIO_Speed   = GPIO_Speed_50MHz;    
	GPIO_Init(GPIOC,&GPIO_InitStructC);                   
}

void led_init2(void){
	RCC_APB2ENR |= (1<<2); // power GPIOA
	RCC_APB2ENR |= (1<<3); // power GPIOB
	RCC_APB2ENR |= (1<<4); // power GPIOC
	
	GPIOB_CRL &= ~( 0x0F<< (4*0)); // clear PB0
	GPIOB_CRL |= (2<<4*0);  // set PB0 Mode as GPIO_Mode_Out_PP

	GPIOA_CRL &= ~( 0x0F<< (4*3)); // clear PA3
	GPIOA_CRL |= (2<<4*3);  //  set PA3 Mode as GPIO_Mode_Out_PP
	
	GPIOC_CRH &= ~( 0x0F<< (4*5)); // clear PC13
	GPIOC_CRH |= (2<<4*5);  // set PC13 Mode as GPIO_Mode_Out_PP	
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

void SysTick_Delay_Ms( __IO uint32_t ms)
{
	uint32_t i;
	SysTick_Config(SystemCoreClock/1000);
	for (i=0; i<ms; i++) {

	while ( !((SysTick->CTRL)&(1<<16)) );
	}

	SysTick->CTRL &=~ SysTick_CTRL_ENABLE_Msk;
}


void state1(){
		GPIOB_BSRR = 0x01<<0;    // PB0 output high_level
		GPIOA_BSRR = 0x01<<(16+3);    // PA3 output Low_level
		GPIOC_BSRR = 0x01<<(16+13);    // PC13 output Low_level
}

void state2(){
		GPIOB_BSRR = 0x01<<(16+0);    // PB0 output low_level
		GPIOA_BSRR = 0x01<<3;    // PA3 output high_level
		GPIOC_BSRR = 0x01<<(16+13);    // PC13 output Low_level
}

void state3(){
		GPIOB_BSRR = 0x01<<(16+0);    // PB0 output low_level
		GPIOA_BSRR = 0x01<<(16+3);    // PA3 output Low_level
		GPIOC_BSRR = 0x01<<13;    // PC13 output high_level
}

unsigned int i=0;
uint32_t del = 1000;
int main(void)
{
	led_init();				
	//led_init2();
	
	
	while(1){
		switch(i%3){
			case 0: state1();
				break;
			case 1: state2();
				break;
			case 2: state3();
				break;
		}
		delay_ms(1000);
		//SysTick_Delay_Ms(del);
		i++;
	}
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
