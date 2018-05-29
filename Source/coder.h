#ifndef __CODER_H_
#define __CODER_H_	 	
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//外部中断 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 
#include "stm32f4xx.h"

#define LM35_TEST 	0

#define ENCLA_PIN               GPIO_Pin_5
#define ENCLA_GPIO_PORT         GPIOA
#define ENCLA_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define ENCLA_SOURCE            GPIO_PinSource5
#define ENCLA_AF                GPIO_AF_TIM2

#define ENCLB_PIN               GPIO_Pin_6
#define ENCLB_GPIO_PORT         GPIOA
#define ENCLB_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define ENCLB_SOURCE            GPIO_PinSource6
#define ENCLB_AF                GPIO_AF_TIM2

// Right Motor Channels
#define REST_PIN             	  GPIO_Pin_8
#define REST_GPIO_PORT       	  GPIOA
#define RSET_GPIO_CLK          RCC_AHB1Periph_GPIOA


#define ENCRB_PIN               GPIO_Pin_7
#define ENCRB_GPIO_PORT         GPIOB
#define ENCRB_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define ENCRB_SOURCE            GPIO_PinSource7
#define ENCRB_AF                GPIO_AF_TIM4

// determine the timers to use
#define ENCL_TIMER              TIM2
#define ENCL_TIMER_CLK          RCC_APB1Periph_TIM2
#define ENCR_TIMER              TIM4
#define ENCR_TIMER_CLK          RCC_APB1Periph_TIM4

#define LEFT_COUNT()            ENCL_TIMER->CNT
#define RIGHT_COUNT()           ENCR_TIMER->CNT

void EXTIX_Init(void);	//外部中断初始化		 
void coder_gpio_init(void);
void configureEncoder(void);
void coder_timer_init(void);
extern int coder_setp;
extern u8 coder_init_count;
#endif
