/**
  ******************************************************************************
  * @file    base_driver.c
  * $Author: wdluo $
  * $Revision: 229 $
  * $Date:: 2014-05-13 13:00:02 +0800 #$
  * @brief   消息定义.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, EmbedNet</center>
  *<center><a href="http:\\www.embed-net.com">http://www.embed-net.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/ 
#include "stm32f4xx.h"
#include "base_driver.h"

/**
  * @brief  配置LED引脚
  * @param  无
  * @retval 无
  */
void LED_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_InitStructure.GPIO_Pin);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_SetBits(GPIOD,GPIO_Pin_8);
}
/**
  * @brief  点亮LED
  * @param  LED_R,LED_G,LED_B
  * @retval 无
  */
void LED_On(uint16_t led)
{
  if(LED_B == led){
    GPIO_ResetBits(GPIOD,LED_B);
  }else{
    GPIO_ResetBits(GPIOB,led);
  }
}

/**
  * @brief  熄灭LED
  * @param  LED_R,LED_G,LED_B
  * @retval 无
  */
void LED_Off(uint16_t led)
{
  if(LED_B == led){
    GPIO_SetBits(GPIOD,LED_B);
  }else{
    GPIO_SetBits(GPIOB,led);
  }
}


