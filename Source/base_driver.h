/**
  ******************************************************************************
  * @file    led.h
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
#ifndef __BASE_DRIVER_H
#define __BASE_DRIVER_H

#define LED_R   GPIO_Pin_15
#define LED_G   GPIO_Pin_14
#define LED_B   GPIO_Pin_8
/* 选择BANK1-NORSRAM1 连接 TFT，地址范围为0X60000000~0X63FFFFFF
 * FSMC_A16 接LCD的DC(寄存器/数据选择)脚
 * 寄存器基地址 = 0X60000000
 * RAM基地址 = 0X60020000 = 0X60000000+2^16*2 = 0X60000000 + 0X20000 = 0X60020000
 * 这里主要用于控制RS进行写寄存器写数据操作，因为板子的液晶RS接在A16管脚，所以要使地址线A16为高电平，则写地址到2的16次方处，并且因为液晶是16bit的
 *所以再乘上2，得出的地址加上BANK所在的地址即可。如RS接A0处，则RAM基地址为bank所在地址加上2的0次方的2倍，如是8bit屏则不需要乘以2
 */
#define FSMC_READ_ADDR  0x60020000
#define FSMC_RAM   *(vu16*)((u32)FSMC_READ_ADDR)  //disp Data ADDR

void LED_Config(void);
void LED_On(uint16_t led);
void LED_Off(uint16_t led);
void POWER_Config(uint16_t Power);
void FSMC_Config(void);
void FSMC_DMA_Configuration(uint8_t *pDST_Buffer,uint8_t *pSRC_Buffer,uint32_t BufferSize);

#endif
