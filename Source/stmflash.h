/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               stmflash.h
** Last modified Date:      2018-05-19
** Last Version:            v1.1
** Description:             热敏电阻NTC 温度传感器 B3470
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              张校源
** Created date:            2017-05-16
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             张校源
** Modified date:           2018-05-18
** Version:                 v1.1
** Description:             增加温度校准参数
**
*********************************************************************************************************/
#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "sys.h"   
#include "b3470.h"
/*********************************************************************************************************
 FLASH 参数设置以及长度
 FLASH 偏置类型为int16_t  占用两个字节
 在写入和读取函数是 是以32位为基础
 所以规定在flash 任务中写入和读取的长度
*********************************************************************************************************/
#define FLASH_PARA_NUM			 			 20
#define FLASH_PARA_LEN_32					 (FLASH_PARA_NUM/2)
#define FLASH_PARA_LEN_8					 (FLASH_PARA_NUM*2)
#define FLASH_TEMP1_OFFSET_POS		 11
#define FLASH_TEMP2_OFFSET_POS		 12
#define FLASH_C3_ZL_HIGH		 			 13
#define FLASH_C3_ZL_LOW	 			 		 14
#define FLASH_C2_ZL_HIGH		 			 15
#define FLASH_C2_ZL_LOW	 			 		 16
#define FLASH_TEMP_STATUS			 		 17
#define FLASH_PWOER_STATUS			 	 18

/*********************************************************************************************************
 硬件相关宏定义
*********************************************************************************************************/
//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000
//FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
// FLASH 一共1M 参数保存在128K处 0x0002 0000 = 128*1024*存储单元
#define FLASH_SAVE_ADDR  0x08020004 	
									
//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  

/*********************************************************************************************************
 FLASH 任务状态编号
*********************************************************************************************************/
#define FLASH_IDLE 					0x00
#define	FLASH_WRITE					0x01
#define FLASH_READ					0x02
#define	FLASH_WRITE_SUCCEE	0x03
#define FLASH_READ_SUCCEE		0x04
#define FLASH_FAILE					0x05
#define MEMORY_WIRTE_SUCCEE	0x06
/*********************************************************************************************************
  外部函数以及变量定义
*********************************************************************************************************/
u32 STMFLASH_ReadWord(u32 faddr);		  	//读出字  
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);   		//从指定地址开始读出指定长度的数据
void flash_init(void);
void flash_mission_polling(void);
void stmflash_read_reverse(u8 *pBuffer,u32 NumToRead);
void offset_calc(void);
void position_init_to_flash(void);
int16_t flash_get_para(uint8_t device);
/*********************************************************************************************************
  全局变量定义 以及外部声明
*********************************************************************************************************/

typedef struct {
  u8 		current_mission;  			//mission type
	u8		mission_state;
  u8		newdata;         	
	u8 		command_index;
  int16_t	buf[FLASH_PARA_NUM];								//电平转换时间
}flash_mission_t;

extern flash_mission_t fmt;

#endif

















