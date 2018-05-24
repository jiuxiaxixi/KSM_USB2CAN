/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               cooler.h
** Last modified Date:      2018-05-23
** Last Version:            v1.2
** Description:             增加制冷片周期性断电控制
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              张校源
** Created date:            2017-05-16
** Version:                 v1.0
** Descriptions:            制冷片 扇热风扇 控制
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             张校源
** Modified date:           2018-05-23
** Version:                 v1.1
** Description:             增加制冷片周期性断电控制
**
*********************************************************************************************************/
#ifndef __COOLER_H
#define __COOLER_H

#include "stm32f4xx.h"
/*********************************************************************************************************
	制冷通断宏定义
*********************************************************************************************************/
#define COOLER_OFF					0x00
#define COOLER_UP						0x01
#define COOLER_DOWN					0x02

#define COOLER_UP_TIME			1800000  //工作半小时
#define COOLER_DOWN_TIME		60000		 //关闭1分钟

/*********************************************************************************************************
	硬件宏定义11是制冷片，12是水冷泵，13是风扇1水冷风扇，14是风扇3内部风扇
*********************************************************************************************************/
#define COOLER_PIN GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14   
#define COOLER_FS_PIN	 GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13

#define            	COOLER_AHBxClock_FUN            RCC_AHB1PeriphClockCmd
#define            	COOLER_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define            	COOLER_PORT                     GPIOD
#define							COOLER_PWM_PIN									GPIO_Pin_14

#define            	POWER_AHBxClock_FUN            	RCC_AHB1PeriphClockCmd
#define            	POWER_GPIO_CLK                 	RCC_AHB1Periph_GPIOD
#define 						POWER_PORT											GPIOD
#define							POWER_PIN												GPIO_Pin_4    //PD4 继电器控制引脚

/*********************************************************************************************************
外部函数及变量声明
*********************************************************************************************************/
extern u8 power_off_state;
extern u8 cooler_received_command;
void	cooler_init(void);
void 	cooler_on(void);
void 	cooler_off(void);
void close_inter_fan(void);
void close_cooler_fan(void);
void power_init(void);
void 	power_on(void);
void 	power_off(void);
void 	cooler_pwm_mission(void);
void open_inter_fan(void);
#endif
