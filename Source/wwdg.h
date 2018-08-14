/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               watchdog.h
** Last modified Date:      2018-05-21
** Last Version:            v1.0 
** Description:             窗口看门狗
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              张校源
** Created date:            2018-05-21
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             张校源
** Modified date:           2018-05-29
** Version:                 
** Description:             
**
*********************************************************************************************************/
#ifndef __WATCH_DOG_H__ 
#define __WATCH_DOG_H__ 

#include "stm32f4xx.h" 
#include "MSD_test.h"  
/*********************************************************************************************************
	宏定义
*********************************************************************************************************/

/*********************************************************************************************************
	外部函数
*********************************************************************************************************/
typedef struct iwdg_t{
	uint8_t 	current_mission;  										//mission type
	uint8_t		mission_state;         								//电平转换时间
	uint8_t 	is_iwdg_set; 
	void (* wdg_flag_set)(struct iwdg_t *iwdg);
	void (* wdg_flag_clear)(struct iwdg_t *iwdg);
	void (* wdg_motor_mission_set)(struct iwdg_t *iwdg,struct motor_t *motor);
	void (* wdg_motor_mission_recovery)(struct iwdg_t *iwdg,struct motor_t *motor);
	void (* wdg_motor_mission_backup)(struct iwdg_t *iwdg,struct motor_t *motor);
}iwdg_t;

extern iwdg_t _iwdg;


void system_attribute_init(void);
void watch_dog_recovery(void);

#endif

/*********************************************************************************************************
  END FILE 
*********************************************************************************************************/
