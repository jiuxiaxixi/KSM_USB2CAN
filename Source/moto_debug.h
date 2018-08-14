/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               motor_debug.h
** Last modified Date:      2018-08-08
** Last Version:            v1.0
** Description:             试剂盘失步率测量
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              张校源
** Created date:            2018-08-08
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:          
** Version:                 
** Description:             
**
*********************************************************************************************************/
#ifndef __MOTOR_DEBUG_H
#define __MOTOR_DEBUG_H
#include <stdint.h>
#include "can.h"
	
typedef struct motor_para{
	uint16_t pwm_send;
	uint16_t pwm_rec;
	uint16_t zero_count;
	uint16_t report_rate;
	void (* pwm_add)(struct motor_para *motor_para);
	void (* pwm_count)(struct motor_para *motor_para,uint16_t );
	void (* report_to_can)(struct motor_para *motor_para);
	//void (* set_report_rate)(struct motor_para *motor_para,uint16_t );
	
}motor_para;
void upload_step_info(uint8_t mission,uint16_t step,uint16_t setpcount);
#endif
