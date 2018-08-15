/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               watchdog.c
** Last modified Date:      2018-05-21
** Last Version:            v1.0
** Description:             看门狗函数任务
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              张校源
** Created date:            2018-05-21
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------

*********************************************************************************************************/
/*********************************************************************************************************
** 是否启用串口调试功能
*********************************************************************************************************/
#define DEBUG 0
#if DEBUG
#include "usart.h"
#define PRINTF(...)   printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*********************************************************************************************************
	头文件
*********************************************************************************************************/
#include "wwdg.h"
#include "mission.h"
/*********************************************************************************************************
	全局变量
*********************************************************************************************************/
extern u32 time;
extern uint8_t usart_state;

/*********************************************************************************************************
** Function name:       wdg_flag_set
** Descriptions:        看门狗复位标记
** input parameters:    
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-29
*********************************************************************************************************/
void wdg_flag_set(struct iwdg_t *iwdg)
{
	iwdg->is_iwdg_set = 1;
}

/*********************************************************************************************************
** Function name:       wdg_flag_clear
** Descriptions:        看门狗复位标记
** input parameters:    
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-29
*********************************************************************************************************/
void wdg_flag_clear(struct iwdg_t *iwdg)
{
	iwdg->is_iwdg_set = 0;
}

/*********************************************************************************************************
** Function name:       wdg_motor_mission_set
** Descriptions:        看门狗复位标记
** input parameters:    
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-29
*********************************************************************************************************/
void wdg_motor_mission_set(struct iwdg_t *iwdg,struct motor_t *motor)
{
	iwdg->mission_state = motor->running_state;
	iwdg->current_mission = motor->current_mission;
	
	motor->current_mission = MOTOR_RESET;
	motor->running_state =	M_RESET_START;
}

/*********************************************************************************************************
** Function name:       wdg_motor_mission_set
** Descriptions:        看门狗复位标记
** input parameters:    
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-29
*********************************************************************************************************/
void wdg_motor_mission_backup(struct iwdg_t *iwdg,struct motor_t *motor)
{
	iwdg->mission_state = motor->running_state;
	iwdg->current_mission = motor->current_mission;
}

/*********************************************************************************************************
** Function name:       wdg_motor_mission_recovery
** Descriptions:        看门狗复位标记
** input parameters:    
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-29
*********************************************************************************************************/
void wdg_motor_mission_recovery(struct iwdg_t *iwdg,struct motor_t *motor)
{
	motor->current_mission = iwdg->current_mission;
	motor->running_state = iwdg->mission_state;
	iwdg->wdg_flag_clear(iwdg);
}

/*********************************************************************************************************
** Function name:       watch_dog_recovery
** Descriptions:        死机后恢复电源和制冷的任务
** input parameters:    0
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-23
*********************************************************************************************************/
void watch_dog_recovery(void)
{
	if(power_satus)
	{
		PRINTF("WDOG 开电 \r\n");
		power_on();
		power_off_state=0;
	}
	else
	{
		PRINTF("WDOG 关电 \r\n");
		power_off();
		power_off_state=1;
	}
	
	if(temp_control)
	{
		PRINTF("WDOG 开启制冷 \r\n");
		lm35_t.pwm_time=0;
		lm35_t.cooler_pwm_function =1;
		lm35_t.cooler_function = 1;
		lm35_t.c3_control_cooler=1;
	}
	else
	{
		PRINTF("WDOG 关闭制冷 \r\n");
		lm35_t.cooler_pwm_function = COOLER_OFF;
		lm35_t.cooler_function=0;
		lm35_t.close_inter_fan_enable=1;
		lm35_t.close_inter_fan_time=time+10000;
		cooler_off();
	}
	iwdg_t *wdg = &_iwdg;
	wdg->wdg_flag_set(wdg);
	mission_failed_send(IWDG_RESET);
	PRINTF("SYSTEM RESET \r\n");
}

/*********************************************************************************************************
** Function name:       system_attribute_init
** Descriptions:        保存在RAM2中 不初始化的变量在开机的时候进行初始化
** input parameters:    0
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-23
*********************************************************************************************************/
void system_attribute_init(void)
{
  USBRxCanBufferIndex=0;
	USBRxIndex=0;

	canRxMsgBufferIndex=0;
	canRxIndex=0;
	
	power_satus=1;
	temp_control=0;
	usart_state=0;
	is_soft_start = 0xAA;
}

iwdg_t _iwdg={
0,
0,
0,
wdg_flag_set,
wdg_flag_clear,
wdg_motor_mission_set,
wdg_motor_mission_recovery,
wdg_motor_mission_backup
};

/*********************************************************************************************************
  END FILE 
*********************************************************************************************************/
