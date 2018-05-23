/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               cooler.c
** Last modified Date:      2018-05-23
** Last Version:            v1.2
** Description:             
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
#include "cooler.h"
#include "temp_control.h"	
#include "notification.h"
u8 power_off_state;
extern u32 time;

//制冷片初始化，端口配置
void	cooler_init(void){
	  GPIO_InitTypeDef GPIO_InitStructure;
	
		COOLER_AHBxClock_FUN(COOLER_GPIO_CLK,ENABLE);//开启时钟
	
		GPIO_InitStructure.GPIO_Pin =  COOLER_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COOLER_PORT, &GPIO_InitStructure);
		
		cooler_off(); //初始化关闭制冷
}


//制冷开起 低电平
void 	cooler_on(void){
	GPIO_ResetBits(COOLER_PORT,COOLER_PIN);
	open_inter_fan();
	LED_ON(COOLER_LED);
}

//制冷关闭 高电平
                                                                        // 11是制冷片，12是水冷泵，13是风扇1-水冷风扇，14是风扇3-内部风扇
void 	cooler_off(void){   //关闭制冷具体函数
	  GPIO_SetBits(COOLER_PORT, GPIO_Pin_11);    //关闭制冷片
		LED_OFF(COOLER_LED);
}

void close_inter_fan(void){
	GPIO_SetBits(COOLER_PORT, GPIO_Pin_14);    //关闭内部风扇 
	PRINTF("关闭内部风扇 \r\n");
}

void open_inter_fan(void){
	GPIO_ResetBits(COOLER_PORT, GPIO_Pin_14);    //关闭内部风扇 
	PRINTF("开启内部风扇 \r\n");
}

void close_cooler_fan(void){                   //关闭水冷泵及其风扇
	GPIO_SetBits(COOLER_PORT, GPIO_Pin_12);      //水冷泵
	GPIO_SetBits(COOLER_PORT, GPIO_Pin_13);      //水冷泵风扇
	
}

static void cooler_pwm_on(void){     //开制冷2
	GPIO_ResetBits(COOLER_PORT,COOLER_PWM_PIN);
	LED_ON(LED_COOLER_PWM);
}


//电源管理-继电器初始化
void power_init(void){
		GPIO_InitTypeDef GPIO_InitStructure;
		POWER_AHBxClock_FUN(POWER_GPIO_CLK,ENABLE);  //时钟使能
	//端口配置
		GPIO_InitStructure.GPIO_Pin =  POWER_PIN;     //PD4 继电器控制引脚
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(POWER_PORT, &GPIO_InitStructure);
		power_on();
}

void 	power_on(void){   //开继电器
	GPIO_ResetBits(POWER_PORT,POWER_PIN);
	LED_ON(LED_POWER);
}

void 	power_off(void){
	GPIO_SetBits(POWER_PORT,POWER_PIN);   //关闭继电器  PD4
	LED_OFF(LED_POWER);  
}


void 	cooler_pwm_mission(void){ // 未用
	switch(lm35_t.cooler_pwm_function){
		case COOLER_OFF:
			break;
		
		case COOLER_UP:
			if(time>lm35_t.pwm_time){
				PRINTF("PWM 温度开启制冷 \r\n");
				cooler_on();
				lm35_t.cooler_pwm_function=COOLER_DOWN;
				lm35_t.cooler_function=1;
				lm35_t.pwm_time=time+COOLER_UP_TIME;
				break;
			}
			break;
			
		case	COOLER_DOWN:
				if(time>lm35_t.pwm_time){
				PRINTF("PWM 温度关闭制冷 \r\n");
			  cooler_off();
				lm35_t.close_inter_fan_time=time+60000;  //延时60S
				lm35_t.close_inter_fan_enable =1;        //关闭内部风扇标志可用
				lm35_t.cooler_function=0;
				lm35_t.cooler_pwm_function=COOLER_UP;
				lm35_t.pwm_time=time+COOLER_DOWN_TIME;
				break;
			}
			break;
	
	}

}
