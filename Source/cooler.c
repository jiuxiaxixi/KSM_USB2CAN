#include "cooler.h"
#include "lm75a.h"	
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
	//GPIO_SetBits(COOLER_PORT,GPIO_Pin_11);
	LED_ON(COOLER_LED);
	
}

//制冷关闭 高电平
                                                                        // 11是制冷片，12是水冷泵，13是风扇1-水冷风扇，14是风扇3-内部风扇
void 	cooler_off(void){   //关闭制冷具体函数
	//GPIO_SetBits(COOLER_PORT,COOLER_PIN);
	  GPIO_SetBits(COOLER_PORT, GPIO_Pin_11);    //关闭制冷片
   	
	
	LED_OFF(COOLER_LED);
}

void close_inter_fan(void){
	GPIO_SetBits(COOLER_PORT, GPIO_Pin_14);    //关闭内部风扇 
}

void close_cooler_fan(void){                   //关闭水冷泵及其风扇
	GPIO_SetBits(COOLER_PORT, GPIO_Pin_12);      //水冷泵
	GPIO_SetBits(COOLER_PORT, GPIO_Pin_13);      //水冷泵风扇
	
}

//



static void cooler_pwm_on(void){     //开制冷2
	GPIO_ResetBits(COOLER_PORT,COOLER_PWM_PIN);
	LED_ON(LED_COOLER_PWM);
}


static void cooler_pwm_off(void){  //关制冷2

	GPIO_SetBits(COOLER_PORT,COOLER_PWM_PIN);
	LED_OFF(LED_COOLER_PWM);
	
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
	switch(lm35_t.cooler_function){
		case COOLER_OFF:
			break;
		case COOLER_UP:
			if(time>lm35_t.pwm_time){
				cooler_pwm_on();
				lm35_t.cooler_function=COOLER_DOWN;
				lm35_t.pwm_time=time+COOLER_UP_TIME;
				break;
			}
			break;
			
		case	COOLER_DOWN:
				if(time>lm35_t.pwm_time){
			  //cooler_pwm_off();
				lm35_t.cooler_function=COOLER_UP;
				lm35_t.pwm_time=time+COOLER_DOWN_TIME;
				break;
			}
			break;
	
	}

}
