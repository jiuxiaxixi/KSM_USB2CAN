#ifndef __LM75A_H
#define __LM75A_H	     
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_usart_dma.h"
#include "usart_screen.h"
#include "can.h"
#include "b3470.h"
#include <stdio.h>
#include "adc.h"

#define LM75_IDLE 		0	
#define LM75_PENGDING 1
#define LM75_SUCCESS	2
#define LM75_TX	3
#define LM75_FALIED		4

#define LM75_ADDR 		0x90
#define LM75_TIME 		3000
#define LM75_TIMEOUT	2000
//#define LM75t_WAITIME 		3000


#define LM75PRED_TIME 		6000//读取温度时间周期
#define LM75WAITIME 		50
#define LM75_MAX_RETRY				5
#define LM75_ACK 			0x01
#define LM75T_IDLE			0x00
#define LM75T_PENDING 	0x01
#define LM75T_QUERY    0x02
#define LM75T_POLLOING	0x03
#define	LM75T_FAILED		0x04
#define LM75T_SUCCESS	0x05
#define LM75_START				0x06
#define LM75_START_CHECK	0x07
#define ZL_WD_H 75            //制冷开启温度
#define ZL_WD_L 45            //制冷关闭温度


#define     ADC1_DR_Address     ((uint32_t)ADC1_BASE+0x4c)

#define LM35_READ_IDLE 		0x00
#define LM35_READ_START 	0x01
#define	LM35_READ_FINISH	0x02
#define ADC_SIZE	400

#define USE_LM35 1				//是否使用LM35 温度传感器  1:LM32 0:B3470
typedef struct {
	u8 	current_mission;  										//mission type
	u8	mission_state;         								//电平转换时间
	u8	index;                                //待串口屏接受成功之后返回数据之后，也就是单片机成功发送一条命令后idex就加1
	u8	retry_count;
	u8  Integred;
	u8  decide;
	u8  rank;
	u32 waitime;
	u32	timeout;
	u32 periodtime;
}LM75_usart_t;

typedef struct {
	u8	mission_state;         								//电平转换时间
	u32 waitime;
	u16 temp;
	u16 temp_real;
	u16 temp_buffer[10];
	u8 cooler_function;
	u32	pwm_time;
	u8 times;
	u32 temp_all;
	u8 temp_high;
	u8 temp_alarm_state;
	u32 close_inter_fan_time;
	u8 close_inter_fan_enable;
}lm35_temp_t;

extern u8 gobal_temp_flag;
extern LM75_usart_t LM75t;
extern volatile lm35_temp_t lm35_t;
u8 lm75a_init(void);
float lm75a_temp_read(void);
void add_end_code(char *buff);
void screen_time_polling(void);
void lm75a_usart_polling(void);
u8 LM75_parpare_buffer(void);
void LM75_mission(void);
extern u8	lm75_status;
void lm75a_mission_polling(void);
void lm75a_temp_read_polling(void);
void bubble_sort_better(__IO u16 a[],u16 n);
u16 temp_faded(u16 temp , u16 max_temp);
void reser_screen(void);
#endif
