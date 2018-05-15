#ifndef __BUZZER_H
#define __BUZZER_H	     
#include "can.h"
#include <stdio.h>

#define    BUZZER_GPIO_CLK   RCC_AHB1Periph_GPIOD
#define    BUZZER_PORT       GPIOD
#define    BUZZER_PIN        GPIO_Pin_15



#define BUZZERIDLE			0x00
#define BUZZERPENDDING 	0x01
#define UP 							0x02
#define DOWN 						0x03
#define MODE_ONE 				0x01
#define MODE_TWO				0x02
#define MODE_THREE			0x03
#define DOWN_TIME 			50

typedef struct {                //类型定义，指定 buzzer_t  为一个结构体类型，等效于关键字 struct ，即不再是变量名而是类型名    可以用   buzzer_t 定义变量
  unsigned char running_state;  //工作状态
	u32 ranktime;
  u32 waittime;								//电平转换时间
	u8	current_mission;
	u8 	length;
	u32 uptime[6];
	u8 	index;
}buzzer_t;

extern buzzer_t buzzer;  //链接外部文件变量
void BUZZER_Init(void);
void BUZZER_mission_polling(void);
void buzzer_mission(u8 mission);
#endif
