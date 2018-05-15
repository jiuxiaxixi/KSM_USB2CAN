#ifndef __USART_SCREEN_H
#define __USART_SCREEN_H
#include <stdio.h>
#include "stm32f4xx.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_usart_dma.h"
#include "can.h"

#define SCREEN_IDLE			0x00
#define SCREEN_PENDING 	0x01
#define SCREEN_QUERY		0x02
#define SCREEN_POLLOING	0x03
#define	SCREEN_FAILED		0x04
#define SCREEN_SUCCESS	0x05

#define MAX_RETRY				5
#define SCREEN_TIMEOUT	2000
#define SCREEN_WAITIME	20

#define SCREEN_ACK 			0x01
typedef struct {
	u8 	current_mission;  										//mission type
	u8	mission_state;         								//电平转换时间
	u8	index;
	u8	retry_count;
	u32 waitime;
	u32	timeout;
}screen_usart_t;

typedef struct {
  u8  	year1;  //工作状态
	u8 		year2;
	u8		buf[5];
}time_t;

extern time_t rct_t;

extern screen_usart_t sut;

void su_mission_polling(void);
void su_query(void);
void su_polling(void);
u8 parpare_buffer(void);


#endif

