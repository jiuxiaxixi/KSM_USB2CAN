#ifndef __MSD_TEST_H
#define __MSD_TEST_H
#include <stdio.h>
#include "stm32f4xx.h"
void motor_mission_polling(void);
void motor_reset(void);
u8 move_to_position(u16 position);
void motor_move_polling(void);
float angle(unsigned int nn,unsigned int mm,unsigned int pp);
u16 motor_inject_offset_position(unsigned int nn,unsigned int mm,unsigned int pp);
u16 step_to_move_calc(unsigned int nn,unsigned int mm,unsigned int pp);
void motor_shocking(void);
u16 MM_position(u8 mm);
void motor_finish(void);
void motor_timer_set(void);
 u8 motor_timeout_check(void);
void motor_maintain_polling(void);

typedef struct {
  unsigned char running_state;  //工作状态
  u32  timeout;         				//超时时间
  u32  waittime;								//电平转换时间
	u8	 current_mission;
	u8	 maintain_dir;
	short int offset[11];
	u8 	error_state;
}motor_t;
extern motor_t motor;
#define M_IDLE								0	
#define M_ENABLE 							1
#define M_START 							2
#define M_TIMEOUT							3
#define M_PENDDING						4
#define M_RESET_START					5
#define	M_RESET_POSITION			6
#define M_RESET_END						7
#define M_RUNNING							8
#define M_FINISH							9
#define M_SHOCK_PENDING 			10
#define M_Positive 						11
#define M_Reverse							12
#define M_PR_END							13
#define M_WAITTING_CODERSTEP 	14

#define M_MAINTAIN_PENDING		15
#define	M_MAINTAIN_POSITIVE		16
#define M_MAINTAIN_REVERSE		17
#define M_RESET_WAIT_STOP     18
#define M_RESET_MOVE_OFFSET 	19
#define MOTOR_TIME_OUT 						20

#define RESET_TIMEOUT 						5500
#define MOTOR_MOVE_ACC_SPEED			350   //电机加速度
#define	MOTOR_MOVE_DEL_SPEED			300   //电机减速度
#define	MOTOR_MOVE_MAX_SPEED			800   //电机最大速度
#define MOTOR_SPEED 							200	 
#define MOTOR_HISPEED							700
#define SHOCKING_SPEED_UP					300
#define SHOCKING_MAX_SPEED 				700
#define MAINTAIN_MAX_SPEED				500
#define MOTOR_WAITTIME						250
#define MOTOR_SHOCKING_WAIT_TIME	350
#define MOTOR_RESET_SPEED					300
#define RESET_OFFSET							-100


extern motor_t motor;
#endif	/* __MSD_TEST_H */
