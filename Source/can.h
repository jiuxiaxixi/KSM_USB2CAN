#ifndef __CAN_H
#define __CAN_H	     
#include "stm32f4xx.h"

#define	CAN_MASTER_ID 	0x88   //上传ID号
#define CAN_SLAVE_ID		0x10   //下发板卡号
#define	ACTION_SUCCESS	0x80   //成功返回值
#define ACTION_FAILED 	0x00   //失败返回值
#define CAN_MISSION_IDLE 			0x00   
#define CAN_MISSION_FINISHED	0xFF


#define MOTOR_DEBUG							0x05    //电机参数调试命令号
#define FLASH_MISSION_WRITE			0x07    //参数写入命令
#define	FLASH_MISSION_READ			0x06    //参数读取命令

#define STATE_QUERY			0x01    //命令号
#define VERSION_UPLOAD	0x03	
#define MOTOR_RESET			0x09
#define MOTOR_INSTALL 	0x0A
#define	MOTOR_INJECTION	0x0B
#define	TEMP_QUERY			0x0C
#define COOLER_START 		0x0D	
#define COOLER_STOP 		0x0E	
//#define REAGENT_CHECK		0x0F	//UNDO
#define	ODC_UPLOAD			0x0F	//UNDO
#define	MOTOR_SHOCKING_START 	0x10
#define MOTOR_SHOCKING_STOP		0x11
#define TIME_MODIFY			0x12
#define TEMP_DISPLAY		0x13
#define	TEMP_SHINING		0x14
#define	POWER_ON				0x15
#define POWER_OFF				0x16
#define	MOTOR_MATAIN		0x17
#define LED1_ON_CMD     0x18 //10 18 NN ;NN=01=红灯、NN=02=绿灯
#define LED1_OFF_CMD    0x19 //10 19 NN ;NN=01=红灯、NN=02=绿灯
#define RESET_SCREEN    0x1A 

//多帧配置
#define CAN_DATA_LENGTH			0x05
#define CAN_DATE_START_BIT	0x03
#define CAN_MISSION_BIT			0x00
#define CAN_COMPITE_BIT			0x01
#define CAN_COMPITE_CONTENT	0x00
#define	CAN_FRAME_FRAG_BIT	0x02


#define CAN1_RX0_INT_ENABLE	1								    
#define CAN_BUFFER_SIZE	500
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
 
u8 CAN1_Send_Msg(void);			

u8 CAN1_Receive_Msg(u8 *buf);			

void parpareUSBframe(u8 *buf);
void can_mesage_send_polling(void);
u8 mission_idle_wait(u8 mission);
void	one_can_frame_send(u8 *buf,u8 length,u8 mission);
void  action_success_send(void);
void action_failed_send(void);
void action_value_send(u8 *buf,u8 length,u8 mission);
void  mission_success_send(u8 mission);
void  mission_failed_send(u8 mission);
void can_mutil_frame_send(u8 *buf,u8 length,u8 mission);
void action_value_send_none_80(u8 *buf,u8 length,u8 mission);
void poniter_plus_one(u16 * index);
void poniter_add(u16 *index,u8 number);

extern CanRxMsg CanRxMessage[CAN_BUFFER_SIZE];
extern uint8_t canSendedIndex;
extern uint16_t canRxMsgBufferIndex;
extern uint16_t canRxIndex;
extern uint8_t temp_control;
extern uint8_t power_satus;

extern uint8_t mission_state[255];
//USB recive message cache
extern CanTxMsg	USBRxMessage[255];
extern uint8_t USBRxCanBufferIndex;
extern uint8_t USBRxIndex;
extern	uint8_t USB2CAN_STATE;
		#define USB_IDLE 				0x00
		#define USB_FRAME_GET 	0x01
		#define USB_SEND_FRAME 	0x02
		#define USB_WAIT_ACK    0X03

#endif

