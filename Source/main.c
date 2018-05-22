/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               main.c
** Last modified Date:      2018-05-18
** Last Version:            v1.1
** Description:             试剂盘主函数
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              张校源
** Created date:            2017-04-09
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             张校源
** Modified date:           2018-05-18
** Version:                 v1.1
** Description:             增加串口调试功能，并且在主函数中定义
**													通过宏USART_DEBUG_ENABLE 打开串口调试功能 
**													可以使用printf 函数 并在主函数中增加一个定时打印任务
**
*********************************************************************************************************/


/*********************************************************************************************************
	头文件
*********************************************************************************************************/
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "config.h"
#include "base_driver.h"
#include "delay.h"

#include "nand_driver.h"
#define NAND_FLASH_TEST 0
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_usart_dma.h"
#include "stm32fxxx_it.h"
#include "string.h"
#include <stdio.h>
#include "temp_control.h"	
#include "b3470.h"
#include "one_dimension_code.h"
#include "can.h"
#include "MicroStepDriver.h" 
#include "MSD_test.h" 
#include "coder.h"
#include "main.h"
#include "usart_screen.h"
#include "stmflash.h"
#include "notification.h"
#include "BUZZER.h"
#include "cooler.h"
#include "adc.h"
#include "mission.h"
#include "tm_stm32f4_watchdog.h"
/*********************************************************************************************************
** 是否启用串口调试功能
*********************************************************************************************************/
#define USART_DEBUG_ENABLE 0  //初始化串口功能

#if USART_DEBUG_ENABLE
#define DEBUG 1
uint32_t uart_debug_time;
#else
#define DEBUG 0
#endif

#if DEBUG
#include "usart.h"
#define PRINTF(...)   printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


/*********************************************************************************************************
 全局变量
*********************************************************************************************************/
char 		USB_SendReady=0;
extern  __IO uint8_t USB_StatusDataSended;
extern  uint32_t USB_ReceivedCount;
extern  uint8_t USB_Tx_Buffer[];
extern  uint8_t USB_Rx_Buffer[];
extern __IO uint8_t DeviceConfigured;

/*********************************************************************************************************
 USB 高速工作模式
*********************************************************************************************************/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


#if USART_DEBUG_ENABLE
/*********************************************************************************************************
** Function name:       b3470_init
** Descriptions:        串口300ms循环打印测试任务
** input parameters:    0
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-18
*********************************************************************************************************/
void usart_debug_mission(void)
{
	if(uart_debug_time < time)
	{
		PRINTF("temp = %d \r\n",b3470_get_temperature_offset(B3470_C3));//字符串写入缓存
		uart_debug_time = time + 300;
	}

}
#endif



int main(void) {
	/* Initialize system */
	u8 	canrxbuf[20];			//USB发送装帧
	u16 comandbuf[3];			//CAN2USB buffer
	u8  send_size_buf[2];	
	u8 	sendcount;
	//USB初始化
	USBD_Init(&USB_OTG_dev,USB_OTG_HS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);
	//can初始化
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,0);//CAN初始化环回模式,波特率125Kbps
#if USART_DEBUG_ENABLE
	//串口普通初始化
	USART_Configuration(9600);
#else
	//DMA串口初始化
	TM_USART_Init(USART1, TM_USART_PinsPack_2, 9600);
	TM_USART_DMA_Init(USART1);
#endif
	
	//滴答时钟初始化
	SysTick_Config(SystemCoreClock/1000);
	
	//一维码扫描初始化
	one_dimension_code_init();
	//复位传感器中断初始化
	EXTIX_Init();
	//编码器初始化
	configureEncoder();
	//FLASH 参数读取
	flash_init();
	//LED 初始化
	LED_init();
	//LED1 初始化
	LED1_init();	
	//制冷初始化
	cooler_init();
	
	//温度传感器初始化
	b3470_init();   
	//初始步进电机走位数据到FLASH
	//position_init_to_flash();
	//步进电机初始化
	MSD_Init();
	//电源控制初始化
	power_init();
	//蜂鸣器初始化
	BUZZER_Init();
	//看门狗初始化
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET)
			PRINTF("SYSTEM RESET\r\n");
	else
			PRINTF("KSM reagent-disk system start!\r\n");
	TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_2s);
	
	
	while (1) {          //无限循环
		TM_WATCHDOG_Reset(); //喂狗
#if USART_DEBUG_ENABLE
		usart_debug_mission();
#else
			//温度显示任务
			temp_display_mission();
#endif

		//蜂鸣器任务
			BUZZER_mission_polling();
		//电机走位任务
			motor_move_polling();

		//电机复位任务
			motor_reset();
		//电机振摇任务
		  motor_shocking();
		//屏幕时间同步任务
			su_mission_polling();
		//FLASH参数偏置任务
			flash_mission_polling();
		//实际温度查询任务
			lm75a_mission_polling();
		//LED动态提醒任务
			led_mission_polling();
			//ADC温度读取任务
			lm75a_temp_read_polling();
			//二维码扫描任务
			one_dimension_code_mission_polling();
			//电机维护命令
			motor_maintain_polling();
			
//通信部分	
		switch(USB2CAN_STATE){ 
		
		case USB_IDLE:
				break;
			
		case USB_FRAME_GET:   //接收数据
				//根据指针的相对大小返回待读取的帧数
				if(canRxMsgBufferIndex>=canRxIndex){
					comandbuf[0]=canRxMsgBufferIndex-canRxIndex;
					comandbuf[1]=canRxMsgBufferIndex;
				}else{
					comandbuf[0]=CAN_BUFFER_SIZE+canRxMsgBufferIndex-canRxIndex;
					comandbuf[1]=canRxMsgBufferIndex;
				}
				//USB上传指示灯
			if(comandbuf[0]>0){
				LED_ON(LED_UPLOAD);
				lmt.usb_tx_time=time+MIN_SLOT;
			}
				//每次查询最多上传220帧
				if(comandbuf[0]<220){
				send_size_buf[0]=comandbuf[0];
				}else{
				send_size_buf[0]=220;
				}
				//发送缓存帧数
				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,send_size_buf,1);
				
				USB2CAN_STATE=USB_SEND_FRAME;
				sendcount=0;
				led_to_notification(LED_USB_POLLING);
				//如果没有待发送的帧，直接返回
				if(send_size_buf[0]==0)
					USB2CAN_STATE=USB_IDLE;
				
			  break;
				
		case USB_SEND_FRAME:
					//发送成功才进行下一次发送
					if(USB_StatusDataSended==0)
						break;
					USB_StatusDataSended=0;
					
					//超过帧数停止发送
					if(sendcount>send_size_buf[0]){
						USB2CAN_STATE=USB_IDLE;
						break;
					}
					//发送帧数
					for(;sendcount<send_size_buf[0];sendcount++){
					//把帧装入发送缓存中
					parpareUSBframe(canrxbuf);
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,canrxbuf,15);
					poniter_plus_one(&canRxIndex);
					}
					break;
			
		}
			//如果有待发送的帧
			if(USBRxCanBufferIndex!=USBRxIndex){
				//LED灯提示
				led_to_notification(LED_CAN_TX);
					if(time>usb_waittime){
						CAN_Transmit(CAN1, &USBRxMessage[USBRxIndex]); 
						USBRxIndex++;
						//1ms帧间间隔
						usb_waittime=time+1;
					}
					
			}
					
		}
	
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
