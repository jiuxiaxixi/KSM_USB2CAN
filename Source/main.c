/****************************************Copyright (c)****************************************************
**                                	ÖØÇì¿ÆË¹ÂõÉúÎï¿Æ¼¼ÓÐÏÞ¹«Ë¾
**                                    6500 ÊÔ¼ÁÏµÍ³                    
**																			@ÕÅÐ£Ô´
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               main.c
** Last modified Date:      2018-05-23
** Last Version:            v1.2
** Description:             ÊÔ¼ÁÅÌÖ÷º¯Êý
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              ÕÅÐ£Ô´
** Created date:            2017-04-09
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             ÕÅÐ£Ô´
** Modified date:           2018-05-18
** Version:                 v1.1
** Description:             Ôö¼Ó´®¿Úµ÷ÊÔ¹¦ÄÜ£¬²¢ÇÒÔÚÖ÷º¯ÊýÖÐ¶¨Òå
**													Í¨¹ýºêUSART_DEBUG_ENABLE ´ò¿ª´®¿Úµ÷ÊÔ¹¦ÄÜ 
**													¿ÉÒÔÊ¹ÓÃprintf º¯Êý ²¢ÔÚÖ÷º¯ÊýÖÐÔö¼ÓÒ»¸ö¶¨Ê±´òÓ¡ÈÎÎñ
**--------------------------------------------------------------------------------------------------------
** Modified by:             ÕÅÐ£Ô´
** Modified date:           2018-05-23
** Version:                 v1.2
** Description:             
**													1.Ôö¼Ó¿´ÃÅ¹· 
**													2.¶Ïµç»Ö¸´»º´æ
**													3.ÐÂÔö¼ÓÎÂ¶È¿ØÖÆ·½°¸  Ê¹ÓÃB3470 NTCÎÂ¶È´«¸ÐÆ÷ 
**--------------------------------------------------------------------------------------------------------
** Modified by:             ÕÅÐ£Ô´
** Modified date:           2018-05-24
** Version:                 v1.3
** Description:             
**													1.ÐÞ¸´USB2CAN bug
**
*********************************************************************************************************/



/*********************************************************************************************************
	Í·ÎÄ¼þ
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
#include "wwdg.h"
/*********************************************************************************************************
** ÊÇ·ñÆôÓÃ´®¿Úµ÷ÊÔ¹¦ÄÜ
*********************************************************************************************************/
#define USART_DEBUG_ENABLE 0  //³õÊ¼»¯´®¿Ú¹¦ÄÜ

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
 È«¾Ö±äÁ¿
*********************************************************************************************************/
char 		USB_SendReady=0;
uint32_t 	usb_timeout;
extern  __IO uint8_t USB_StatusDataSended;
extern  uint32_t USB_ReceivedCount;
extern  uint8_t USB_Tx_Buffer[];
extern  uint8_t USB_Rx_Buffer[];
extern __IO uint8_t DeviceConfigured;
/*********************************************************************************************************
 ¿´ÃÅ¹·È«¾Ö±äÁ¿
*********************************************************************************************************/

/*********************************************************************************************************
 USB ¸ßËÙ¹¤×÷Ä£Ê½
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
** Descriptions:        ´®¿Ú300msÑ­»·´òÓ¡²âÊÔÈÎÎñ
** input parameters:    0
** output parameters:   0
** Returned value:      0
** Created by:          ÕÅÐ£Ô´
** Created Date:        2018-05-18
*********************************************************************************************************/
void usart_debug_mission(void)
{
	if(uart_debug_time < time)
	{
		//PRINTF("temp = %d \r\n",Get_Adc_v(0));//×Ö·û´®Ð´Èë»º´æ
		PRINTF("CAN µØÖ· %d %d Ê±¼ä %d %d\r\n",USBRxCanBufferIndex,USBRxIndex,time,usb_waittime);
		uart_debug_time = time + 300;
	}

}
#endif



int main(void) {
	/* Initialize system */
	u8 	canrxbuf[20];			//USB·¢ËÍ×°Ö¡
	u16 comandbuf[3];			//CAN2USB buffer
	u8  send_size_buf[2];	
	u8 	sendcount;
	u8  last_can_mailbox=CAN_TxStatus_NoMailBox;
	u8 	can_send_sate=0;

	//USB³õÊ¼»¯
	USBD_Init(&USB_OTG_dev,USB_OTG_HS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);
	//can³õÊ¼»¯
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_1tq,CAN_BS1_6tq,42,0);//CAN³õÊ¼»¯»·»ØÄ£Ê½,²¨ÌØÂÊ125Kbps
#if USART_DEBUG_ENABLE
	//´®¿ÚÆÕÍ¨³õÊ¼»¯
	USART_Configuration(9600);
#else
	//DMA´®¿Ú³õÊ¼»¯
	TM_USART_Init(USART1, TM_USART_PinsPack_2, 9600);
	TM_USART_DMA_Init(USART1);
#endif
	
	//µÎ´ðÊ±ÖÓ³õÊ¼»¯
	SysTick_Config(SystemCoreClock/1000);
	
	//Ò»Î¬ÂëÉ¨Ãè³õÊ¼»¯
	one_dimension_code_init();
	//¸´Î»´«¸ÐÆ÷ÖÐ¶Ï³õÊ¼»¯
	EXTIX_Init();
	//±àÂëÆ÷³õÊ¼»¯
	configureEncoder();
	//FLASH ²ÎÊý¶ÁÈ¡
	flash_init();
	//LED ³õÊ¼»¯
	LED_init();
	//LED1 ³õÊ¼»¯
	LED1_init();	
	//ÖÆÀä³õÊ¼»¯
	cooler_init();
	
	//ÎÂ¶È´«¸ÐÆ÷³õÊ¼»¯
	b3470_init();   
	//³õÊ¼²½½øµç»ú×ßÎ»Êý¾Ýµ½FLASH
	//position_init_to_flash();
	//²½½øµç»ú³õÊ¼»¯
	MSD_Init();
	//µçÔ´¿ØÖÆ³õÊ¼»¯
	power_init();
	//·äÃùÆ÷³õÊ¼»¯
	BUZZER_Init();
	//¿´ÃÅ¹·³õÊ¼»¯
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET || is_soft_start == 0xAA)
	{
			watch_dog_recovery();
	}
	else
	{
		system_attribute_init();
		PRINTF("KSM reagent-disk system start!\r\n");
		
	}
	
	TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_32s);
	
	
	while (1) {          //ÎÞÏÞÑ­»·
		TM_WATCHDOG_Reset(); //Î¹¹·
#if USART_DEBUG_ENABLE
		usart_debug_mission();
#else
			//ÎÂ¶ÈÏÔÊ¾ÈÎÎñ
			temp_display_mission();
#endif

		//·äÃùÆ÷ÈÎÎñ
			BUZZER_mission_polling();
		//µç»ú×ßÎ»ÈÎÎñ
			motor_move_polling();
			motor_diagnosis_mission();
		//µç»ú¸´Î»ÈÎÎñ
			motor_reset();
		//µç»úÕñÒ¡ÈÎÎñ
		  motor_shocking();
		//ÆÁÄ»Ê±¼äÍ¬²½ÈÎÎñ
			su_mission_polling();
		//FLASH²ÎÊýÆ«ÖÃÈÎÎñ
			flash_mission_polling();
		//Êµ¼ÊÎÂ¶È²éÑ¯ÈÎÎñ
			lm75a_mission_polling();
		//LED¶¯Ì¬ÌáÐÑÈÎÎñ
			led_mission_polling();
			//ADCÎÂ¶È¶ÁÈ¡ÈÎÎñ
			lm75a_temp_read_polling();
			//¶þÎ¬ÂëÉ¨ÃèÈÎÎñ
			one_dimension_code_mission_polling();
			//µç»úÎ¬»¤ÃüÁî
			motor_maintain_polling();
#if USE_LM35
			//ÖÆÀäÍ¨¶ÏÈÎÎñ
			cooler_pwm_mission();
#endif 

			//Ê±¼ä¿ì³¬Ê±ÁË ÖØÖÃÊ±¼ä
			if( time >= 0xEFFFFFFF && USB2CAN_STATE == USB_IDLE && motor.running_state == M_IDLE)
			{
				timer_reset();
			}
		//Í¨ÐÅ²¿·
			
		switch(USB2CAN_STATE){ 
		
		case USB_IDLE:
				break;
			
		case USB_FRAME_GET:   //½ÓÊÕÊý¾Ý
				//¸ù¾ÝÖ¸ÕëµÄÏà¶Ô´óÐ¡·µ»Ø´ý¶ÁÈ¡µÄÖ¡Êý
				if(canRxMsgBufferIndex>=canRxIndex){
					comandbuf[0]=canRxMsgBufferIndex-canRxIndex;
					comandbuf[1]=canRxMsgBufferIndex;
				}else{
					comandbuf[0]=CAN_BUFFER_SIZE+canRxMsgBufferIndex-canRxIndex;
					comandbuf[1]=canRxMsgBufferIndex;
				}
				//USBÉÏ´«Ö¸Ê¾µÆ
				led_to_notification(LED_USB_POLLING);

				//Ã¿´Î²éÑ¯×î¶àÉÏ´«220Ö¡
				if(comandbuf[0]<100){
				send_size_buf[0]=comandbuf[0];
				}else{
				send_size_buf[0]=100;
				}
				//·¢ËÍ»º´æÖ¡Êý
				USB_StatusDataSended=0;
				DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,send_size_buf,1);
				
				
				USB2CAN_STATE=USB_SEND_FRAME;
				sendcount=0;
				
				//Èç¹ûÃ»ÓÐ´ý·¢ËÍµÄÖ¡£¬Ö±½Ó·µ»Ø
				if(send_size_buf[0]==0)
					USB2CAN_STATE=USB_IDLE;
				
			  break;
				
		case USB_SEND_FRAME:
				//·¢ËÍ³É¹¦²Å½øÐÐÏÂÒ»´Î·¢ËÍ
				 if(USB_StatusDataSended==0)
					break;
					//³¬¹ýÖ¡ÊýÍ£Ö¹·¢ËÍ
					//·¢ËÍÍê³ÉÖ®ºóÍ£Ö¹
					if(sendcount>=send_size_buf[0])
					{
						USB2CAN_STATE=USB_IDLE;
						break;
					}
					//°ÑÖ¡×°Èë·¢ËÍ»º´æÖÐ
					parpareUSBframe(canrxbuf);
					USB_StatusDataSended=0;
					DCD_EP_Tx(&USB_OTG_dev,CDC_IN_EP,canrxbuf,15);
					usb_timeout = time + 20; //5ms×Ô¶¯ÖØ´«
					USB2CAN_STATE = USB_WAIT_ACK;
					break;
					
		case USB_WAIT_ACK:
			
			if(time > usb_timeout)
				 {
					PRINTF("USBÖØ´«³¬Ê±\r\n");
					USB2CAN_STATE=USB_IDLE;
					 sendcount++;
					USB_StatusDataSended=1;
					break;
				 }
			if(USB_StatusDataSended==0)
				break;
			  //Ö¸Õë++
				led_to_notification(LED_USB_UPLOAD);
				poniter_plus_one(&canRxIndex);
				sendcount++;
			  USB2CAN_STATE = USB_SEND_FRAME;
			break;
		}
		
			
		switch(can_send_sate)
		{
			
			case 0:
				if(USBRxCanBufferIndex!=USBRxIndex)
					can_send_sate =1;
				break;
			case 1:
				 last_can_mailbox = CAN_Transmit(CAN1, &USBRxMessage[USBRxIndex]);
				 led_to_notification(LED_CAN_TX);
				 can_send_sate = 2;
				 usb_waittime = time+20;
				 //break;
			
			case 2:
				if(time > usb_waittime)
				{
					can_send_sate=1;
					break;
				}
					
				if(CAN_TransmitStatus(CAN1, last_can_mailbox) == CAN_TxStatus_Ok)
				{
					USBRxIndex++;
					can_send_sate=0;
					break;
				}
				break;
		}

		}
		
	
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
