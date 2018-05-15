#include "notification.h"
led_mission_t lmt;
extern u32 time;

void	LED_init(void){
	 GPIO_InitTypeDef GPIO_InitStructure;
	
		LED_AHBxClock_FUN(LED_GPIO_CLK,ENABLE);
	
		GPIO_InitStructure.GPIO_Pin =  LED_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
	
		LED_OFF(LED_PIN);
}

void	LED1_init(void){
	 GPIO_InitTypeDef GPIO_InitStructure;
	
		LED1_AHBxClock_FUN(LED1_GPIO_CLK,ENABLE);
	
		GPIO_InitStructure.GPIO_Pin =  LED1_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED1_PORT, &GPIO_InitStructure);
	
		LED1_OFF(GREEN_LED1);
		LED1_ON(RED_LED1);			//开机启动时，指示灯显示红色
}

void LED_ON(uint16_t LED){
	GPIO_ResetBits(LED_PORT,LED);

}

void LED_OFF(uint16_t LED){
		GPIO_SetBits(LED_PORT,LED);

}

void LED_TOGGLE(uint16_t LED){
	GPIO_WriteBit(LED_PORT,LED,(BitAction)(1 -GPIO_ReadOutputDataBit( LED_PORT, LED )));
}

void LED1_ON(uint16_t LED1){
	GPIO_ResetBits(LED1_PORT,LED1);

}

void LED1_OFF(uint16_t LED1){
		GPIO_SetBits(LED1_PORT,LED1);

}

//LED 显示状态

void led_mission_polling(void){
	switch(lmt.mission_state){
		case LED_IDLE:
			break;
		
		case LED_UP:
			if(lmt.current_mission==LED_CAN_RX){
				LED_ON(LED_CAN);
				lmt.can_rx_time=time+MIN_SLOT;
				lmt.mission_state=LED_DOWN;
			}
			
			if(lmt.current_mission==LED_USB_UPLOAD){
				LED_ON(LED_UPLOAD);
				lmt.usb_tx_time=time+MIN_SLOT;
				lmt.mission_state=LED_DOWN;
			}
			
			if(lmt.current_mission==LED_CAN_TX){
				LED_ON(LED_CAN_SEND);
				lmt.usb_rx_time=time+MIN_SLOT;
				lmt.mission_state=LED_DOWN;
			}
			
			if(lmt.current_mission==LED_USB_POLLING){
				LED_ON(LED_USB_TX);
				lmt.usb_polling_time=time+MIN_SLOT;
				lmt.mission_state=LED_DOWN;
			}
			
			lmt.current_mission=LED_MISSION_IDLE;
			break;
			
		case LED_DOWN:
			
			if(time>lmt.can_rx_time){
				LED_OFF(LED_CAN);
			}

			if(time>lmt.usb_tx_time){
				LED_OFF(LED_UPLOAD);
			}
			
			if(time>lmt.usb_rx_time){
				LED_OFF(LED_CAN_SEND);
			}
			
			if(time>lmt.usb_polling_time){
				LED_OFF(LED_USB_TX);
			}
						
			if(time>lmt.can_rx_time && time>lmt.usb_rx_time && time>lmt.usb_tx_time && time>lmt.usb_polling_time)
				lmt.mission_state=LED_IDLE;
			
			break;
	}

}

void led_to_notification(u8 mission){
	lmt.mission_state=LED_UP;
	lmt.current_mission=mission;

}

