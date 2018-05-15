#include "one_dimension_code.h"
char USART_Buffer2[200];
char one_dimension_code_state=0;
odc_t odc;
extern u32 time;

void one_dimension_code_init(){          //二维码部分初始化任务，各种初始化，初始化端口，通信线，波特率，时间什么的
	TM_GPIO_Init(GPIOD, GPIO_Pin_7, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High); //初始化端口
	TM_USART_Init(USART2, TM_USART_PinsPack_2, 9600);  //
	TM_USART_DMA_Init(USART2);
	TM_GPIO_SetPinHigh(GPIOD, GPIO_Pin_7);
	odc.waittime=0;
	odc.timeout=0;
}

void one_dimension_code_start(){    //打开扫码头 PD7置0
	//TM_GPIO_TogglePinValue(GPIOD, GPIO_Pin_7);
	TM_GPIO_SetPinLow(GPIOD, GPIO_Pin_7);
	
}


uint16_t one_dimension_code_read(){                        //   读二维码
	return TM_USART_Gets(USART2, odc.buf, sizeof(odc.buf));
}

void one_dimension_code_mission_polling(){  //one_dimension_code 二维码任务
	switch(odc.running_state){
		case ODC_IDLE:            //0x00   无动作
		break;
		
		case ODC_ENABLE:      //0x01      使能
			TM_GPIO_SetPinLow(GPIOD, GPIO_Pin_7);   // PD7 置低     开
			odc.running_state=ODC_DATAREAD;        //状态设置为读数据
			odc.timeout=time+ODC_OUTTIME;          // 超时时间= 工作时间+输出时间750
		break;
		
		case ODC_DATAREAD:       //0x02    读数据
			
			if(time>odc.timeout)                      //如果实际时间超时了
				odc.running_state=ODC_TIMEOUT;          //运行状态设置为    0x03 超时
				odc.length=one_dimension_code_read();   // 长度设置为 ...
			if(odc.length){                            // 逻辑判断   如果长度 为非0 则为真 ，此处即 如果长度非0
				TM_GPIO_SetPinHigh(GPIOD, GPIO_Pin_7);   // PD7 置高     关
				odc.running_state=ODC_SUCCESS;           // 状态设置为成功
				buzzer_mission(MODE_TWO);                 // 蜂鸣器任务  2  
			}
			break;
			
		case ODC_TIMEOUT:         //0x03   超时
			TM_GPIO_SetPinHigh(GPIOD, GPIO_Pin_7);   // PD7 置高 关闭
			odc.timeoutCount++;                          //超时计数+1
			if(odc.timeoutCount>=ODC_MAX_RETRY){         //如果超过了重试次数
					//mission_failed_send(ODC_UPLOAD);      //返回失败标志
				mission_success_send(ODC_UPLOAD);       //
					odc.running_state=ODC_IDLE;          //不做动作
					odc.timeoutCount=0;                  //超时次数清零
					break;
				}
			odc.running_state=ODC_PENDDING;        //运行状态 待定
			odc.waittime=time+ODC_WAITTIME;
		break;
		
		case ODC_PENDDING:             //0x04   待定
			if(time>=odc.waittime)         // 等一段时间后，使能 
				odc.running_state=ENABLE;
			break;
			
		case ODC_SUCCESS:             //0x05    成功
			
			if((odc.length)==0){                  //没收到数据， 状态设置为超时
			odc.running_state=ODC_TIMEOUT;
				break;
			}
			//odc.buf[odc.length]=0x0A;
			//odc.length=odc.length+1;
			odc.waittime=time+ODC_WAITTIME;
			
			can_mutil_frame_send((uint8_t *)odc.buf,odc.length,0xA0);
			mission_success_send(ODC_UPLOAD);   //返回 
			odc.running_state=ODC_IDLE;
			odc.length=0;
			odc.timeoutCount=0;           //之前没有这一句， 超时计数可能累积
		break;
		
	}
	
	
}

