/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               temp_control.c
** Last modified Date:      2018-05-23
** Last Version:            v1.2
** Description:             
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              张校源
** Created date:            2017-05-16
** Version:                 v1.0
** Descriptions:            试剂盘温度控制程序
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             张校源
** Modified date:           2018-05-22
** Version:                 v1.1
** Description:             增加B3470 温度传感器控制方案
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             张校源
** Modified date:           2018-05-23
** Version:                 v1.2
** Description:             修正负数上传bug 
**													修复内部风扇关闭后不重新打开bug
*********************************************************************************************************/
/*********************************************************************************************************
** 是否启用串口调试功能
*********************************************************************************************************/
#define DEBUG 0
#if DEBUG
#include "usart.h"
#define PRINTF(...)   printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*********************************************************************************************************
	头文件
*********************************************************************************************************/
#include "cooler.h"
#include "BUZZER.h"
#include "temp_control.h"	
/*********************************************************************************************************
	全局变量
*********************************************************************************************************/
extern u32 time;
u32 lm75a_time=0;
u8	lm75_status=0;
u8 gobal_temp_flag=0;  
u16 gobal_temp = 0;
extern u8 usart_state;
char LM75_tx_buffer[20];
char LM75_rx_buffer[20];

LM75_usart_t LM75t;
volatile lm35_temp_t lm35_t;

/*********************************************************************************************************
** Function name:       lm75a_temp_read_polling
** Descriptions:        温度读取任务 周期性读取
** input parameters:    0
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-22
*********************************************************************************************************/
void lm75a_temp_read_polling(void){
	switch(lm35_t.mission_state){          //根据 任务状态号执行
		
		case LM35_READ_IDLE:
			if(time>lm35_t.waitime){
			lm35_t.mission_state=LM35_READ_START;
				lm35_t.times=0;
				lm35_t.temp_all=0;
			}
			break;
			
		case LM35_READ_START:                   //开始读取温度
			
#if USE_LM35
		lm35_t.temp_buffer[lm35_t.times]=Get_Adc_Average(ADC_Channel_2,100); 
#else
		lm35_t.temp_buffer[lm35_t.times]=b3470_get_temperature_offset(B3470_C3); 
#endif 		
			lm35_t.times++;                 //加1                                                          
			if(lm35_t.times>2)
					lm35_t.mission_state=LM35_READ_FINISH;

			break;
		
		case LM35_READ_FINISH:                   //温度读入后的处理
		for(u8 i=0;i<lm35_t.times;i++){
			lm35_t.temp_all+=lm35_t.temp_buffer[i];
		}
		
		lm35_t.temp_real=lm35_t.temp_all/lm35_t.times;  //采样温度求平均  -3 是为了矫正误差        temp_real 是真实温度
		//只有LM35 才进行温度渐变
#if USE_LM35
		lm35_t.temp=temp_faded(lm35_t.temp_real,150);   //********调用温度渐变         temp  是调用渐变后的显示温度	
#else
		lm35_t.temp=lm35_t.temp_real;
#endif 	
			//处理温度在正常范围内渐变
	  
		lm35_t.mission_state=LM35_READ_IDLE;
		lm35_t.waitime=time+1000;
	
		
		if(lm35_t.temp_real<70)
		{             //根据温度选择  蜂鸣器 报警状态
			 lm35_t.temp_alarm_state=1;
		}
		
		if(lm35_t.temp_alarm_state==1)
		if(lm35_t.temp_real>150)
		{
			buzzer_mission(MODE_THREE);
			lm35_t.temp_alarm_state=0;
		}
		
		if(time>lm35_t.close_inter_fan_time && lm35_t.close_inter_fan_enable)   //内部风扇关闭标志 可用  且 时间过了60S   关闭内部风扇
		{
			close_inter_fan();
			lm35_t.close_inter_fan_enable=0;
		}
		
#if !USE_LM35
		if(lm35_t.cooler_function)
		{    //开关制冷条件
#if C3_DIPALY_ONLY
			if(b3470_get_temperature_offset(B3470_C2)<flash_get_para(FLASH_C2_ZL_LOW))
			{
				PRINTF("关闭制冷C2 %d %d\r\n",b3470_get_temperature_offset(B3470_C2),flash_get_para(FLASH_C3_ZL_LOW));
				cooler_off();
				lm35_t.close_inter_fan_time=time+60000;  //延时60S
				lm35_t.close_inter_fan_enable =1;        //关闭内部风扇标志可用
				//设置时间延时关闭风扇
				
			}
			if(b3470_get_temperature_offset(B3470_C2)>flash_get_para(FLASH_C2_ZL_HIGH))
			{
				PRINTF("开启制冷C2 %d %d\r\n",b3470_get_temperature_offset(B3470_C2),flash_get_para(FLASH_C3_ZL_HIGH));
				cooler_on();
				lm35_t.close_inter_fan_enable=0;     //关闭内部风扇标志为不可关
			}
#else 
			if(lm35_t.temp_real<flash_get_para(FLASH_C3_ZL_LOW))
			{
				
				//设置时间延时关闭风扇
				PRINTF("C3关闭温度\r\n");
				lm35_t.c3_control_cooler = 0;
				cooler_off();
				lm35_t.close_inter_fan_time=time+60000;  //延时60S
				lm35_t.close_inter_fan_enable =1;        //关闭内部风扇标志可用
				
			}
			if(lm35_t.temp_real>flash_get_para(FLASH_C3_ZL_HIGH))
			{
				PRINTF("C3开启温度\r\n");
				lm35_t.c3_control_cooler = 1;
			}
#endif
		}
		if(lm35_t.c3_control_cooler & lm35_t.cooler_function)
		{
			if(b3470_get_temperature_offset(B3470_C2)<flash_get_para(FLASH_C2_ZL_LOW))
			{
				PRINTF("关闭制冷C2 %d %d %d\r\n",b3470_get_temperature_offset(B3470_C2),flash_get_para(FLASH_C3_ZL_LOW),flash_get_para(FLASH_C2_STOP_TIME)*1000);
				cooler_off();
				lm35_t.close_cooler_time = time +flash_get_para(FLASH_C2_STOP_TIME)*1000;
				lm35_t.close_inter_fan_time=time+60000;  //延时60S
				lm35_t.close_inter_fan_enable =1;        //关闭内部风扇标志可用
				lm35_t.close_cooler_enable =1;        //关闭内部风扇标志可用
				//设置时间延时关闭风扇
				
			}
			if(b3470_get_temperature_offset(B3470_C2)>flash_get_para(FLASH_C2_ZL_HIGH) || (time > lm35_t.close_cooler_time && lm35_t.close_cooler_enable) )
			{
				PRINTF("开启制冷C2 %d %d\r\n",b3470_get_temperature_offset(B3470_C2),flash_get_para(FLASH_C3_ZL_LOW));
				cooler_on();
				lm35_t.close_inter_fan_enable=0;     //关闭内部风扇标志为不可关
			}
		}
#else 
		if(lm35_t.cooler_function)
		{
		
		if(lm35_t.temp_real<ZL_WD_L)
			{
				
				//设置时间延时关闭风扇
				PRINTF("LM35关闭温度 %d\r\n",lm35_t.temp_real);
				cooler_off();
				lm35_t.close_inter_fan_time=time+60000;  //延时60S
				lm35_t.close_inter_fan_enable =1;        //关闭内部风扇标志可用
				
			}
			if(lm35_t.temp_real>ZL_WD_H)
			{
				cooler_on();
				lm35_t.close_inter_fan_enable=0;     //关闭内部风扇标志为不可关
				PRINTF("LM35开启温度%d\r\n",lm35_t.temp_real);
			}
		}
#endif
		
		
		
		break;
		default:
			lm35_t.mission_state = LM35_READ_IDLE;
			break;
	}
	
}


u16 temp_faded(u16 temp , u16 max_temp){   //温度渐变程序（查询温度时接收到真实温度，最大温度）
		//如果大于15， 标志为0，返回这个温度
		if(temp>=max_temp)
		{           
			gobal_temp =temp;
			gobal_temp_flag=0;
			return gobal_temp;
		}
  
		//小于15的时候
		if(!gobal_temp_flag)
		{          //读取温度值  gobal_temp_flag默认为0
			gobal_temp_flag=1;
			gobal_temp = temp;
		}
		
		if(gobal_temp_flag)
		{             //如果新温度相等，不变，如果相差0.1，更新，如果有变化，0.2为单位变化
			if(gobal_temp == temp)
				return gobal_temp;
			
			if((gobal_temp - temp == 1) || (temp - gobal_temp == 1))
			{
				gobal_temp = temp;
				return gobal_temp;
			}
				
			if(gobal_temp < temp)
			{
				gobal_temp += 2;
				return gobal_temp;
			}	
				
			if(gobal_temp > temp)
			{
				gobal_temp -= 2;
				return gobal_temp;
			}			
		}
   return temp;
}


// *************************** 温度显示屏任务***************************//
void temp_display_mission(void){
	float temp;
	u8 length=10;
	switch(LM75t.mission_state){
		
		case SCREEN_DIS_IDLE:
			if(usart_state==0)
				 break;
					
			if(time>=LM75t.periodtime && sut.mission_state == SCREEN_IDLE)    //显示屏空闲判断
					LM75t.mission_state=SCREEN_DIS_PENDING;	
			break;
					
			
			
			
			
		case SCREEN_DIS_START:
			
		if(sut.mission_state!=SCREEN_IDLE)
			break;
		
		if(LM75t.retry_count>1){
					LM75t.index=2;
					LM75t.decide=lm35_t.temp%10;
					TM_USART_DMA_Send(USART1, (uint8_t *)LM75_tx_buffer,LM75_parpare_buffer());
				}
		else{
					 length=10;
					 sprintf(LM75_tx_buffer,"rank.val=1");//字符串写入缓存
					 LM75_tx_buffer[length++]=0xFF;
					 LM75_tx_buffer[length++]=0xFF;
					 LM75_tx_buffer[length++]=0xFF;
					TM_USART_DMA_Send(USART1, (uint8_t *)LM75_tx_buffer,length);//串口发送发送数据
				}

				 
					LM75t.mission_state=SCREEN_DIS_START_CHECK;  //检查是否开始工作
					LM75t.timeout=time+200;
				//LM75t.mission_state=LM75T_IDLE;
		break;
		
		
				
				
				
		case  SCREEN_DIS_START_CHECK:     //检查状态
					if(sut.mission_state!=SCREEN_IDLE)
							break;
				
					//如果还是空闲的 ，再次尝试 间隔0.2S
				if(time>LM75t.timeout){
					LM75t.retry_count++;
					LM75t.timeout=time+200;
					
					if(LM75t.retry_count>3){
						mission_failed_send(TEMP_DISPLAY);  //重试次数超过3次，报失败
						LM75t.mission_state=SCREEN_DIS_IDLE;
						LM75t.retry_count=0;
					}
					
							LM75t.mission_state=SCREEN_DIS_START;
					break;
				}
				
				if(TM_USART_Gets(USART1,LM75_rx_buffer,5)){//如果收到返回的确认帧，报显示成功
					if(LM75_rx_buffer[0]==LM75_ACK){
						LM75t.mission_state=SCREEN_DIS_IDLE;
						usart_state=1;              //允许传输数据
						mission_success_send(TEMP_DISPLAY);
				}
			}
		
		break;
		
			
			
		case SCREEN_DIS_PENDING:
			    temp=lm35_t.temp/10;
					if(temp>12){
			    LM75t.rank=3;  //红色
		      }else if((temp>8)&&(temp<=12)){
			    LM75t.rank=2;  //黄色
		      }else if(temp<=8){//(temp>=5)&&(temp<=8)
			    LM75t.rank=1;  //蓝色，正常
		      }
					
					LM75t.rank=1;                     //???????????????????????????????????????????  目前由屏幕自己判断
					LM75t.Integred=lm35_t.temp/10;       //温度整数位转换
				  LM75t.decide=lm35_t.temp%10;         //温度小数位转换
		      LM75t.mission_state=SCREEN_DIS_QUERY;
					LM75t.waitime=time+LM75WAITIME;
					LM75t.index=1;
			break;
					
					
		case SCREEN_DIS_QUERY:  //定时发送温度
			if(sut.mission_state != SCREEN_IDLE)
				break;
		 if(time>LM75t.waitime)
		 {
				TM_USART_DMA_Send(USART1, (uint8_t *)LM75_tx_buffer,LM75_parpare_buffer());
			//set timer
				LM75t.timeout=time+LM75_TIMEOUT;//设置超时时间
				LM75t.mission_state=SCREEN_DIS_POLLOING;
		 }
			break;
		
		case SCREEN_DIS_POLLOING:
			if(time>LM75t.timeout){//单片机发送出去后没有收到应答
				LM75t.retry_count++;
				
				if(LM75t.retry_count>LM75_MAX_RETRY){
					LM75t.mission_state=SCREEN_DIS_FAILED;
					break;
				}
				LM75t.mission_state=SCREEN_DIS_PENDING;//不到5次就在这个状态查询是否超时5次
				break;
			}
				
			if(TM_USART_Gets(USART1,LM75_rx_buffer,5)){
				if(LM75_rx_buffer[0]==LM75_ACK){
						LM75t.index++;
						LM75t.retry_count=0;
					  LM75t.waitime=time+LM75WAITIME;
						LM75t.mission_state=SCREEN_DIS_QUERY;
				}
			}
				if(LM75t.index>=3){
					LM75t.mission_state=SCREEN_DIS_SUCCESS;
				}
					break;
				
		case SCREEN_DIS_FAILED:
			LM75t.index=0;
			LM75t.mission_state=SCREEN_DIS_IDLE;
			LM75t.retry_count=0;
		  LM75t.periodtime=time+LM75PRED_TIME;
			break;
		
		case SCREEN_DIS_SUCCESS:
				LM75t.index=0;
				LM75t.mission_state=SCREEN_DIS_IDLE;
				LM75t.retry_count=0;
		    LM75t.periodtime=time+LM75PRED_TIME;
			break;
		
		default:
				LM75t.mission_state=SCREEN_DIS_IDLE;
		break;
			}
	}
//********************************************************温度显示屏任务结束*************************************//

u8 LM75_parpare_buffer(){    //修改颜色、温度值到发送缓存
		u8 length;
		if(LM75t.index<3){
			 if(LM75t.index==0)//温度等级
			 {
				 length=10;
			   sprintf(LM75_tx_buffer,"rank.val=%d",LM75t.rank);
				 LM75_tx_buffer[length++]=0xFF;
         LM75_tx_buffer[length++]=0xFF;
         LM75_tx_buffer[length++]=0xFF;
			 }
			 if(LM75t.index==1)//整数赋值
			 {
				 if(LM75t.Integred>9)
				 length=9;
				 if(LM75t.Integred<10)
					 length=8;
			   sprintf(LM75_tx_buffer,"n0.val=%d",LM75t.Integred);
				 LM75_tx_buffer[length++]=0xFF;
         LM75_tx_buffer[length++]=0xFF;
         LM75_tx_buffer[length++]=0xFF;
			 }
			 if(LM75t.index==2)//小数赋值 
			 {
				 length=8;
			   sprintf(LM75_tx_buffer,"n7.val=%d",LM75t.decide);
				 LM75_tx_buffer[length++]=0xFF;
         LM75_tx_buffer[length++]=0xFF;
         LM75_tx_buffer[length++]=0xFF;
			 }
		}
		return length;
}


void reser_screen(void){
	       u8 length;
				 length=6;
			   sprintf(LM75_tx_buffer,"page 0");
				 LM75_tx_buffer[length++]=0xFF;   //FF FF FF 为结束符
         LM75_tx_buffer[length++]=0xFF;
         LM75_tx_buffer[length++]=0xFF;
	       TM_USART_DMA_Send(USART1, (uint8_t *)LM75_tx_buffer,length);//串口发送发送数据
        }

void lm75a_mission_polling(void){
	int16_t temp_int;
	char temp_frame[3];
	
		switch(	lm75_status){
			case LM75_IDLE:
				break;
			
			case	LM75_PENGDING:
				PRINTF("temp is %d \r\n",lm35_t.temp);
				if(lm35_t.temp>550)
				{
					
					lm75_status=LM75_FALIED;
				}else{
					lm75_status=LM75_SUCCESS;
				}
				
				break;
				
			case LM75_SUCCESS:
				temp_int=lm35_t.temp;			//25.2 >> 252
				
				temp_frame[0]=0x80;
				temp_frame[1]=temp_int>>8;
				temp_frame[2]=temp_int>>0;
				mission_state[TEMP_QUERY]=0x80;
				one_can_frame_send((u8 *)temp_frame,3,TEMP_QUERY);
			
				temp_int=b3470_get_temperature_offset(B3470_C2);			//25.2 >> 252
				
				temp_frame[0]=0x80;
				temp_frame[1]=temp_int>>8;
				temp_frame[2]=temp_int>>0;
				mission_state[TEMP_QUERY]=0x80;
				one_can_frame_send((u8 *)temp_frame,3,0xAA);
				lm75_status=LM75_IDLE;
			break;
			
			case	LM75_FALIED:
				mission_failed_send(TEMP_QUERY);
				lm75_status=LM75_IDLE;
			break;
				
			default :
				break;
		}
	
}




