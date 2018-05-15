#include "lm75a.h"	
#include "cooler.h"
#include "MSD_test.h"  
#include "BUZZER.h"
__IO uint16_t ADC1OscConver[ADC_SIZE];
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
static void Adc1IoConfig(void)    //内部函数
{
    /*
    IO口配置
    */
    GPIO_InitTypeDef    GPIO_InitStructure;
    //开C口时钟，
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    //复位配置
    GPIO_StructInit(&GPIO_InitStructure);
    //设置为模拟输入模式
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
    //不带上拉
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
    //将以上设置应用于寄存器
    GPIO_Init(GPIOA,&GPIO_InitStructure);
}
/*
功能:tim2触发器初始化
*/
static void Tim2Config(void)
{
    /*
    tim配置
    */
    TIM_TimeBaseInitTypeDef         TIM_TimeBaseInitStructure;
    //打开TIM时钟8
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    //初始化配置结构,不受其它配置干扰
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    //对APB1时钟不分频,由system_stm32f4xx.c可知
    //APB1=AHB/4=SYSCLK/4=168M/4=42M,则TIM3=APB1*2=84M
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    //分频值=168M/(Prescaler+1)/2=0.5us
    TIM_TimeBaseInitStructure.TIM_Prescaler=42-1;
    //溢出时间确定
    TIM_TimeBaseInitStructure.TIM_Period=44;
    //向上计数方式
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
    //将以上配置应用于定时器
    TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);
    //数据更新作为触发源
    TIM_SelectOutputTrigger(TIM8,TIM_TRGOSource_Update);
}
/*
功能:ADC1的示波器功能配置
备注:使用tim2做触发
*/
void Adc1OscConfig(void)
{
     
    /*
    dma配置
    */
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;
    //开启MDA2时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    //初始化各寄存器配置
    DMA_DeInit(DMA2_Stream0);
    DMA_StructInit( &DMA_InitStructure);
    //选取DMA通道0,数据流0
    DMA_InitStructure.DMA_Channel=DMA_Channel_0;
	
    //数据传入地址->ADC基地址加上DR寄存器偏移地址
    DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)ADC1_DR_Address;
    //数据送入地址
    DMA_InitStructure.DMA_Memory0BaseAddr=(uint32_t)ADC1OscConver;
    //数据传送方向为外设到SRAM
    DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralToMemory;
    //数据缓冲区1
    DMA_InitStructure.DMA_BufferSize=ADC_SIZE;
    //外设地址固定
    DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
    //内存地址自增
    DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
    //数据类型为半字
    DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;//DMA_PeripheralDataSize_Byte;//DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;//DMA_MemoryDataSize_Byte;//DMA_MemoryDataSize_HalfWord;
    //循环传输
    DMA_InitStructure.DMA_Mode=DMA_Mode_Circular;
    //高优先级
    DMA_InitStructure.DMA_Priority=DMA_Priority_High;
    //不使用FIFO模式
    DMA_InitStructure.DMA_FIFOMode=DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold=DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst=DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;
     
//  DMA_DoubleBufferModeCmd(DMA2_Stream0, DISABLE);
    //将以上设置应用于DMA2，通道0，数据流0
    DMA_Init(DMA2_Stream0,&DMA_InitStructure);
    //使能DMA
    DMA_Cmd(DMA2_Stream0, ENABLE);
    //选择DMA2通道数据流0
    NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream0_IRQn;//DMA2_Stream0_IRQHandler;
    //抢占式优先级为0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
    //响应式优先级为12
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=4;
    //通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    //将以上配置应用于NVIC
    NVIC_Init(&NVIC_InitStructure);
    //使能DMA传输完成中断
   // DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
     
    /*
    功能:ADC配置
    */
    ADC_InitTypeDef ADC_InitStructure;
    //开启ADC1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    //12位转换精度
    ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b;//ADC_Resolution_8b;//ADC_Resolution_12b; 
    //使用单通道转换模式
    ADC_InitStructure.ADC_ScanConvMode=DISABLE;
    //不使用多次转换模式
    ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;//ENABLE;//DISABLE;
    //使用外部上升沿触发模式
    ADC_InitStructure.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_Rising;//ADC_ExternalTrigConvEdge_Rising;//ADC_ExternalTrigConvEdge_None;
    //TIM3溢出触发
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_TRGO;//ADC_ExternalTrigConv_T1_CC1;
    //选择右对齐方式
    ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
    //选用转换通道数为1个
    ADC_InitStructure.ADC_NbrOfConversion=1;
    //将以上设置应用于对应寄存器
    ADC_Init(ADC1,&ADC_InitStructure);
    //使能ADC在DMA模式下的连续转换
    ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);
    //使能ADC的DMA模式
    ADC_DMACmd(ADC1,ENABLE);
    //配置ADC1规则组(得出其单次采样时间为:(3+12)/21≈0.7us)->12bit
    ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_480Cycles);//ADC_SampleTime_3Cycles/ADC_SampleTime_480Cycles
    //使能ADC
    ADC_Cmd(ADC1,ENABLE);
}
 
/*
功能:各ADC共同功能配置
*/
static void AdcSenCommConfig(void)
{
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;
    //ADC为独立模式
    ADC_CommonInitStructure.ADC_Mode=ADC_Mode_Independent;
	
    //ADC时钟为APB2的2分频->84/4=21MHZ(F407ADC在2.4-3.6V供电电压下最大速率36M,稳定速度为30M)
    ADC_CommonInitStructure.ADC_Prescaler=ADC_Prescaler_Div8;
	
    //使用ADC的DMA复用
    ADC_CommonInitStructure.ADC_DMAAccessMode=ADC_DMAAccessMode_2;
	
    //两次采样的间隔时间为5个ADC时钟周期(5/21≈0.24us)
    ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles;
    //将以上设置应用于对应的寄存器
    ADC_CommonInit(&ADC_CommonInitStructure);
}
/*
功能:开始ADC1示波器采样
*/
void StartAdc1OscSam(void)
{
 TIM_Cmd(TIM8, ENABLE);
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
}
/*
功能:停止ADC1示波器采样
*/
void StopAdc1OscSam(void)
{
 TIM_Cmd(TIM8, DISABLE);
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,DISABLE);
}

u8 lm75a_init(){

	Adc1IoConfig();
	AdcSenCommConfig();
	Tim2Config();
	Adc1OscConfig();
	
	return 0;
}

	
void lm75a_temp_read_polling(void){
//	u32 temp_val;
	switch(lm35_t.mission_state){          //根据 任务状态号执行
		
		case LM35_READ_IDLE:
			if(time>lm35_t.waitime){
			lm35_t.mission_state=LM35_READ_START;
				lm35_t.times=0;
				lm35_t.temp_all=0;
				//StartAdc1OscSam();
			}
			break;
			
		case LM35_READ_START:                   //开始读取温度
		/*	if(motor.running_state!=M_IDLE)
				break;
		*/
			lm35_t.temp_buffer[lm35_t.times]=Get_Adc_Average(ADC_Channel_2,100);      
			lm35_t.times++;                 //加1                                                          
			if(lm35_t.times>4)
					lm35_t.mission_state=LM35_READ_FINISH;
				
			break;
		
		case LM35_READ_FINISH:                   //温度读入后的处理
			
		for(u8 i=0;i<lm35_t.times;i++){
			//lm35_t.temp+=lm35_t.temp_buffer[i];
			lm35_t.temp_all+=lm35_t.temp_buffer[i];
		}
		
		lm35_t.temp_real=lm35_t.temp_all/lm35_t.times-3;  //采样温度求平均  -3 是为了矫正误差        temp_real 是真实温度
		
			//处理温度在正常范围内渐变
	  lm35_t.temp=temp_faded(lm35_t.temp_real,150);   //********调用温度渐变         temp  是调用渐变后的显示温度	

/*		if(75<lm35_t.temp_real<=80)
			 lm35_t.temp-=5;
  	else if(80<lm35_t.temp_real<90)
			lm35_t.temp-=10;
*/
		
		lm35_t.mission_state=LM35_READ_IDLE;
		lm35_t.waitime=time+10000;
	
		
		if(lm35_t.temp_real<70){             //根据温度选择  蜂鸣器 报警状态
		lm35_t.temp_alarm_state=1;
		}
		
		if(lm35_t.temp_alarm_state==1)
		if(lm35_t.temp_real>150){
			buzzer_mission(MODE_THREE);
			lm35_t.temp_alarm_state=0;
		}
		
		if(time>lm35_t.close_inter_fan_time && lm35_t.close_inter_fan_enable)   //内部风扇关闭标志 可用  且 时间过了60S   关闭内部风扇
			close_inter_fan();
		
		if(lm35_t.cooler_function){    //开关制冷条件
			if(lm35_t.temp_real<ZL_WD_L){
				cooler_off();
				lm35_t.close_inter_fan_time=time+60000;  //延时60S
				lm35_t.close_inter_fan_enable =1;        //关闭内部风扇标志可用
				//设置时间延时关闭风扇
				
			}
			if(lm35_t.temp_real>ZL_WD_H){
				cooler_on();
				lm35_t.close_inter_fan_enable=0;     //关闭内部风扇标志为不可关
			}
		}
		break;
	}
	
}


u16 temp_faded(u16 temp , u16 max_temp){   //温度渐变程序（查询温度时接收到真实温度，最大温度）
		
		if(temp>=max_temp){           //如果大于15， 标志为0，返回这个温度
			gobal_temp =temp;
			gobal_temp_flag=0;
			return gobal_temp;
		}
  
		
		
		//小于15的时候
		if(!gobal_temp_flag){          //读取温度值  gobal_temp_flag默认为0
			gobal_temp_flag=1;
			gobal_temp = temp;
		}
		
		
		if(gobal_temp_flag){             //如果新温度相等，不变，如果相差0.1，更新，如果有变化，0.2为单位变化
			if(gobal_temp == temp)
				return gobal_temp;
			
			if((gobal_temp - temp == 1) || (temp - gobal_temp == 1)){
				gobal_temp = temp;
				return gobal_temp;
			}
			
			if(gobal_temp < temp){
				gobal_temp += 2;
				return gobal_temp;
			}	
			
			if(gobal_temp > temp){
				gobal_temp -= 2;
				return gobal_temp;
			}			
		}
   return temp;
}


// *************************** 温度显示屏任务***************************//
void LM75_mission(void){
	float temp;
	u8 length=10;
	switch(LM75t.mission_state){
		
		case LM75T_IDLE:
			if(usart_state==0)
				 break;
					
			if(time>=LM75t.periodtime && sut.mission_state == SCREEN_IDLE)    //显示屏空闲判断
					LM75t.mission_state=LM75T_PENDING;	
			break;
					
			
			
			
			
		case LM75_START:
			
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

				 
					LM75t.mission_state=LM75_START_CHECK;  //检查是否开始工作
					LM75t.timeout=time+200;
				//LM75t.mission_state=LM75T_IDLE;
		break;
		
		
				
				
				
		case  LM75_START_CHECK:     //检查状态
					if(sut.mission_state!=SCREEN_IDLE)
							break;
				
					//如果还是空闲的 ，再次尝试 间隔0.2S
				if(time>LM75t.timeout){
					LM75t.retry_count++;
					LM75t.timeout=time+200;
					
					if(LM75t.retry_count>3){
						mission_failed_send(TEMP_DISPLAY);  //重试次数超过3次，报失败
						LM75t.mission_state=LM75T_IDLE;
						LM75t.retry_count=0;
					}
					
							LM75t.mission_state=LM75_START;
					break;
				}
				
				if(TM_USART_Gets(USART1,LM75_rx_buffer,5)){//如果收到返回的确认帧，报显示成功
					if(LM75_rx_buffer[0]==LM75_ACK){
						LM75t.mission_state=LM75T_IDLE;
						usart_state=1;              //允许传输数据
						mission_success_send(TEMP_DISPLAY);
				}
			}
		
		break;
		
			
			
		case LM75T_PENDING:
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
		      LM75t.mission_state=LM75T_QUERY;
					LM75t.waitime=time+LM75WAITIME;
					LM75t.index=1;
			break;
					
					
		case LM75T_QUERY:  //定时发送温度
			if(sut.mission_state != SCREEN_IDLE)
				break;
		 if(time>LM75t.waitime)
		 {
				TM_USART_DMA_Send(USART1, (uint8_t *)LM75_tx_buffer,LM75_parpare_buffer());
			//set timer
				LM75t.timeout=time+LM75_TIMEOUT;//设置超时时间
				LM75t.mission_state=LM75T_POLLOING;
		 }
			break;
		
		case LM75T_POLLOING:
			if(time>LM75t.timeout){//单片机发送出去后没有收到应答
				LM75t.retry_count++;
				
				if(LM75t.retry_count>LM75_MAX_RETRY){
					LM75t.mission_state=LM75T_FAILED;
					break;
				}
				LM75t.mission_state=LM75T_PENDING;//不到5次就在这个状态查询是否超时5次
				break;
			}
				
			if(TM_USART_Gets(USART1,LM75_rx_buffer,5)){
				if(LM75_rx_buffer[0]==LM75_ACK){
						LM75t.index++;
						LM75t.retry_count=0;
					  LM75t.waitime=time+LM75WAITIME;
						LM75t.mission_state=LM75T_QUERY;
				}
			}
				if(LM75t.index>=3){
					LM75t.mission_state=LM75T_SUCCESS;
				}
					break;
				
		case LM75T_FAILED:
			LM75t.index=0;
			LM75t.mission_state=LM75T_IDLE;
			LM75t.retry_count=0;
		  LM75t.periodtime=time+LM75PRED_TIME;
			break;
		
		case LM75T_SUCCESS:
				LM75t.index=0;
				LM75t.mission_state=LM75T_IDLE;
				LM75t.retry_count=0;
		    LM75t.periodtime=time+LM75PRED_TIME;
			break;
		
		default:
				LM75t.mission_state=LM75T_IDLE;
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
	u16 temp_int;
	char temp_frame[3];
	
		switch(	lm75_status){
			case LM75_IDLE:
				break;
			
			case	LM75_PENGDING:
				
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


void bubble_sort_better(__IO u16 a[],u16 n)  //这是什么鬼？  跟ADC有关
{
    for(u16 i=0; i<n-1; i++)
    {
        u16 isSorted = 0;
        for(u16 j=0; j<n-1-i; j++)
        {
            if(a[j] > a[j+1])
            {
                isSorted = 1;
                u16 temp = a[j];
                a[j] = a[j+1];
                a[j+1]=temp;
            }
        }
        if(isSorted) break; 
    }
}

//


void DMA2_Stream0_IRQHandler(void)     //MDA2 中断处理
{	
	
	
	 	if(DMA_GetFlagStatus(DMA2_Stream0,DMA_IT_TC)==SET)  
   {   
		 StopAdc1OscSam(); //关闭
		 lm35_t.mission_state=LM35_READ_FINISH;		 
		DMA_ClearFlag(DMA2_Stream0,DMA_IT_TC);	
}
	
} 


