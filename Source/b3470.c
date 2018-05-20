/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               b3470.c
** Last modified Date:      2018-05-18
** Last Version:            v1.1
** Description:             热敏电阻NTC 温度传感器 B3470
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              张校源
** Created date:            2018-05-16
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             张校源
** Modified date:           2018-05-18
** Version:                 v1.1
** Description:             增加负温度读取(小数部分未校准)
**
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
#include "b3470.h"
#include "stmflash.h"
/*********************************************************************************************************
	全局变量
*********************************************************************************************************/
u16 table_B3470[B3470_TEMP_RANGE] = {
    705,  738,  771,  806,  841,   // -20  -  -16
    878,  915,   954, 994, 1035,   // -15  -  -11
    1077, 1119, 1163, 1208, 1254,   // -10  -  -6
    1301, 1349, 1398, 1448, 1499,   //  -5  -  -1
    1550, 1603, 1656 ,1710, 1765,  	//  0   -   4
	1820, 1876, 1933, 1990, 2048, 	//  5   -   9
	2106, 2165, 2224, 2283, 2343,   //  10  -   14
	2403, 2463, 2524, 2584, 2645, 	//  15  -   19
	2705, 2766, 2826, 2886, 2946,	//  20  -  24
	3006, 3066, 3125, 3184, 3243,	//  25  -  29
	3301, 3358, 3416, 3472, 3528, 	//  30  -  34
	3584, 3639, 3693, 3747, 3799, 	//  35  -  39
	3852, 3903, 3954, 4004, 4053, 	//  40  -  44
	4101, 4149, 4196, 4242, 4287, 4332   //  45  -  50
};

/*********************************************************************************************************
** Function name:       b3470_init
** Descriptions:        初始化b3470对应的ADC GPIO 
** input parameters:    0
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-16
*********************************************************************************************************/
void  b3470_init(void)
{    
  GPIO_InitTypeDef  		GPIO_InitStructure;               //定义结构体变量
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(B3470_IO_CLOCK_1, ENABLE);			//使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(B3470_IO_CLOCK_2, ENABLE);			//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(B3470_ADC_CLOCK, ENABLE);		 //使能ADC1时钟

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = B3470_IO_GPIO_PIN_1;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;			//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//不带上下拉   
  GPIO_Init(B3470_IO_GPIO_PORT_1, &GPIO_InitStructure);//初始化  
	
	GPIO_InitStructure.GPIO_Pin = B3470_IO_GPIO_PIN_2;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;			//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//不带上下拉   
  GPIO_Init(B3470_IO_GPIO_PORT_2, &GPIO_InitStructure);//初始化  
 
	RCC_APB2PeriphResetCmd(B3470_ADC_CLOCK,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(B3470_ADC_CLOCK,DISABLE);	//复位结束	 
	
	
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(B3470_ADC_PERIPH, &ADC_InitStructure);//ADC初始化
	
	ADC_Cmd(B3470_ADC_PERIPH, ENABLE);//开启AD转换器	
}				  

/*********************************************************************************************************
** Function name:       bubble_sort_better
** Descriptions:        冒泡排序法
** input parameters:    数组指针 和 长度
** output parameters:   传递的数组指针
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-16
*********************************************************************************************************/
static void bubble_sort_better( u16 a[],u16 n)
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


/*********************************************************************************************************
** Function name:       adc_value_get
** Descriptions:        获取B3470_ADC_PERIPH 对应channel的转换结果
** input parameters:    ADC_Channel
** output parameters:   0
** Returned value:      ADC_Channel_value
** Created by:          张校源
** Created Date:        2018-05-16
*********************************************************************************************************/
u16 adc_value_get(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(B3470_ADC_PERIPH, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(B3470_ADC_PERIPH);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(B3470_ADC_PERIPH, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(B3470_ADC_PERIPH);	//返回最近一次ADC1规则组的转换结果
}


/*********************************************************************************************************
** Function name:       adc_value_filter
** Descriptions:        多次ADC 转换 去除极大极小值 加上平均滤波
** input parameters:    ADC_Channel 转换次数
** output parameters:   0
** Returned value:      ADC_Channel_filter_value
** Created by:          张校源
** Created Date:        2018-05-16
*********************************************************************************************************/
u16 adc_value_filter(u8 ch)
{
	u32 temp_val=0;
	u16 temp[B3470_ADC_FUN_CONV_TIMES];
	u8 t;
	for(t=0;t<B3470_ADC_FUN_CONV_TIMES;t++)
	{
		temp[t]=adc_value_get(ch);
	}
	bubble_sort_better(temp,B3470_ADC_FUN_CONV_TIMES);
	
	for(t=B3470_ADC_FUN_OFF_TAIL;t<B3470_ADC_FUN_CONV_TIMES-B3470_ADC_FUN_OFF_TAIL;t++)
		temp_val+=temp[t];
	
	return temp_val/(B3470_ADC_FUN_CONV_TIMES-2*B3470_ADC_FUN_OFF_TAIL);
}

/*********************************************************************************************************
** Function name:       adc_value_conv_temperature
** Descriptions:        根据传递出来的参数查找表格
** input parameters:    ADC 采样值
** output parameters:   无
** Returned value:      温度
** Created by:          张校源
** Created Date:        2018-05-14
*********************************************************************************************************/
int16_t adc_value_conv_temperature(u16 adc_value)
{
	int16_t temp_h;
	u16 temp_low;
	for(u8 i=0;i<B3470_TEMP_RANGE; i++)
	{
		if(adc_value < table_B3470[i])
		{
			temp_h= (i - 22)*10;
			//TODO 负温度小数部分校准
			temp_low = (adc_value -table_B3470[i-1])*10/(table_B3470[i] -table_B3470[i-1]);
			return temp_h + temp_low;
		}
	}
	return 500;
}

/*********************************************************************************************************
** Function name:       get_temperature
** Descriptions:        查询对应接口的温度传感器 
** input parameters:    ADC 通道 （B3470_C3 ，B3470_C2）
** output parameters:   无
** Returned value:      温度
** Created by:          张校源
** Created Date:        2018-05-14
*********************************************************************************************************/

int16_t b3470_get_temperature(u8 ch)
{
	return adc_value_conv_temperature(adc_value_filter(ch));
}

/*********************************************************************************************************
** Function name:       b3470_get_temperature_offset
** Descriptions:        返回偏移之后的温度数据 
** input parameters:    ADC 通道 （B3470_C3 ，B3470_C2）
** output parameters:   无
** Returned value:      温度
** Created by:          张校源
** Created Date:        2018-05-14
*********************************************************************************************************/

int16_t b3470_get_temperature_offset(u8 ch)
{
	if(ch!=B3470_C3 && ch!=B3470_C2 )
		PRINTF("B3470 channel is not right %d",ch);
	PRINTF("B3470 offset %d",flash_get_para(ch));
	return adc_value_conv_temperature(adc_value_filter(ch))+flash_get_para(ch);
}

/*********************************************************************************************************
  END FILE 
*********************************************************************************************************/
