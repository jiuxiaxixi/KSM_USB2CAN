/****************************************Copyright (c)****************************************************
**                                	重庆科斯迈生物科技有限公司
**                                    6500 试剂系统                    
**																			@张校源
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:               stmflash.c
** Last modified Date:      2018-05-19
** Last Version:            v1.1
** Description:             热敏电阻NTC 温度传感器 B3470
** 
**--------------------------------------------------------------------------------------------------------
** Created By:              张校源
** Created date:            2017-05-16
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             张校源
** Modified date:           2018-05-18
** Version:                 v1.1
** Description:             增加温度校准参数
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

#include "stmflash.h"
#include "can.h"
#include "MSD_test.h"  
/*********************************************************************************************************
 FLASH 参数全局变量
*********************************************************************************************************/
flash_mission_t fmt; //STM32内部FLASH读写 驱动代码	   
 
/*********************************************************************************************************
** Function name:       STMFLASH_ReadWord
** Descriptions:        读取指定地址的半字(16位数据) 
** input parameters:    faddr:读地址 
** output parameters:   0
** Returned value:      对应数据.
** Created by:          张校源
** Created Date:        2017-05-01
*********************************************************************************************************/
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  
/*********************************************************************************************************
** Function name:       STMFLASH_GetFlashSector
** Descriptions:        获取某个地址所在的flash扇区
** input parameters:    flash地址
** output parameters:   0
** Returned value:      0~11,即addr所在的扇区
** Created by:          张校源
** Created Date:        2017-05-01
*********************************************************************************************************/
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}

/*********************************************************************************************************
** Function name:       STMFLASH_Read
** Descriptions:        从指定地址开始写入指定长度的数据
** 特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
**         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
**         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
**         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
** 				 该函数对OTP区域也有效!可以用来写OTP区!
** 				 OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
** input parameters:    WriteAddr:起始地址(此地址必须为4的倍数!!) 
**											pBuffer:数据指针 
**											NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2017-05-01
*********************************************************************************************************/
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
 // FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
//  FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 


/*********************************************************************************************************
** Function name:       STMFLASH_Read
** Descriptions:        从指定地址开始读出指定长度的数据
** input parameters:    ReadAddr:起始地址 pBuffer:数据指针 NumToRead:字(4位)数
** output parameters:   pBuffer:数据指针
** Returned value:      0
** Created by:          张校源
** Created Date:        2017-05-01
*********************************************************************************************************/
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}


/*********************************************************************************************************
** Function name:       flash_mission_polling
** Descriptions:        flash 参数任务
** input parameters:    mission.c 中接受到can 的指令
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2017-05-01
*********************************************************************************************************/
void flash_mission_polling(){
	static u8 flash_u8_buf[FLASH_PARA_LEN_8];
	static u8	upload[2];
	switch(fmt.mission_state){
	
	case FLASH_IDLE:
		if(fmt.newdata){
			offset_calc();
			fmt.newdata=0;
		}
		break;
	
	case FLASH_WRITE:    //写入内存
		
		STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)fmt.buf,FLASH_PARA_LEN_32);
		fmt.mission_state=FLASH_WRITE_SUCCEE;
	
	break;
	
	case FLASH_READ:   //读取内存
		stmflash_read_reverse(flash_u8_buf,FLASH_PARA_LEN_32);
		fmt.mission_state=FLASH_READ_SUCCEE;
	break;
	
	case FLASH_WRITE_SUCCEE:  //写入成功
			
	
		mission_success_send(fmt.current_mission);
		fmt.newdata=1;
		fmt.mission_state=FLASH_IDLE;
	
		break;
	
	case FLASH_READ_SUCCEE:
		//MSB AND LSB problem
		if(fmt.command_index*2+1 > FLASH_PARA_LEN_8)
			break;
		upload[0]=flash_u8_buf[fmt.command_index*2];
		upload[1]=flash_u8_buf[fmt.command_index*2+1];
		
		action_value_send(upload,2,fmt.current_mission);
	
		fmt.mission_state=FLASH_IDLE;
	
		break;
	
	case MEMORY_WIRTE_SUCCEE:
		mission_success_send(fmt.current_mission);
		fmt.newdata=1;
		fmt.mission_state=FLASH_IDLE;
		break;
	
	case FLASH_FAILE:
		break;
	}
}

/*********************************************************************************************************
** Function name:       flash_init
** Descriptions:        flash 参数初始化 从FLASH_SAVE_ADDR读取数据到 全局变量
** input parameters:    0
** output parameters:   fmt.buf
** Returned value:      0
** Created by:          张校源
** Created Date:        2017-05-01
*********************************************************************************************************/
void flash_init(){
	STMFLASH_Read(FLASH_SAVE_ADDR,(u32 *)fmt.buf,FLASH_PARA_LEN_32);
	fmt.newdata=1;
}

void stmflash_read_reverse(u8 *pBuffer,u32 NumToRead){
	u8 temp[256];
	STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)temp,NumToRead);
	for(u8 i=0;i<NumToRead*4;i+=2){
		*(pBuffer+i)=temp[i+1];
		*(pBuffer+i+1)=temp[i];
	}
}
/*********************************************************************************************************
** Function name:       offset_calc
** Descriptions:        偏移量计算  偏移值 = 理论值 - 实际值
**											motor.offset 计算出来的偏移值
**											fmt.buf			 FLASH写入的实际值
**											angle				 理论值
** input parameters:    0
** output parameters:   fmt.buf
** Returned value:      0
** Created by:          张校源
** Created Date:        2017-05-01
*********************************************************************************************************/
void offset_calc(void){			
	motor.offset[1]=fmt.buf[1];
	motor.offset[2]=fmt.buf[2]-angle(1,1,1);
	motor.offset[3]=fmt.buf[3]-angle(1,1,2);
	motor.offset[4]=fmt.buf[4]-angle(1,1,3);
	motor.offset[5]=fmt.buf[5]-angle(2,1,1);
	motor.offset[6]=fmt.buf[6]-angle(2,1,2);
	motor.offset[7]=fmt.buf[7]-angle(2,1,3);
	motor.offset[8]=fmt.buf[8]-angle(3,1,1);
	motor.offset[9]=fmt.buf[9]-angle(3,1,2);
	motor.offset[10]=fmt.buf[10]-angle(3,1,3);
	//PRINTF("fmt.temperature_offset_c2 %d fmt.temperature_offset_c3 %d\r\n",fmt.temperature_offset_c2,fmt.temperature_offset_c3);
}

/*********************************************************************************************************
** Function name:       position_init_to_flash
** Descriptions:        把电机要走的脉冲数写入内存 测试用
** input parameters:    0
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2017-05-01
*********************************************************************************************************/
void position_init_to_flash(void)
{
	fmt.buf[1]=0;
	fmt.buf[2]=step_to_move_calc(1,1,1);
	fmt.buf[3]=step_to_move_calc(1,1,2);
	fmt.buf[4]=step_to_move_calc(1,1,3);
	fmt.buf[5]=step_to_move_calc(2,20,1);
	fmt.buf[6]=step_to_move_calc(2,21,2);
	fmt.buf[7]=step_to_move_calc(2,24,3);
	fmt.buf[8]=step_to_move_calc(3,1,1);
	fmt.buf[9]=step_to_move_calc(3,1,2);
	fmt.buf[10]=step_to_move_calc(3,1,3);
	fmt.current_mission=FLASH_MISSION_WRITE;
	fmt.mission_state=FLASH_WRITE;
	fmt.newdata=1;
}

/*********************************************************************************************************
** Function name:       flash_get_para
** Descriptions:        获取对应设备的偏移参数
** input parameters:    B3470_C2 B3470_C3
** output parameters:   0
** Returned value:      0
** Created by:          张校源
** Created Date:        2018-05-20
*********************************************************************************************************/
int16_t flash_get_para(uint8_t device)
{
	switch(device)
	{
		case B3470_C2:
			return fmt.buf[FLASH_TEMP1_OFFSET_POS];
		case B3470_C3:
			return fmt.buf[FLASH_TEMP2_OFFSET_POS];
		case FLASH_C3_ZL_HIGH:
			
			if(fmt.buf[FLASH_C3_ZL_HIGH] == -1)
				return 75;
			else
				return fmt.buf[FLASH_C3_ZL_HIGH];
			
		case FLASH_C3_ZL_LOW:
			
			if(fmt.buf[FLASH_C3_ZL_LOW] == -1)
				return 45;
			else
				return fmt.buf[FLASH_C3_ZL_LOW];
			
		case FLASH_C2_ZL_HIGH:
			
			if(fmt.buf[FLASH_C2_ZL_HIGH] == -1)
				return 100;
			else
				return fmt.buf[FLASH_C2_ZL_HIGH];		
			
		case FLASH_C2_ZL_LOW:
			
			if(fmt.buf[FLASH_C2_ZL_LOW] == -1)
				return -20;
			else
				return fmt.buf[FLASH_C2_ZL_LOW];		
			
		case FLASH_C2_STOP_TIME:
			
			if(fmt.buf[FLASH_C2_STOP_TIME] == -1)
				return 60;
			else
				return fmt.buf[FLASH_C2_STOP_TIME];	
		
		case FLASH_LM35_ZL_HIGH:
			if(fmt.buf[FLASH_LM35_ZL_HIGH] == -1)
				return 70;
			else
				return fmt.buf[FLASH_LM35_ZL_HIGH];	
			
		case FLASH_LM35_ZL_LOW:
			if(fmt.buf[FLASH_LM35_ZL_LOW] == -1)
				return 45;
			else
				return fmt.buf[FLASH_LM35_ZL_LOW];			

		case FLASH_LM35_ON_TIME:
			if(fmt.buf[FLASH_LM35_ON_TIME] == -1)
				return (30*60);
			else
				return fmt.buf[FLASH_LM35_ON_TIME];	
			
		case FLASH_LM35_OFF_TIME:
			if(fmt.buf[FLASH_LM35_OFF_TIME] == -1)
				return (60);
			else
				return fmt.buf[FLASH_LM35_OFF_TIME];					
		
		case FLASH_LM35_DIS_OFFSET:
			if(fmt.buf[FLASH_LM35_DIS_OFFSET] == -1)
				return -40;
			else
				return fmt.buf[FLASH_LM35_DIS_OFFSET];	
		default:
			PRINTF("fmt got no para of %d",device);
			return 0;
	}
}




