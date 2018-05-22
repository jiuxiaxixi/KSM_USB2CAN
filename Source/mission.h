#ifndef __MISSION_H
#define __MISSION_H	 

/**************
	1.34A 修改自动关机再振摇之后修改电机速度
	1.35A 调整振摇速度
	1.36A 更改关机时直接关闭电机
	1.37A	加入电机维护命令
	1.38A 降低振摇速度 
	1.39A 更改关机前复位
	1.40A 修复走位bug
	1.41A	增加可变帧缓存
	1.42A 增加在收到电机相关命令时设置超时时间，开机命令时重置timer
				增加显示屏失败测试连接
	1.43A 解决电源管理的兼容问题			
	1.44A 增加步进电机底层驱动最小步数，增加复位传感器检测功能,
	关机时不允许对电机进行操作
	1.45A 增加电机失步判断，再失步时以最小速度前进直到超时
	1.46B	更改为关机时重置时间，更改二维码扫码次数和时间为2次，750ms
	    ，更改电机失步速度，提高走位准确程度
	1.47T 电机加速度改为500，减速度300 测试
	1.47S 制冷上下限温度设置为7.5 和4.0，电机加速度为500，减速度300
	1.48B //cooler_pwm_off(); 试剂仓内风扇，水冷泵，水冷泵风扇常开 温度达到要求后只关闭制冷片
	1.49B 1优化扫码提前返回标志的情况；2温度跳变处理，使其0.2变化
	1.50B cooler_pwm_off();cooler_off; 水冷泵，水冷泵风扇常开 温度达到要求后只关闭制冷片和试剂仓内风扇
	1.51B 增加10 18和10 19两个命令号，用于开关试剂盘旋转工作状态指示灯
	1.52B 优化温度显示跳变的情况；收到关机命令时初始化系统参数，防止溢出；温度降到下限后关闭制冷片，延时60S关闭内部风扇
	1.52S 加速度从500改为350
	1.53B 修复电机走位精度偏差 
	1.54B 修复电源管理关电后温度不再刷新的情况；增加命令10 1A 复位温度显示屏幕初始界面。
	1.61B 增加 B3470温度传感器驱动 增加串口调试功能
	1.61B 2018/5/20 增加温度传感器补偿调试功能
*////////////////

#include "temp_control.h"	
#include "one_dimension_code.h"
#include "can.h"
#include "MicroStepDriver.h" 
#include "MSD_test.h" 
#include "coder.h"
#include "main.h"
#include "usart_screen.h"
#include "stmflash.h"
#include "notification.h"
#include "cooler.h"
#include "one_dimension_code.h"
extern 	u32 usb_waittime;
void mission_polling(void);


#endif
