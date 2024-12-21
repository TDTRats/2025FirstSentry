#ifndef _LK_MOTOR_
#define _LK_MOTOR_
#include "board.h"
#include "motor.h"
#include "can_calculate.h"
#include "can.h"
#define Torque_constant 0.32   //扭矩常量

class LK_Motor_Rec :public Virtual_Motor_Calculate
{
	private:
	int64_t originAngle : 56; //只用作查看多圈角度原始值用(一般电机不用)
	public:
	void GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg) override;
};

class LK_Motor:public Virtual_Motor
{
public:
	bool use_my_send;
	LK_Motor_Rec  lk_Motor_Rec;
	LK_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t control_id,uint32_t _Std_ID);
	
	static void SendCurrentAgreement(uint8_t canbuff[8],float canResult) ;

	void Motor_Send();

	void Motor_Receive(u8 can_x,CanRxMsg *_CanRxMsg, u8 ifsetzero);//接收函数

//以上为基本电机配置，下面为特殊电机的单独配置
	
		void motorRead_totalAngle(void);  //读电机多圈编码器
		void clearErrorFlag(void);//清除异常标志
		void motorClose(void);//电机关闭命令
		void motorStop(void);//电机停止命令
		void motorRun(void);//电机运行命令               //这些命令调用则立刻执行
	
	
		//只有在定义了use_my_send=1时才可以使用一下函数，否则无效
		void Auto_ctrlSpeed(float ctrlspeed);//电机速度控制
		//注意，以下的最大速度不能为0；
		void Auto_ctrlPosition(float ctrlposition,float maxspeed=1000);  //多圈位置控制 ,需要搭配motorRead_totalAngle 读电机内置的角度使用
			
	
	private:
	uint32_t control_id;
	u8 canTxBuff[8];
	u8 readcnt;
	
};

void LKList_Send();
void LKList_Receive(u8 can_x,CanRxMsg *_CanRxMsg);


#endif