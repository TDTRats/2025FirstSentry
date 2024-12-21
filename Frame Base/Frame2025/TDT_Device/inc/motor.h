/*****************************************************************************
File name: TDT_Device\inc\motor.h
Author: 郑俊元
Version: 1.3.1.191119_alpha
Date: 19.10.15
History: 
	——————————————————————————————————————————————————————————————————————————
	参考Readme.md
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "board.h"
#include "pid.h"
#include "can_calculate.h"
#include <stdint.h>

/**
 * @addtogroup TDT_Motor
 * @{
 */

///电机can口枚举定义，用于motorList指定索引
enum Can_x
{
	Can_1 = 0, ///<CAN1
	Can_2 = 1  ///<CAN2
};

///电机类型-带G的为云台电机，用于CanInfo根据电机类型进行解析
enum MotorType
{
	M2006,
	M3508,
	GM6020,
	MF9025,
	MT7015,
	MG6010,
	MG8016
};


/**
* @details  电机构造器，主要根据不同的pid填充canResult 包含了一些功能 以及脱力情况
            需要填充private中的指针来当作数据接口
 */
class Virtual_Motor
{
public:
	Virtual_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID);
	///Pid内环
	Pid pidInner;
	///Pid外环
	Pid pidOuter;

	CanInfo canInfo;
	CanInfo mycoder;    //放入一个供编码器使用的数据接口

	uint8_t updateOfflineFlag;
	uint16_t updateOfflineCnt;
	
	///对电机进行初始化,包括CAN使能
	void motorInit(void);

	///控制电流
	float ctrlCurrent(float current); //考虑 oneDeforce deforceFlag  justRead  offlineFlag updateOfflineFlag;

	///控制速度
	float ctrlSpeed(float speed, int8_t planIndex=0);

	///控制位置
	float ctrlPosition(double position, int8_t planIndex = 0);

	///设置功率限幅指针
	void setPowerOutLimit(float *Kp);

	///设置电机机械零点值
	void setZeroValue(uint16_t offset);
	
	float canResult;
	
	//	///获取电机类型
	MotorType getType(void);

	///获取can口
	Can_x getCan_x(void);

	///获取电机can_id
	uint32_t getStd_Id(void);

	///获取获取电机是否使能
	uint8_t getEnableMotor(void);

	///获取该对象对应电机的电流最大值
	uint16_t getMotorCurrentLimit(void);

	///获取指定电机类型的电流最大值
	static uint16_t getMotorCurrentLimit(MotorType motorType);

	///获取该对象对应电机的速度最大值
	uint16_t getMotorSpeedLimit(void);

	///获取指定电机类型的速度最大值
	static uint16_t getMotorSpeedLimit(MotorType motorType);

	///获取该对象对应电机的温度最大值
	uint16_t getMotorMaxTemp(void);

	///获取指定电机类型的温度最大值
	static uint16_t getMotorMaxTemp(MotorType motorType);
	
	//电机失能（不发不收）
	void setMotorDisable();
	
	//只读电机
	void setMotorJustRead();
	
	//脱力单电机
	void setOneDeforceFlag(bool ifdeforce);
	
	struct MotorInfo
	{
		MotorType type;	 ///<电机类型
		Can_x can_x;	 ///<挂载CAN总线[CAN_1 / CAN_2]
		uint32_t std_ID; ///<电机CAN总线反馈StdID
	} motorInfo;
	
protected:
		//注意使用的两种情况
	void sendThisCanMsg(uint32_t id,uint8_t* canbuff=nullptr);
	
  void (*Send_Current_Agreement)(uint8_t[8], float);

	
	struct
	{
		u8 pidOuterCnt;	   ///<外环计次-用于外环
		u8 posOffSetFlag;  ///<位置校零标志位
		double offSetPos;  ///<位置校零设定值变量
		float overHeatKp;  ///<电机过热保护系数，作用于内环输出
		float *powerOutKp; ///<功率控制系数，作用于内环输出 , 功率输出限幅系数
		//以下变量初始化函数中进行赋默认值
		float currentLimit; ///<电流限制
		float speedLimit;	///<速度限制
		float offSetLimit;	///<位置校零输出限幅系数
		float tempLimit;	///<温度限制
		float criticalTemp; ///<临界温度
		float maxOverTemp;	///<最大超出温度
		float degree; //编码器精度
	} otherInfo;
	
	///功能使能位
	struct
	{
		u8 disable ;         //既不发也不收
		u8 oneDeforce;        //单电机脱力
		u8 justRead;				// 只读不发
		u8 overTempProtect; ///<过热保护
		u8 powerLimit;		///<使用功率系数powerKp
	} enableFlag;
	
	///对温度进行限幅计算
	void overHeatProtect(int16_t temp);
};

class Motor:public Virtual_Motor
{
private:	
	static bool IfContainSend(int can_i, int idIndex);
	static void updateMotorStatus(Motor* motor, int can_i, int idIndex, int idOffset);
	static void sendCanMessage(int can_i, int idIndex);
public:
	///创建电机对象，并进行初始化（能进行数据接收）
	Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID);

	///定时器回调函数，can发送
	static void DjisendCanMsg();

	static float canBuff[2][12]; ///<电机CAN信息缓冲区
	
	bool torque_control;  //慎用 首先只有6020可以选择是否开启电流控制，
//	其次开启之后这一组的控制帧都会改变，要求这一组只有6020电机才可以
};
///结构体：电机列表，用于通过CAN口和ID检索（轮询）所有电机
typedef struct
{
	Motor *motorPoint;	  ///<电机对象指针
} MotorList;

extern MotorList motorList[2][12];



#endif
