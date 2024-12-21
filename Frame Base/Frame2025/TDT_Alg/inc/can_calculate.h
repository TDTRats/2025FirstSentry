/******************************
File name: TDT_Alg\src\can_calculation.cpp
Description: PID算法
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加对云台电机和底盘电机的区分，增加电机离线检测
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __CAN_CALCULATE_H__
#define __CAN_CALCULATE_H__

#include "board.h"
#pragma pack(1)		
typedef struct
{
	uint16_t encoder;				///<原始机械编码器值
	int16_t speed;					///<原始机械转速
	int32_t trueCurrent;			//实际电流
	int16_t temperature;			//电机温度
	
	float Auto_totalAngle;          //部分电机有(大疆电机无)，反馈电机发的多圈角度
	
	float dps;						///<转速（度每秒），与mpu6050单位相等
	int32_t totalAngle;				///<总角度
	float totalAngle_f;				///<总角度（浮点），与mpu6050单位相等
	u8 lostFlag;					//掉线标志位
	int32_t totalEncoder_SI;		//速度积分出来的位置
	int16_t offsetEncoder;			//机械角度初始值
	int32_t totalRound;				///<总圈数
	int32_t totalEncoder;			///<总角度
	
	
	int16_t lastEncoderCalibration; ///<上次减去机械角度初值后的角度
	int16_t encoderCalibration;		///<减去机械角度初值后的角度
	int32_t lastEncoder;			//上次总角度值
	int32_t lastTotalEncoder;		//上次总机械角度值
	uint16_t msgCnt;				// 接受消息计数
	uint16_t lostCnt;				// 接受消息计数

} CanInfo;

#pragma pack()	


class Virtual_Motor_Calculate
{
protected:
	CanInfo *Info;

	u8 speedUnit=1;

	void Motor_Offset(CanInfo *Info, u8 ifsetzero);

	void rec_Calculate(CanInfo *Info,uint16_t degree=8192); //消息来源和输出，编码器精度
public:
	
	void motor_Rec_Calculate(CanInfo *Info,CanRxMsg *_CanRxMsg, uint16_t degree=8192, u8 ifsetzero=0);

	virtual void GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg)=0;  //接收信息重写 必须要填写的是encoder和speed

};

/**
  * @brief 提供dji电机can数据的解析和信息
  * @details can数据的解析，输出到电机对象的数据对象
  * @note 通过 motorPointList 指针列表自动输出到对应电机对象
  * @sa motorList[2][8]
  */
class DJI_Motor :public Virtual_Motor_Calculate
{
private:
public:
	void Dji_Motor_Calculate(u8 can_x, CanRxMsg *_CanRxMsg);

	static uint8_t IsDJIMotorCheck(CanRxMsg _CanRxMsg);

	void GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg) override;
};


/** @} */

#endif
