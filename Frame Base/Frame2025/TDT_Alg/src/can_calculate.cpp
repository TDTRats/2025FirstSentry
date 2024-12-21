/******************************
File name: TDT_Alg\src\can_calculation.cpp
Description: PID算法
function:
	——————————————————————————————————————————————————————————————————————————
	void Can::Motor_Offset(CAN_TypeDef* CANx, CanRxMsg* _CanRxMsg)
	——————————————————————————————————————————————————————————————————————————
	void  Can::Motor_Information_Calculate(CAN_TypeDef* CANx, CanRxMsg* _CanRxMsg)
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "can_calculate.h"
#include "motor.h"

/**
 * @ingroup TDT_ALG
 * @defgroup TDT_CAN_CALC CAN信息处理
 * @brief CAN信息处理，主要针对大疆电机的信息处理
 */
 
 void Virtual_Motor_Calculate::rec_Calculate(CanInfo *Info,uint16_t degree)
{
	/*计算校准后的机械角度*/
	Info->encoderCalibration = Info->encoder - Info->offsetEncoder;
	if (Info->encoderCalibration > degree/2)
	{
		Info->encoderCalibration -= degree;
	}
	else if (Info->encoderCalibration < -degree/2)
	{
		Info->encoderCalibration +=degree;
	}

	/*计算电机旋转总圈数*/
	if (Info->encoderCalibration - Info->lastEncoderCalibration > degree/2)
	{
		Info->totalRound--;
	}
	else if (Info->encoderCalibration - Info->lastEncoderCalibration < -degree/2)
	{
		Info->totalRound++;
	}
	/*计算电机旋转总机械角度值*/
	Info->totalEncoder = (Info->totalRound * degree + Info->encoderCalibration);
	/*计算电机旋转总角度*/
	Info->totalAngle = Info->totalEncoder * 360.0f/degree;	 //(强制转换成整型)
	Info->totalAngle_f = Info->totalEncoder * 360.0f/degree; //(浮点型)

	Info->dps = Info->speed * 6.0f*speedUnit; //(浮点型)

	Info->totalEncoder_SI += Info->speed;
	//速度积分和位置反馈做互补滤波！！！
	/*记录此次机械角度*/
	Info->lastEncoder = Info->encoder;
	Info->lastTotalEncoder = Info->totalEncoder;
	Info->lastEncoderCalibration = Info->encoderCalibration;

	/*电机离线检测部分*/
	Info->lostCnt = 0;	//清空计数器
	Info->lostFlag = 0; //电机在线
}
/**
  * @brief DJI电机上电初始化
  * @param[in] can_x CAN1/CAN2
  * @param[in] _CanRxMsg Can原始数据地址
  */
void Virtual_Motor_Calculate::Motor_Offset(CanInfo *Info, u8 ifsetzero )
{
	//非云台电机才认为上电点为零点
	if ( ifsetzero == 1)
	{
		Info->offsetEncoder = Info->encoder;
	}
	Info->lastEncoder = Info->encoder;											 //解决上电圈数不对问题
	Info->totalRound = 0;														 //电机角度归零时用
	Info->totalAngle = 0;
	Info->totalEncoder_SI = 0;
	Info->totalEncoder = 0;
}

/**
  * @brief DJI电机can返回信息计算
  * @param[in] can_x CAN1/CAN2
  * @param[in] _CanRxMsg Can原始数据地址
  */
void  Virtual_Motor_Calculate::motor_Rec_Calculate(CanInfo *Info,CanRxMsg *_CanRxMsg, uint16_t degree, u8 ifsetzero)
{
	GetOriginData(Info,_CanRxMsg);
	
	//初始化校准
	if (++Info->msgCnt < 3)
	{
		Motor_Offset(Info, ifsetzero);
		return;
	}
	Info->msgCnt = 100;
	
	
	rec_Calculate(Info,degree);

}


void DJI_Motor::GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg)
{
	Info->encoder = (int16_t)((_CanRxMsg->Data[0] << 8) | (_CanRxMsg->Data[1]));	 //机械角度
	Info->speed = (int16_t)((_CanRxMsg->Data[2] << 8) | (_CanRxMsg->Data[3]));		 //速度
	Info->trueCurrent = (int16_t)((_CanRxMsg->Data[4] << 8) | (_CanRxMsg->Data[5])); //实际电流
	Info->temperature = (int16_t)(_CanRxMsg->Data[6]);		
}


void DJI_Motor::Dji_Motor_Calculate(u8 can_x, CanRxMsg *_CanRxMsg)
{
	//不是大疆标准电机，直接退出
	if (IsDJIMotorCheck(*_CanRxMsg) == 0)
	{
		return;
	}
	//如果该can口的电机没有使能，则报error并退出
	if (motorList[can_x][_CanRxMsg->StdId - 0x201].motorPoint->getEnableMotor() == 0)
	{
		return;
	}
	
	Info = &(motorList[can_x][_CanRxMsg->StdId - 0x201].motorPoint->canInfo);
	/*获取电机转速及机械角度*/
	MotorType motorType = motorList[can_x][_CanRxMsg->StdId - 0x201].motorPoint->getType();
	
	motor_Rec_Calculate(Info,_CanRxMsg,8192, motorType!= GM6020);
}

/**
  * @brief 大疆电机检查
  * @param[in] _CanRxMsg 通过判断ID来确定是不是大疆电机
  * @return 1:是，0：不是
  */
uint8_t DJI_Motor::IsDJIMotorCheck(CanRxMsg _CanRxMsg)
{
	if (_CanRxMsg.StdId > 0x200 && _CanRxMsg.StdId < 0x20C)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}







/**
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加对云台电机和底盘电机的区分，增加电机离线检测
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
	24.9.13  codeartist 重构 提供其他电机的接口
	——————————————————————————————————————————————————————————————————————————
*/