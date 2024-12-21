/******************************
File name: TDT_Device\src\can_task.cpp
Description: 电机数据的处理
Class:
	——————————————————————————————————————————————————————————————————————————
	Virtual_Motor 虚电机构造器，包含了电机的所有算法，功能标志位，提供了外部电机发送接口
	——————————————————————————————————————————————————————————————————————————
	功能位使用：（需要在初始化时使用，禁止运行中赋值）
	——————————————————————————————————————————————————————————————————————————
	void setMotorDisable();//电机失能（不发不收）
	——————————————————————————————————————————————————————————————————————————
	void setMotorJustRead();//只读电机
	——————————————————————————————————————————————————————————————————————————
	void setPowerOutLimit(float *Kp);///设置功率限幅指针
	——————————————————————————————————————————————————————————————————————————
	void setOneDeforceFlag(bool ifdeforce) //脱力单电机(可以在运行时使用）
	——————————————————————————————————————————————————————————————————————————
	（过热保护自动开，根据电机列表的最大温度选择开不开）
	——————————————————————————————————————————————————————————————————————————
	设置方法：
	——————————————————————————————————————————————————————————————————————————
	void setZeroValue(uint16_t offset);
	——————————————————————————————————————————————————————————————————————————
	子类发送函数接口：
	——————————————————————————————————————————————————————————————————————————
	void sendThisCanMsg(uint32_t id,uint8_t* canbuff); 调用此函数发送 默认不发canbuff，若enableFlag.use_my_send==1 则默认发canbuff
	——————————————————————————————————————————————————————————————————————————
	virtual void Send_Current_Agreement(float *canbuff,float canResult) 需要调用此函数重写发送电流协议
	——————————————————————————————————————————————————————————————————————————
	计算输出函数：
	——————————————————————————————————————————————————————————————————————————
	float ctrlCurrent(float current);
	——————————————————————————————————————————————————————————————————————————
	float ctrlSpeed(float speed, int8_t planIndex=0);
	——————————————————————————————————————————————————————————————————————————
	float ctrlPosition(double position, int8_t planIndex = 0);
	——————————————————————————————————————————————————————————————————————————
	初始化流程：
	——————————————————————————————————————————————————————————————————————————
	diaMotor = new Motor(M2006, CAN2, 0x203);
	diaMotorInner.kp = 3;
	.......
	diaMotor->pidInner.paramPtr = &diaMotorInner;
	diaMotor->pidInner.fbValuePtr[0]=&diaMotor->canInfo.speed;
	diaMotorOuter.kp = 2.5;
	.......
	diaMotor->pidOuter.paramPtr = &diaMotorOuter;
	diaMotor->pidOuter.fbValuePtr[0] =&diaMotor->canInfo.totalAngle_f;
	调用电机设置函数和电机功能函数
	
	非大疆电机需要重写发送函数并调用，重写接收函数并调用
	——————————————————————————————————————————————————————————————————————————
	可观测参数：Pid  canInfo updateOfflineFlag enableFlag
	——————————————————————————————————————————————————————————————————————————
class Motor
	——————————————————————————————————————————————————————————————————————————
	大疆电机是联合发送，所以有单独的发送函数和接收函数
	——————————————————————————————————————————————————————————————————————————
	使用电机列表的方式，自动接收和发送，注意联合发送所带来的问题
	——————————————————————————————————————————————————————————————————————————
	torque_control 力矩控制，慎用，详情见.h
	
	
*****************************/
#include "motor.h"
#include "can.h"
#include "dbus.h"



///电机电流输出限制表
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,GM6020,MF9025(小匝数),MT7015,MG6010,MG8016
uint16_t motorCurrentLimitList[7] = {10000, 16384, 30000,2048,2000,2048,2048};

//电机速度输出限制表-取决于各类电机的最高转速
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,GM6020,MF9025,MT7015,MG6010,MG8016
uint16_t motorSpeedLimitList[7] = {9500, 9158, 320,4260,3480,1506,1548};

//电机绕组最高允许温度列表
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,GM6020,MF9025,MT7015,MG6010,MG8016
uint16_t motorMaxTempList[7] = {0, 125, 125,125,125,125,125};

//电机编码器精度对应列表
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,GM6020,MF9025,MT7015,MG6010,MG8016
uint16_t motordegreeList[7] = {8192, 8192,8192,65535,16383,65535,65535};


MotorList motorList[2][12]={0};
float Motor::canBuff[2][12]={0};
extern uint8_t deforceFlag;
/** @} */



/**
 * @param  motorType        电机类型: 用以区别不同类型的电机
 * @param  _Canx             CAN口
 * @param  _Std_ID           CAN标准标识符
 * @sa motorInit
 */
Virtual_Motor::Virtual_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID) : pidInner(Pid(1)), pidOuter(Pid(1))
{
	//电机基本信息填充
	motorInfo.type = motorType;
	motorInfo.std_ID = _Std_ID;
	motorInfo.can_x = _Canx == CAN1 ? Can_1 : Can_2;


	//电机额外信息填充
	otherInfo.currentLimit = motorCurrentLimitList[motorType]; //输出限幅（内环输出限幅）
	otherInfo.speedLimit = motorSpeedLimitList[motorType];	   //速度限幅（外环输出限幅）
	otherInfo.tempLimit = motorMaxTempList[motorType];		   //绕组最高允许温度
	otherInfo.offSetLimit = otherInfo.currentLimit / 3;		   //默认位置校零时以三分之一输出开始
	otherInfo.criticalTemp = 0.7f * otherInfo.tempLimit;	   //过热保护临界温度
	otherInfo.maxOverTemp = 0.8f * otherInfo.tempLimit;		   //过热保护截止温度，高于此温度电机输出为0
	otherInfo.degree=motordegreeList[motorType];
	//对于有最高绕组温度的电机开启过热保护
	if (motorMaxTempList[motorType] != 0)
	{
		this->enableFlag.overTempProtect = 1; //过热保护使能
	}
		//pid默认反馈值填充
	pidInner.fbValuePtr[0] = &canInfo.speed;
	pidOuter.fbValuePtr[0] = &canInfo.totalEncoder;
	
	//进行初始化
	motorInit();
	
	
}

void Virtual_Motor::motorInit(void)
{
	canInfo.lostFlag=1;
  updateOfflineFlag=1;
	canInfo.offsetEncoder = 0;
	
}

/**
 * @details 对输入的电流进行限幅，包括温度限幅、功率限幅、最大值限幅；并将电流值填入canBuff的缓存区中（可选）
 * @param  current          电流值
 * @param  sendFlag         是否将限幅后的输出值填入canBuff对应的缓存区中
 * @return float 经过温度限幅、功率限幅、最大值限幅的电流值
 */
float Virtual_Motor::ctrlCurrent(float current)
{
	updateOfflineFlag=0;
	updateOfflineCnt=0;
	if (enableFlag.disable||canInfo.lostFlag || deforceFlag || updateOfflineFlag ||enableFlag.justRead ||enableFlag.oneDeforce ) 
	{
		canResult=0;
		return 0;
	}

	current = LIMIT(current, -motorCurrentLimitList[motorInfo.type], motorCurrentLimitList[motorInfo.type]);
	//过热保护
	if (enableFlag.overTempProtect == 1)
	{
		this->overHeatProtect(canInfo.temperature);
		current *= otherInfo.overHeatKp;
	}
	
	// 功率输出限幅系数
	if (enableFlag.powerLimit == 1)
	{
		current *= *otherInfo.powerOutKp;
	}
	canResult=current; 
	return current;
}

/**
 * @details 将设定的速度值通过pid计算后，将pid输出值传入ctrlCurrent
 * @param  speed            速度设定值
 * @param  planIndex        方案号
 * @param  sendFlag         是否自动调用发送函数（传入sendFlag中）
 * @return float 经过内环pid控制，以及温度限幅、功率限幅、最大值限幅的 \b 电流值
 */
float Virtual_Motor::ctrlSpeed(float speed, int8_t planIndex)
{
	if (enableFlag.disable||canInfo.lostFlag || deforceFlag || updateOfflineFlag ||enableFlag.justRead ||enableFlag.oneDeforce ) 
	{
		pidInner.Clear();
		return ctrlCurrent(0);
	}
	/*PID计算，默认输出到对象的result变量*/
	pidInner.Calculate(speed, planIndex);
	
	return ctrlCurrent((pidInner.result));
}

/**
 * @details 将设定的位置通过pid计算后，将pid输出值传入ctrlSpeed
 * @param  position         位置设定值
 * @param  planIndex        方案号
 * @param  sendFlag         是否自动调用发送函数
 * @return float 经过双环pid控制，以及温度限幅、功率限幅、最大值限幅的电流值
 */
float Virtual_Motor::ctrlPosition(double position, int8_t planIndex)
{
	/*内环*/
	if (enableFlag.disable||canInfo.lostFlag || deforceFlag || updateOfflineFlag ||enableFlag.justRead ||enableFlag.oneDeforce ) 
	{
		pidOuter.Clear();
		return ctrlSpeed(0,planIndex);
	}
	/*外环*/
	if (++otherInfo.pidOuterCnt >= 2)
	{
		otherInfo.pidOuterCnt = 0;
		/*PID计算，默认输出到对象的result变量*/
		pidOuter.Calculate(position, planIndex);
	}
	return ctrlSpeed(pidOuter.result);
}

/**
 * @param  temp             当前温度
 */
void Virtual_Motor::overHeatProtect(int16_t temp)
{
	otherInfo.overHeatKp = (float)(1.0f - ((temp - otherInfo.criticalTemp) / (otherInfo.maxOverTemp - otherInfo.criticalTemp)));
	otherInfo.overHeatKp = LIMIT(otherInfo.overHeatKp, 0, 1);
}

/**
 * @details 调用后会在控制电流时将该指针的值乘到电流中
 * @warning - 不建议传入局部变量的地址。如果确实需要，请保证直接或间接调用ctrlCurrent的过程在该局部变量的生命周期中，否则请使用该函数更新地址
 * - 该函数不对传入的地址进行有效检查
 * @param  Kp               功率限幅指针
 */
void Virtual_Motor::setPowerOutLimit(float *Kp)
{
	enableFlag.powerLimit = 1;
	otherInfo.powerOutKp = Kp;
}

/**
 * @details 将零点值更改
 * @warning 该操作可能会导致圈数多1或少1，建议在初始化时使用。下版本可能会修复此问题
 * @todo 修复圈数可能多1或少1的问题（同步更改lastEncoderCalibration）
 * @param  offset           电机机械零点值
 */
void Virtual_Motor::setZeroValue(uint16_t offset)
{
	canInfo.offsetEncoder = offset;

	canInfo.encoderCalibration = canInfo.encoder - canInfo.offsetEncoder;
	if (canInfo.encoderCalibration > otherInfo.degree/2)
	{
		canInfo.encoderCalibration -= otherInfo.degree;
	}
	else if (canInfo.encoderCalibration < -otherInfo.degree/2)
	{
		canInfo.encoderCalibration += otherInfo.degree;
	}
	canInfo.encoderCalibration = canInfo.encoder - canInfo.offsetEncoder;
	if (canInfo.encoderCalibration > otherInfo.degree/2)
	{
		canInfo.encoderCalibration -= otherInfo.degree;
	}
	else if (canInfo.encoderCalibration < -otherInfo.degree/2)
	{
		canInfo.encoderCalibration += otherInfo.degree;
	}
}




///交换两个变量的值，仅限于整形相同变量类型
#define Change_Value(a, b) \
	a ^= b;                \
	b ^= a, a ^= b;


/**
* @param  state            是否失能电机
 */
void Virtual_Motor::setMotorDisable()
{
		enableFlag.disable  = 1;
}

/**
* @param  state            是否只读电机
 */
void Virtual_Motor::setMotorJustRead()
{
		enableFlag.justRead = 1;
}

/**
* @param  state            是否只读电机
 */
void Virtual_Motor::setOneDeforceFlag(bool ifdeforce)
{
		enableFlag.oneDeforce = ifdeforce;
}

/**
 * @return MotorType 电机类型
 */
MotorType Virtual_Motor::getType(void)
{
	return motorInfo.type;
}
/**
 * @return Can_x can口
 */
Can_x Virtual_Motor::getCan_x(void)
{
	return motorInfo.can_x;
}
/**
 * @return uint32_t 电机can_id
 */
uint32_t Virtual_Motor::getStd_Id(void)
{
	return motorInfo.std_ID;
}

/**
 * @return uint8_t 获取电机是否使能
 */
uint8_t Virtual_Motor::getEnableMotor(void)
{
	return (enableFlag.disable==0);
}

/**
 * @return uint16_t 该对象对应电机的电流最大值
 */
uint16_t Virtual_Motor::getMotorCurrentLimit(void)
{
	return motorCurrentLimitList[motorInfo.type];
}
/**
 * @param  motorType        指定电机类型
 * @return uint16_t 指定电机类型的电流最大值
 */
uint16_t Virtual_Motor::getMotorCurrentLimit(MotorType motorType)
{
	return motorCurrentLimitList[motorType];
}

/**
 * @return uint16_t 该对象对应电机的速度最大值
 */
uint16_t Virtual_Motor::getMotorSpeedLimit(void)
{
	return motorSpeedLimitList[motorInfo.type];
}
/**
 * @param  motorType        指定电机类型
 * @return uint16_t 指定电机类型的速度最大值
 */
uint16_t Virtual_Motor::getMotorSpeedLimit(MotorType motorType)
{
	return motorSpeedLimitList[motorType];
}

/**
 * @return uint16_t 该对象对应电机的温度最大值
 */
uint16_t Virtual_Motor::getMotorMaxTemp(void)
{
	return motorMaxTempList[motorInfo.type];
}
/**
 * @param  motorType        指定电机类型
 * @return uint16_t 指定电机类型的温度最大值
 */
uint16_t Virtual_Motor::getMotorMaxTemp(MotorType motorType)
{
	return motorMaxTempList[motorType];
}


void Virtual_Motor::sendThisCanMsg(uint32_t id,uint8_t* canbuff)
{
	if(enableFlag.disable==1)
		return;
	
	 // 检查是否超过100ms未刷新或脱力标志
    if (updateOfflineCnt ++ >=100 )
    {
			updateOfflineCnt=100;
      updateOfflineFlag = 1;
       
    }
		  // 检查电机是否离线
    if (canInfo.lostCnt++ >= 10)
    {
			canInfo.lostCnt=10;
      canInfo.lostFlag = 1;
    }
	
	if(enableFlag.oneDeforce || deforceFlag || enableFlag.justRead)
	{
		pidInner.fbValue=*pidInner.fbValuePtr[0];
		pidOuter.fbValue=*pidOuter.fbValuePtr[0];
	}
		
	if(enableFlag.justRead ==1)
		return;
	
	if(enableFlag.oneDeforce || deforceFlag || updateOfflineFlag ||  canInfo.lostFlag)
	{
		this->canResult=0;
	}
	uint8_t cantx[8];
	Send_Current_Agreement(cantx,this->canResult);
	
	if(canbuff==nullptr)
		canTx(cantx, motorInfo.can_x  == 0 ? CAN1 : CAN2, id); 
	else
		canTx(canbuff, motorInfo.can_x  == 0 ? CAN1 : CAN2, id);
}




Motor::Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID):Virtual_Motor(motorType, _Canx, _Std_ID)
{
		if (_Std_ID < 0x200 || _Std_ID > 0x20C)
	{
		enableFlag.disable=1;
		return;
	}
	
	if(motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].motorPoint!=nullptr &&
	motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].motorPoint!=this)
	{
		motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].motorPoint->enableFlag.disable=1;
		enableFlag.disable=1;
		return;
	}
	motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].motorPoint = this;
}

void Motor::DjisendCanMsg()
{
		for (int can_i = 0; can_i < 2; can_i++)
      for (int idIndex = 0; idIndex < 3; idIndex++)
				for (int idOffset = 0; idOffset < 4; idOffset++)
				{
					Motor* motor = motorList[can_i][idIndex * 4 + idOffset].motorPoint;
					if(motor==0 || motor->enableFlag.justRead)
					continue;
					updateMotorStatus(motor, can_i, idIndex, idOffset);		
				}
	
	
    for (int can_i = 0; can_i < 2; can_i++)
       for (int idIndex = 0; idIndex < 3; idIndex++)
       {
           if (IfContainSend(can_i, idIndex))
           {
              sendCanMessage(can_i, idIndex);
           }
       }
}

bool Motor::IfContainSend(int can_i, int idIndex)
{
    bool containMotor = false;

    for (int idOffset = 0; idOffset < 4; idOffset++)
    {
       Motor* motor = motorList[can_i][idIndex * 4 + idOffset].motorPoint;

        if (motor==0 || motor->enableFlag.disable || motor->enableFlag.justRead)
				{
					continue;
				}
            

        containMotor = true;
    }
    return containMotor;
}

void Motor::updateMotorStatus(Motor* motor, int can_i, int idIndex, int idOffset)
{
    // 检查是否超过100ms未刷新或脱力标志
    if (motor->updateOfflineCnt ++ >=100 )
    {
				motor->updateOfflineCnt=100;
        motor->updateOfflineFlag = 1;
       
    }
		  // 检查电机是否离线
    if (motor->canInfo.lostCnt++ >= 7)
    {
			motor->canInfo.lostCnt=7;
      motor->canInfo.lostFlag = 1;
    }
		
		if(motor->updateOfflineFlag || motor->canInfo.lostFlag ||motor->enableFlag.oneDeforce ||deforceFlag ||motor->enableFlag.justRead || motor->enableFlag.disable )
		{
			canBuff[can_i][idIndex * 4 + idOffset] = 0;
			motor->pidInner.fbValue=*motor->pidInner.fbValuePtr[0];
			motor->pidOuter.fbValue=*motor->pidOuter.fbValuePtr[0];
		}
		else
		canBuff[can_i][idIndex * 4 + idOffset] = motor->canResult;
		
}

void Motor::sendCanMessage(int can_i, int idIndex)
{
    int canId = (idIndex == 0) ? 0x200 : (idIndex == 1) ? 0x1ff : 0x2ff;
	
		for (int idOffset = 0; idOffset < 4; idOffset++)
    {
       if(motorList[can_i][idIndex * 4 + idOffset].motorPoint->torque_control==1 && motorList[can_i][idIndex * 4 + idOffset].motorPoint->getType()==GM6020)
			 canId = (idIndex == 0) ? 0x200 : (idIndex == 1) ? 0x1FE : 0x2FE;
		}
    canTx(&canBuff[can_i][idIndex * 4], (can_i == 0) ? CAN1 : CAN2, canId);
}

/**
Author: 郑竣元
Version: 1.3.1.191119_alpha
Date: 19.10.15
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.24 肖银河-改写数据合帧的方法，修复数据合帧时的BUG
	——————————————————————————————————————————————————————————————————————————
	19.11.19 肖银河-总体框架更改，将三大主体部分重新分配完成-核心功能已测试
	——————————————————————————————————————————————————————————————————————————
	19.11.15 肖银河-将方法的具体实现放到cpp文件
	——————————————————————————————————————————————————————————————————————————
	19.11.12 肖银河-将C_Motor分为Pid feedBack Motor三部分，并完善代码规范
	——————————————————————————————————————————————————————————————————————————
	19.10.15 郑竣元-首次完成
	——————————————————————————————————————————————————————————————————————————
	24.9.13  codeartist 重构，方便其他电机的复用
	——————————————————————————————————————————————————————————————————————————

*/