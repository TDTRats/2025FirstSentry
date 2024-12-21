#include "chassis_task.h"

Chassis::Chassis()
{
	initMotor();
	pidParamLoad();
	state = new State_class<uint8_t,Chassis>(this,&Chassis::feedBack);
}

void Chassis::init()
{
	status=taskStateRun;
	taskHz=_1000hz;
}

void Chassis::run()
{
	power.getPowerKp();
	if(!deforceFlag)
	{
		if(*state == 3)//纯手动模式
		{
			speedSet.data[0] = RC.Key.CH[0];
			speedSet.data[1] = RC.Key.CH[1];
			speedSet.data[2] = RC.Key.CH[11];
		}
		else if(*state == 2)//开环原地小陀螺
		{
			speedSet.data[0] = 0;
			speedSet.data[1] = 0;
			speedSet.data[2] = ROTATE_SPEED;
		}
		else if(*state == 1)//底盘跟随模式
		{
			speedSet.data[0] = RC.Key.CH[0];
			speedSet.data[1] = RC.Key.CH[1];
			speedSet.data[2] = follow.pidInner.result;
		}
		else if(*state == 0)//？待定
		{
			
		}
		wheelCalc.mixWheel_calculate(speedSet);
		outAngle();
		outSpeed();
	}
	//电机自动脱力
}

void Chassis::feedBack(uint8_t from,uint8_t to)
{
	uint8_t transition = from<<4 | to;
	switch(transition)
	{
		case 0x01:
			
			break;
		default:
			break;
	}
}

void Chassis::outSpeed()
{
	if(isDirectionOk)
	{
		wheels.omLeft.speedMotor->ctrlSpeed(wheelCalc.wheelSpeed.data[0]);
		wheels.omRight.speedMotor->ctrlSpeed(wheelCalc.wheelSpeed.data[1]);
		wheels.stLeft.speedMotor->ctrlSpeed(wheelCalc.wheelSpeed.data[2]);
		wheels.stRight.speedMotor->ctrlSpeed(wheelCalc.wheelSpeed.data[3]);
	}
	else
	{
		wheels.omLeft.speedMotor->ctrlSpeed(0);
		wheels.omRight.speedMotor->ctrlSpeed(0);
		wheels.stLeft.speedMotor->ctrlSpeed(0);
		wheels.stRight.speedMotor->ctrlSpeed(0);
	}
}

void Chassis::outAngle()
{
	wheels.stLeft.angleMotor->ctrlPosition(wheelCalc.steerAngle.data[1]);
	wheels.stRight.angleMotor->ctrlPosition(wheelCalc.steerAngle.data[0]);
}

void Chassis::pidParamLoad()
{
	//Left omni wheel pid coefficient
	wheels.omLeft.innerParam_Speed.kp = 10;
	wheels.omLeft.innerParam_Speed.ki = 0;
	wheels.omLeft.innerParam_Speed.kd = 0;
	wheels.omLeft.innerParam_Speed.integralErrorMax = 100;
	wheels.omLeft.innerParam_Speed.resultMax = wheels.omLeft.speedMotor->getMotorCurrentLimit();
	wheels.omLeft.outerParam_Speed.kp = 10;
	wheels.omLeft.outerParam_Speed.ki = 0;
	wheels.omLeft.outerParam_Speed.kd = 0;
	wheels.omLeft.outerParam_Speed.integralErrorMax = 100;
	wheels.omLeft.outerParam_Speed.resultMax = wheels.omLeft.speedMotor->getMotorSpeedLimit();
	//Right omni wheel pid coefficient
	wheels.omRight.innerParam_Speed.kp = 10;
	wheels.omRight.innerParam_Speed.ki = 0;
	wheels.omRight.innerParam_Speed.kd = 0;
	wheels.omRight.innerParam_Speed.integralErrorMax = 100;
	wheels.omRight.innerParam_Speed.resultMax = wheels.omRight.speedMotor->getMotorCurrentLimit();
	wheels.omRight.outerParam_Speed.kp = 10;
	wheels.omRight.outerParam_Speed.ki = 0;
	wheels.omRight.outerParam_Speed.kd = 0;
	wheels.omRight.outerParam_Speed.integralErrorMax = 100;
	wheels.omRight.outerParam_Speed.resultMax = wheels.omRight.speedMotor->getMotorSpeedLimit();
	//Left steering wheel pid coefficient
	wheels.stLeft.innerParam_Speed.kp = 10;
	wheels.stLeft.innerParam_Speed.ki = 0;
	wheels.stLeft.innerParam_Speed.kd = 0;
	wheels.stLeft.innerParam_Speed.integralErrorMax = 100;
	wheels.stLeft.innerParam_Speed.resultMax = wheels.stLeft.speedMotor->getMotorCurrentLimit();
	wheels.stLeft.outerParam_Speed.kp = 10;
	wheels.stLeft.outerParam_Speed.ki = 0;
	wheels.stLeft.outerParam_Speed.kd = 0;
	wheels.stLeft.outerParam_Speed.integralErrorMax = 100;
	wheels.stLeft.outerParam_Speed.resultMax = wheels.stLeft.speedMotor->getMotorSpeedLimit();
	//+
	wheels.stLeft.innerParam_Angle.kp = 10;
	wheels.stLeft.innerParam_Angle.ki = 0;
	wheels.stLeft.innerParam_Angle.kd = 0;
	wheels.stLeft.innerParam_Angle.integralErrorMax = 100;
	wheels.stLeft.innerParam_Angle.resultMax = wheels.stLeft.angleMotor->getMotorCurrentLimit();
	wheels.stLeft.outerParam_Angle.kp = 10;
	wheels.stLeft.outerParam_Angle.ki = 0;
	wheels.stLeft.outerParam_Angle.kd = 0;
	wheels.stLeft.outerParam_Angle.integralErrorMax = 100;
	wheels.stLeft.outerParam_Angle.resultMax = wheels.stLeft.angleMotor->getMotorSpeedLimit();
	//Right steering wheel pid coefficient
	wheels.stRight.innerParam_Speed.kp = 10;
	wheels.stRight.innerParam_Speed.ki = 0;
	wheels.stRight.innerParam_Speed.kd = 0;
	wheels.stRight.innerParam_Speed.integralErrorMax = 100;
	wheels.stRight.innerParam_Speed.resultMax = wheels.stRight.speedMotor->getMotorCurrentLimit();
	wheels.stRight.outerParam_Speed.kp = 10;
	wheels.stRight.outerParam_Speed.ki = 0;
	wheels.stRight.outerParam_Speed.kd = 0;
	wheels.stRight.outerParam_Speed.integralErrorMax = 100;
	wheels.stRight.outerParam_Speed.resultMax = wheels.stRight.speedMotor->getMotorSpeedLimit();
	//+
	wheels.stRight.innerParam_Angle.kp = 10;
	wheels.stRight.innerParam_Angle.ki = 0;
	wheels.stRight.innerParam_Angle.kd = 0;
	wheels.stRight.innerParam_Angle.integralErrorMax = 100;
	wheels.stRight.innerParam_Angle.resultMax = wheels.stRight.angleMotor->getMotorCurrentLimit();
	wheels.stRight.outerParam_Angle.kp = 10;
	wheels.stRight.outerParam_Angle.ki = 0;
	wheels.stRight.outerParam_Angle.kd = 0;
	wheels.stRight.outerParam_Angle.integralErrorMax = 100;
	wheels.stRight.outerParam_Angle.resultMax = wheels.stRight.angleMotor->getMotorSpeedLimit();
	
	//Load pid coefficient
	//Omni
	wheels.omLeft.speedMotor->pidInner.paramPtr=&wheels.omLeft.innerParam_Speed;
	wheels.omLeft.speedMotor->pidOuter.paramPtr=&wheels.omLeft.outerParam_Speed;
	
	wheels.omRight.speedMotor->pidInner.paramPtr=&wheels.omRight.innerParam_Speed;
	wheels.omRight.speedMotor->pidOuter.paramPtr=&wheels.omRight.outerParam_Speed;
	//Steering
	wheels.stLeft.speedMotor->pidInner.paramPtr=&wheels.stLeft.innerParam_Speed;
	wheels.stLeft.speedMotor->pidOuter.paramPtr=&wheels.stLeft.outerParam_Speed;
	wheels.stLeft.angleMotor->pidInner.paramPtr=&wheels.stLeft.innerParam_Angle;
	wheels.stLeft.angleMotor->pidOuter.paramPtr=&wheels.stLeft.outerParam_Angle;
	
	wheels.stRight.speedMotor->pidInner.paramPtr=&wheels.stRight.innerParam_Speed;
	wheels.stRight.speedMotor->pidOuter.paramPtr=&wheels.stRight.outerParam_Speed;
	wheels.stRight.angleMotor->pidInner.paramPtr=&wheels.stRight.innerParam_Angle;
	wheels.stRight.angleMotor->pidOuter.paramPtr=&wheels.stRight.outerParam_Angle;
	
	follow.innerParam.kp = 10;
	follow.innerParam.ki = 0;
	follow.innerParam.kd = 0;
	follow.innerParam.integralErrorMax = 10;
	follow.innerParam.resultMax = 1000;
	
	follow.outerParam.kp = 10;
	follow.outerParam.ki = 0;
	follow.outerParam.kd = 0;
	follow.outerParam.integralErrorMax = 100;
	follow.outerParam.resultMax = 1000;
	
	follow.pidInner.paramPtr=&follow.innerParam;
	follow.pidOuter.paramPtr=&follow.outerParam;
	follow.pidOuter.fbValuePtr[0]=&infoProcess.gimbalMsg.angleDelta;
	follow.pidInner.fbValuePtr[0]=&follow.pidOuter.result;
}

void Chassis::initMotor()
{
	wheels.omLeft.speedMotor = new Motor(M3508,CAN1,0x201);
	wheels.omRight.speedMotor = new Motor(M3508,CAN1,0x202);
	wheels.stLeft.angleMotor = new Motor(M3508,CAN1,0x203);
	wheels.stLeft.speedMotor = new Motor(M3508,CAN1,0x204);
	wheels.stRight.angleMotor = new Motor(M3508,CAN1,0x205);
	wheels.stRight.speedMotor = new Motor(M3508,CAN1,0x206);
	
	wheels.omLeft.speedMotor->setPowerOutLimit(&power.powerLimitKp);
	wheels.omRight.speedMotor->setPowerOutLimit(&power.powerLimitKp);
	wheels.stLeft.angleMotor->setPowerOutLimit(&power.powerLimitKp);
	wheels.stLeft.speedMotor->setPowerOutLimit(&power.powerLimitKp);
	wheels.stRight.angleMotor->setPowerOutLimit(&power.powerLimitKp);
	wheels.stRight.speedMotor->setPowerOutLimit(&power.powerLimitKp);
}

void Chassis::stateUpdate()
{
	*state = infoProcess.gimbalMsg.state;
}