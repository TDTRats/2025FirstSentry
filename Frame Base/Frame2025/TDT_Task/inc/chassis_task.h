#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "board.h"
#include "task_virtual.h"
#include "motor.h"
#include "mixWheel.h"
#include "task_virtual.h"
#include "state_task.h"
#include "dbus.h"
#include "info_task.h"
#include "power.h"
#include "pid.h"

#define ROTATE_SPEED 1000

class Chassis : public VirtualTask
{
public:
	Chassis();
	void init() override;
	void run() override;
	void feedBack(uint8_t,uint8_t);
	void outSpeed();
	void outAngle();
	void initMotor();
	void pidParamLoad();
	void stateUpdate();
private:
	
	typedef struct _steer
	{
		Motor* angleMotor;
		Motor* speedMotor;
		PidParam innerParam_Angle;
		PidParam outerParam_Angle;
		PidParam innerParam_Speed;
		PidParam outerParam_Speed;
	}sWheel;
	
	typedef struct _omni
	{
		Motor* speedMotor;
		PidParam innerParam_Speed;
		PidParam outerParam_Speed;
	}oWheel;
	
	struct _wheels
	{
		oWheel omLeft;
		oWheel omRight;
		sWheel stLeft;
		sWheel stRight;
	}wheels;
	
	struct _follow
	{
		Pid pidInner,pidOuter;
		PidParam innerParam,outerParam;
	}follow;
	
	State_class<uint8_t,Chassis>* state;
	mixWheel wheelCalc;
	vec3f speedSet;
	vec2f wheelAngleFb;
	bool isDirectionOk;
};
#endif