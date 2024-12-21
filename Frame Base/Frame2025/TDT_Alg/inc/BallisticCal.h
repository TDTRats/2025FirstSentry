#ifndef _BALLISTICCALC_H
#define _BALLISTICCALC_H

/*****************************************************************************
File name: TDT_Alg\inc\BallisticCalc.h
Description: 弹道本地解算
Author: 杨沛璠
Version: 0.0.1
Date: 24.3.16
History: 
	——————————————————————————————————————————————————————————————————————————
	24.3.16 首次记录
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#include "board.h"
#include "math.h"
#include "my_math.h"
//#include "NewtonMethod.h"

#define g 9.8
#define k  0.01	//空气阻力系数
#define M_PI 3.14

class BallisticCalc
{
public:
	uint8_t reachable;	//弹道是否可到达
	double dest_height, dest_distance, calc_height;	//视觉解算目标相对高度、水平距离以及对视觉进行线性修正之后的相对高度
	double kx;	//空气阻力线性补偿系数，也可以用来修正视觉的误差
	double visionErrDeadZone;	//视觉误差死区，在距离小于个距离时误差不存在
	BallisticCalc(double _k, double _visionErrDeadZone) : kx(_k), visionErrDeadZone(_visionErrDeadZone){};
	//double calcTrajectory(double angle, double velocity, double distance, double height);
	double calcAngle(double velocity, double distance, double height);
		
};


/*
这个类的弹道模型考虑了空气阻力，但是算力消耗大会导致CPU死机，不要使用

class AirBallisticCalc : public NM_Equation
{
public:
	uint8_t reachable;
	double _velocity, _distance, _height;
	AirBallisticCalc(uint16_t _maxTime_) : NM_Equation(_maxTime_, 0.01f, 0.001f){};
	double equationFunc(double x) override 
	{
		return calcTrajectory(x, _velocity, _distance, _height);
	};
	double calcTrajectory(double angle, double velocity, double distance, double height);
	double findAngle(double judgAng, double velocity, double distance, double height);
};
*/

#endif