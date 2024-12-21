#ifndef _BALLISTICCALC_H
#define _BALLISTICCALC_H

/*****************************************************************************
File name: TDT_Alg\inc\BallisticCalc.h
Description: �������ؽ���
Author: ����[
Version: 0.0.1
Date: 24.3.16
History: 
	����������������������������������������������������������������������������������������������������������������������������������������������������
	24.3.16 �״μ�¼
	����������������������������������������������������������������������������������������������������������������������������������������������������
*****************************************************************************/
#include "board.h"
#include "math.h"
#include "my_math.h"
//#include "NewtonMethod.h"

#define g 9.8
#define k  0.01	//��������ϵ��
#define M_PI 3.14

class BallisticCalc
{
public:
	uint8_t reachable;	//�����Ƿ�ɵ���
	double dest_height, dest_distance, calc_height;	//�Ӿ�����Ŀ����Ը߶ȡ�ˮƽ�����Լ����Ӿ�������������֮�����Ը߶�
	double kx;	//�����������Բ���ϵ����Ҳ�������������Ӿ������
	double visionErrDeadZone;	//�Ӿ�����������ھ���С�ڸ�����ʱ������
	BallisticCalc(double _k, double _visionErrDeadZone) : kx(_k), visionErrDeadZone(_visionErrDeadZone){};
	//double calcTrajectory(double angle, double velocity, double distance, double height);
	double calcAngle(double velocity, double distance, double height);
		
};


/*
�����ĵ���ģ�Ϳ����˿��������������������Ĵ�ᵼ��CPU��������Ҫʹ��

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