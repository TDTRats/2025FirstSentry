/*****************************************************************************
File name: TDT_Alg\src\BallisticCalc.cpp
Description: 弹道本地解算
Author: 杨沛�?
Version: 0.0.1
Date: 24.3.16
History: 
	—————————————————————————————————————————————————————————————————————————�?
	24.3.16 首次记录
	—————————————————————————————————————————————————————————————————————————�?
*****************************************************************************/
#include "BallisticCal.h"


/**
 * @brief:datas for Jscope
*/
double ang1, ang2, angJ, h_bal, x_bal;


/**
 * @brief:解算诸元
 * @param: 弹丸初速度
 * @param: 发射器与目标的水平距�?
 * @param: 发射器与目标相对高度，目标比发射器高为正
 * @note: 仰角在头朝上时为正，要注意将仰角用为云台pitch的目标角度时是否需要取反！
*/
double BallisticCalc::calcAngle(double velocity, double distance, double height)
{
	double angle1, angle2, theta;
	h_bal = height;
	x_bal = distance;
	dest_height = height;
	dest_distance = distance;
	
    //对视觉解算结果进行修正。但是需要注意的是误差机理尚未明确，所以修改逻辑有待进一步优�?
	if (distance > visionErrDeadZone)
	{
		height += (distance - visionErrDeadZone)*kx;
	}
	calc_height = height;

	double a = g * distance * distance / (2 * velocity * velocity);
    double b = -distance;
    double c = a + height;

    // 计算判别�?
    double discriminant = b * b - 4 * a * c;

    // 检查是否无�?
    if (discriminant >= 0) {
		reachable = 1;
        // 计算两个可能�?
        angle1 = atan((-b + my_sqrt(discriminant)) / (2 * a));
        angle2 = atan((-b - my_sqrt(discriminant)) / (2 * a));
//如果算力拮据可以使用快速三角函数计�?
//		angle1 = fast_atan2(-b + my_sqrt(discriminant), 2*a);
//		angle2 = fast_atan2(-b - my_sqrt(discriminant), 2*a);


        // Choose the smaller positive angle as the required elevation angle
		ang1 = angle1;
		ang2 = angle2;
		
        theta = (abs(angle1) < abs(angle2)) ? angle1 : angle2;  //解的判别
        return theta * 180 / M_PI; // Convert to degrees
    } else {
        // If the discriminant is negative, the target is not reachable
        reachable = 0;
		return -1;
    }
}

/*
double AirBallisticCalc::findAngle(double judgAng, double velocity, double distance, double height)
{
	_velocity = velocity;
    _distance = distance;
    _height = height;

    double ang;
    ang = Solve(0, judgAng);

    if (Solvable)
    {
        reachable = 1;
        return ang;
    }
    else
    {
        reachable = 0;
        return -1;
    }
}



double AirBallisticCalc::calcTrajectory(double angle, double velocity, double distance, double height)
{
	double x = 0, y = 0; // Initial position
    double vx = velocity * my_cos(angle), vy = velocity * my_sin(angle); // Initial velocity components
    double dt = 0.01; // Time step

    // Simulate the trajectory
    while (x < distance) 
	{
        // Update position
        x += vx * dt;
        y += vy * dt;

        // Update velocity
        double v = my_sqrt(vx * vx + vy * vy);
        vx -= k * v * vx * dt;
        vy -= (g + k * v * vy) * dt;
    }

    // Return the vertical distance from the target
    return y - height;
}
*/