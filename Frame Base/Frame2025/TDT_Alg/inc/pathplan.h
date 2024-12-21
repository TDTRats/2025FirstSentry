/*
 * @Author: handsome
 * @Date: 2022-11-30
 * @LastEditTime: 2022-11-30
 * @LastEditors: handsome
 * @Description: 电机设定值路径规划
 */
#ifndef _PATHPLAN_H_
#define _PATHPLAN_H_

#include "board.h"
#include "cycle.h"

//路径规划结构体
struct PathPlan
{
    float nowTime;
    float param[6];
    Cycle cycle;
};

//路径规划计算设定值函数
//参数：路径规划结构体、初位置、末位置、期望时间
float pathPlanCalt(PathPlan *path,float x0,float x1,float t1);

void clearTime(PathPlan *path1,PathPlan *path2=0,PathPlan *path3=0,PathPlan *path4=0,PathPlan *path5=0,PathPlan *path6=0,PathPlan *path7=0,PathPlan *path8=0);


//
struct RealTimeInfo
{
    float realTimeSpeed,lastPosition;
    float realTimeAcc,lastSpeed;
    Cycle cycle;
    u8 count;
};

void getRealTimeInfo(RealTimeInfo* realTimeInfo,float realTimePosition);
void clearInfo(RealTimeInfo* realTimeInfo);

#endif