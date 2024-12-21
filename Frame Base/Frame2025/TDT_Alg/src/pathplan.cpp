/*
 * @Author: handsome
 * @Date: 2022-11-30
 * @LastEditTime: 2022-11-30
 * @LastEditors: handsome
 * @Description: 电机设定值路径规划
 */
#include "pathPlan.h"

float pathPlanCalt(PathPlan *path,float x0,float x1,float t1)
{   
    if(path->nowTime==0)
    {
        //限定条件：始末速度和始末加速度均为零
        float v0=0, v1=0, a0=0, a1=0;
        //确定参数及固定曲线
        path->param[0] = x0;                                                                                                  //0????
        path->param[1] = v0;                                                                                                  //1????
        path->param[2] = a0 / 2;                                                                                              //2????
        path->param[3] = (20 * x1 - 20 * x0 - (8 * v1 + 12 * v0) * t1 - (3 * a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 3));      //3????
        path->param[4] = (30 * x0 - 30 * x1 + (14 * v1 + 16 * v0) * t1 + (3 * a0 - 2 * a1) * powf(t1, 2)) / (2 * pow(t1, 4)); //4????
        path->param[5] = (12 * x1 - 12 * x0 - (6 * v1 + 6 * v0) * t1 - (a0 - a1) * powf(t1, 2)) / (2 * pow(t1, 5));           //5????
    }
    path->nowTime += path->cycle.getCycleT();//累加法获取总时间
    
    float x=0;
    
    //运动过程中计算三次多项式的值
    if(path->nowTime<=t1)
    {
        for (u8 j = 0; j < 6; j++)
		{
            x += path->param[j] * powf(path->nowTime, j);
		}
    }
	//时间到达期望时间，由于末速度和末加速度均为零，忽略位置差，认为已到达指定位置
    else
    {
        x = x1;
    }
    return x;
}


void clearTime(PathPlan *path1,PathPlan *path2,PathPlan *path3,PathPlan *path4,PathPlan *path5,PathPlan *path6,PathPlan *path7,PathPlan *path8)
{
    memset(path1, 0, sizeof(*path1));
    if(path2!=0)
        memset(path2, 0, sizeof(*path2));
    if(path3!=0)
        memset(path3, 0, sizeof(*path3));
    if(path4!=0)
        memset(path4, 0, sizeof(*path4));
    if(path5!=0)
        memset(path5, 0, sizeof(*path5));
    if(path6!=0)
        memset(path6, 0, sizeof(*path6));
    if(path7!=0)
        memset(path7, 0, sizeof(*path7));
}







void getRealTimeInfo(RealTimeInfo* realTimeInfo,float realTimePosition)
{
    
    float dt = realTimeInfo->cycle.getCycleT();
    
    switch(realTimeInfo->count)
    {
        case 0://
            realTimeInfo->lastPosition = realTimePosition;
            realTimeInfo->count++;
            return;
        case 1://
            realTimeInfo->lastSpeed = (realTimePosition-realTimeInfo->lastPosition)/dt;
            realTimeInfo->lastPosition = realTimePosition;
            realTimeInfo->count++;
            return;
        case 2:
            break;
    }
    
    
    realTimeInfo->realTimeSpeed = (realTimePosition-realTimeInfo->lastPosition)/dt;
    realTimeInfo->lastPosition = realTimePosition;
    
    //
    realTimeInfo->realTimeAcc = (realTimeInfo->realTimeSpeed-realTimeInfo->lastSpeed)/dt;
    realTimeInfo->lastSpeed = realTimeInfo->realTimeSpeed;
    
}

void clearInfo(RealTimeInfo* realTimeInfo)
{
    memset(realTimeInfo, 0, sizeof(*realTimeInfo));
}


