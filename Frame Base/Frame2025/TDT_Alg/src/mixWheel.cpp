#include "mixWheel.h"

mixWheel::mixWheel()
{
}

//待实现
//void mixWheel::nowSpeed_calculate(vec4f steeringFeedback, vec2f wheelFeedback)
//{
//	
//}

void mixWheel::mixWheel_calculate(vec3f speedDef)
{
    float vx_c = my_cos_rad(PI/4)*speedDef.data[0]+my_sin_rad(PI/4)*speedDef.data[1];
    float vy_c = -my_sin_rad(PI/4)*speedDef.data[0]+my_cos_rad(PI/4)*speedDef.data[1];
    float vw_c = speedDef.data[2];
    //计算FL轮速
    wheelSpeed.data[0] = -vx_c + L*vw_c;
    //计算FR轮速
    wheelSpeed.data[1] = vy_c + L*vw_c;
    //计算RL轮速
    if(vx_c + L*vw_c == 0)
    {
        if(vy_c == 0)
        {
            steerAngle.data[0] = steerAngleNow.data[0];
            wheelSpeed.data[2] = 0;
        }
        else
        {
            //判断更接近-pi/4还是3/4pi，灵活选择
						if(fabs(differangle_caculate(steerAngleNow.data[0]*RADIAN_TO_DEGREE,-45.0f)) <=90.0f)
							steerAngle.data[0] = - PI/4;
						else
							steerAngle.data[0] =  PI*3/4;
            wheelSpeed.data[2] = vy_c / my_sin_rad(steerAngle.data[0] - PI/4);
        }
    }
    else if(vy_c == 0)
    {
        //判断更接近pi/4还是5/4pi，灵活选择
				if(fabs(differangle_caculate(steerAngleNow.data[0]*RADIAN_TO_DEGREE,45.0f)) <=90.0f)
					steerAngle.data[0] = PI/4;
				else
					steerAngle.data[0] =  PI*5/4;
        wheelSpeed.data[2] = (vx_c + L*vw_c) / my_cos_rad(steerAngle.data[0] - PI/4);
    }
    else
    {
        steerAngle.data[0] = fast_atan2(vy_c,vx_c + L*vw_c) + PI/4;
				if(fabs(differangle_caculate(steerAngleNow.data[0]*RADIAN_TO_DEGREE,steerAngle.data[0]*RADIAN_TO_DEGREE)) >90.0f)
					steerAngle.data[0] +=  PI;
        wheelSpeed.data[2] = vy_c / my_sin_rad(steerAngle.data[0] - PI/4);
    }
    //计算RR轮速
    if(vx_c == 0)
    {
        if(-vy_c + L*vw_c == 0)
        {
            steerAngle.data[1] = steerAngleNow.data[1];
            wheelSpeed.data[3] = 0;
        }
        else
        {
            if(fabs(differangle_caculate(steerAngleNow.data[1]*RADIAN_TO_DEGREE,-45.0f)) <=90.0f)
							steerAngle.data[1] = - PI/4;
						else
							steerAngle.data[1] =  PI*3/4;
            wheelSpeed.data[3] = vy_c - L*vw_c / my_sin_rad(steerAngle.data[1] - PI/4);
        }
    }
    else if(-vy_c + L*vw_c == 0)
    {
       //后续判断更接近pi/4还是5/4pi，灵活选择
				if(fabs(differangle_caculate(steerAngleNow.data[1]*RADIAN_TO_DEGREE,45.0f)) <=90.0f)
					steerAngle.data[1] = PI/4;
				else
					steerAngle.data[1] =  PI*5/4;
        wheelSpeed.data[3] = vx_c / my_cos_rad(steerAngle.data[1] - PI/4);
    }
    else
    {
        steerAngle.data[1] = fast_atan2(vy_c - L*vw_c,vx_c) + PI/4;
				if(fabs(differangle_caculate(steerAngleNow.data[1]*RADIAN_TO_DEGREE,steerAngle.data[1]*RADIAN_TO_DEGREE)) >90.0f)
					steerAngle.data[0] +=  PI;
        wheelSpeed.data[3] = vx_c / my_sin_rad(steerAngle.data[1] - PI/4);
    }
}