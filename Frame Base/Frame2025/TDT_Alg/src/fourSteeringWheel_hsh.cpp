/*
 * @Author: hsh
 * @Date: 2022-11-24 21:22:20
 * @LastEditTime: 2022-11-24
 * @LastEditors: liuskywalkerjskd:修复恶性bug并进行运动优化
 * @Description: 四舵轮速度解算——矢量叠加法
 */ 
#include "fourSteeringWheel_hsh.h"
#include "my_math.h"
#include "math.h"

#define RADIAN_TO_DEGREE 57.3f
#define DEGREE_TO_RADIAN 0.017452f

#define WHEEL_CAN_BACK 1

float meca_angle1,meca_angle2;
FourSteeringWheel::FourSteeringWheel()
{
	steeringPara.wheelbase = WHEELBASE;	  //L
	steeringPara.wheeltrack = STEER_WHEELTRACK ; //D
	steeringPara.rotate_x_offset = ROTATE_X_OFFSET;
	steeringPara.rotate_y_offset = ROTATE_Y_OFFSET;
	steeringPara.wheel_perimeter = PERIMETER;
	steeringPara.W_Cdistance = my_sqrt(my_pow(WHEELBASE) + my_pow(STEER_WHEELTRACK ));
}

void FourSteeringWheel::fourSteeringWheel_calculate(vec3f speedDef, vec4f wheelFeedback)
{	
	
	wheels.RF.angleFdb = wheelFeedback.data[0];
	wheels.LF.angleFdb = wheelFeedback.data[1];
	wheels.LB.angleFdb = wheelFeedback.data[2];
	wheels.RB.angleFdb = wheelFeedback.data[3];
	
	
	steeringSpeed.vy = LIMIT(speedDef.data[0], -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); //mm/s
	steeringSpeed.vx = LIMIT(speedDef.data[1], -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); //mm/s
	steeringSpeed.vw = LIMIT(speedDef.data[2], -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED); //rad/s
	steeringSpeed.linearVelocity = steeringSpeed.vw * STEER_WHEELTRACK/2.0f;
	steeringSpeed.lineraVelocity2= steeringSpeed.vw *QUAN_WHEELTRACK/2.0f;
	
	//当三个速度向量均很小时则不进行运算
	if ((ABS(steeringSpeed.vy) <= 50 && ABS(steeringSpeed.vx) <= 50 && ABS(steeringSpeed.vw) <= 0.3)
		||((ABS(steeringSpeed.vy + steeringSpeed.linearVelocity * sinf(PI / 4))<=0.2 &&
			ABS(steeringSpeed.vx - steeringSpeed.linearVelocity * sinf(PI / 4))<=0.2)	))	
	{
		
		outputData.meca_speed.data[0]=0;
																	
		outputData.meca_speed.data[1]=0;
																	
		outputData.meca_speed.data[2]=0;
																	
		outputData.meca_speed.data[3]=0;

		/*输出结构体赋值*/
		outputData.angle.data[0] =  wheels.RF.angleFdb;
		outputData.angle.data[1] =  wheels.LF.angleFdb;
		outputData.angle.data[2] =	wheels.LB.angleFdb;
		outputData.angle.data[3] =  wheels.RB.angleFdb;
		

		outputData.speed_rpm.data[0] = 0;
		outputData.speed_rpm.data[1] = 0;
		outputData.speed_rpm.data[2] = 0;
		outputData.speed_rpm.data[3] = 0;
		return;
	}
	
		//各轮组合速度大小
		wheels.RF.speed = my_sqrt(my_pow(steeringSpeed.vx -  steeringSpeed.linearVelocity * cosf(PI / 4)) 
		+ my_pow(steeringSpeed.vy + steeringSpeed.linearVelocity* sinf(PI / 4)));
	
		wheels.LF.speed = my_sqrt(my_pow(steeringSpeed.vx - steeringSpeed.linearVelocity * cosf(PI / 4)) 
		+ my_pow(steeringSpeed.vy - steeringSpeed.linearVelocity * sinf(PI / 4)));
	
		wheels.LB.speed = my_sqrt(my_pow(steeringSpeed.vx +  steeringSpeed.linearVelocity * cosf(PI / 4)) 
		+ my_pow(steeringSpeed.vy -  steeringSpeed.linearVelocity* sinf(PI / 4)));
	
		wheels.RB.speed = my_sqrt(my_pow(steeringSpeed.vx + steeringSpeed.linearVelocity * cosf(PI / 4)) 
		+ my_pow(steeringSpeed.vy + steeringSpeed.linearVelocity * sinf(PI / 4)));
		
		wheels.RF.angle = fast_atan2(steeringSpeed.vy +  steeringSpeed.linearVelocity * sinf(PI / 4),
		steeringSpeed.vx -  steeringSpeed.linearVelocity * cosf(PI / 4));
		
		wheels.LF.angle = fast_atan2(steeringSpeed.vy - steeringSpeed.linearVelocity * sinf(PI / 4),
		steeringSpeed.vx - steeringSpeed.linearVelocity * cosf(PI / 4));
	
		wheels.LB.angle = fast_atan2(steeringSpeed.vy -  steeringSpeed.linearVelocity * sinf(PI / 4),
		steeringSpeed.vx +  steeringSpeed.linearVelocity * cosf(PI / 4));
		
		wheels.RB.angle = fast_atan2(steeringSpeed.vy + steeringSpeed.linearVelocity * sinf(PI / 4),
		steeringSpeed.vx + steeringSpeed.linearVelocity * cosf(PI / 4));
		
		
		
		
			//统一弧度为角度
		wheels.RF.angle *=  RADIAN_TO_DEGREE;
		wheels.LF.angle *=  RADIAN_TO_DEGREE;
		wheels.LB.angle *=  RADIAN_TO_DEGREE;
		wheels.RB.angle *=  RADIAN_TO_DEGREE;
		
		//统一零点为y轴 逆时针正，-180，180
		wheels.RF.angle-=90;
		if(wheels.RF.angle<-180) wheels.RF.angle+=360;
		
		wheels.LF.angle-=90;
		if(wheels.LF.angle<-180) wheels.LF.angle+=360;
		
		wheels.LB.angle-=90;
		if(wheels.LB.angle<-180) wheels.LB.angle+=360;
		
		wheels.RB.angle-=90;
		if(wheels.RB.angle<-180) wheels.RB.angle+=360;
		
		
			//各轮组合速度大小
		wheels.quan_RF.speed = my_sqrt(my_pow(steeringSpeed.vx - steeringSpeed.lineraVelocity2 * cosf(PI / 4)) 
		+ my_pow(steeringSpeed.vy + steeringSpeed.lineraVelocity2* sinf(PI / 4)));
		
		wheels.quan_LF.speed = my_sqrt(my_pow(steeringSpeed.vx - steeringSpeed.lineraVelocity2 * cosf(PI / 4)) 
		+ my_pow(steeringSpeed.vy - steeringSpeed.lineraVelocity2 * sinf(PI / 4)));
	
		wheels.quan_LB.speed = my_sqrt(my_pow(steeringSpeed.vx +  steeringSpeed.lineraVelocity2  * cosf(PI / 4)) 
		+ my_pow(steeringSpeed.vy -  steeringSpeed.lineraVelocity2 * sinf(PI / 4)));
	
		wheels.quan_RB.speed = my_sqrt(my_pow(steeringSpeed.vx + steeringSpeed.lineraVelocity2 * cosf(PI / 4)) 
		+ my_pow(steeringSpeed.vy + steeringSpeed.lineraVelocity2 * sinf(PI / 4)));
		
		
		wheels.quan_RF.angle=wheels.RF.angle-45;
		if(wheels.quan_RF.angle<-180) 
			wheels.quan_RF.angle+=360;
		
		wheels.quan_LF.angle=wheels.LF.angle+45;
		if(wheels.quan_LF.angle>180) 
			wheels.quan_LF.angle-=360;
		
		wheels.quan_LB.angle=wheels.LB.angle-45;
		if(wheels.quan_LB.angle<-180) 
			wheels.quan_LB.angle+=360;
		
		wheels.quan_RB.angle=wheels.RB.angle+45;
		if(wheels.quan_RB.angle>180) 
			wheels.quan_RB.angle-=360;
		
		
		
		
		
		float angle1,angle2,angle3,angle4;
		
		angle1=wheels.RF.angle;
		angle2=wheels.LF.angle;
		angle3=wheels.LB.angle;
		angle4=wheels.RB.angle;
		
#if WHEEL_CAN_BACK
		
  		if(ABS(differangle_caculate(wheels.RF.angle,wheels.RF.angleFdb))>=90)
		{
			if(differangle_caculate(wheels.RF.angle,wheels.RF.angleFdb)<0)
				wheels.RF.angle+=180;
			else
				wheels.RF.angle-=180;
		}
		
			if(ABS(differangle_caculate(wheels.LF.angle,wheels.LF.angleFdb))>=90)
		{
			
			if(differangle_caculate(wheels.LF.angle,wheels.LF.angleFdb)<0)
				wheels.LF.angle+=180;
			else
				wheels.LF.angle-=180;
		}
		
			if(ABS(differangle_caculate(wheels.LB.angle,wheels.LB.angleFdb))>=90)
		{
			if(differangle_caculate(wheels.LB.angle,wheels.LB.angleFdb)<0)
				wheels.LB.angle+=180;
			else
				wheels.LB.angle-=180;
		}
		
			if(ABS(differangle_caculate(wheels.RB.angle,wheels.RB.angleFdb))>=90)
		{
			if(differangle_caculate(wheels.RB.angle,wheels.RB.angleFdb)<0)
				wheels.RB.angle+=180;
			else
				wheels.RB.angle-=180;
		}
		
		if(ABS(wheels.RF.angle-angle1)>90)	
		wheels.RF.speed*=-1;
		
		if(ABS(wheels.LF.angle-angle2)>90)
		wheels.LF.speed*=-1;
		
		if(ABS(wheels.LB.angle-angle3)>90)
		wheels.LB.speed*=-1;
		
		if(ABS(wheels.RB.angle-angle4)>90)
		wheels.RB.speed*=-1;
#endif 
		
		
		/*设定值转化为当前角度圈数下的设定值*/
		wheels.RF.angle=wheels.RF.angleFdb+differangle_caculate(wheels.RF.angle,wheels.RF.angleFdb);
		
		wheels.LF.angle=wheels.LF.angleFdb+differangle_caculate(wheels.LF.angle,wheels.LF.angleFdb);
		
		wheels.LB.angle=wheels.LB.angleFdb+differangle_caculate(wheels.LB.angle,wheels.LB.angleFdb);
		
		wheels.RB.angle=wheels.RB.angleFdb+differangle_caculate(wheels.RB.angle,wheels.RB.angleFdb);
		
		//* MOTOR_DECELE_RATIO_3508; //mm/s --->n/min 转子转速
		wheels.RF.speed_rpm = wheels.RF.speed / steeringPara.wheel_perimeter * 60 / MOTOR_DECELE_RATIO_3508;
		wheels.LF.speed_rpm = wheels.LF.speed / steeringPara.wheel_perimeter * 60 / MOTOR_DECELE_RATIO_3508;
		wheels.LB.speed_rpm = wheels.LB.speed / steeringPara.wheel_perimeter * 60 / MOTOR_DECELE_RATIO_3508;
		wheels.RB.speed_rpm = wheels.RB.speed / steeringPara.wheel_perimeter * 60 / MOTOR_DECELE_RATIO_3508;
		
		
		
	
		outputData.meca_speed.data[0]=wheels.quan_RF.speed*my_cos_angle(wheels.quan_RF.angle)/ QUAN_PERIMETER * 60 / MOTOR_DECELE_RATIO_3508;
			
		outputData.meca_speed.data[1]=wheels.quan_LF.speed*my_cos_angle(wheels.quan_LF.angle)/ QUAN_PERIMETER * 60 / MOTOR_DECELE_RATIO_3508;
		
		outputData.meca_speed.data[2]=wheels.quan_LB.speed*my_cos_angle(wheels.quan_LB.angle)/ QUAN_PERIMETER * 60 / MOTOR_DECELE_RATIO_3508;
		
		outputData.meca_speed.data[3]=wheels.quan_RB.speed*my_cos_angle(wheels.quan_RB.angle)/ QUAN_PERIMETER * 60 / MOTOR_DECELE_RATIO_3508;

		/*输出结构体赋值*/
		outputData.angle.data[0] = wheels.RF.angle;
		outputData.angle.data[1] = wheels.LF.angle;
		outputData.angle.data[2] = wheels.LB.angle;
		outputData.angle.data[3] = wheels.RB.angle;
		

		outputData.speed_rpm.data[0] = wheels.RF.speed_rpm;
		outputData.speed_rpm.data[1] = wheels.LF.speed_rpm;
		outputData.speed_rpm.data[2] = wheels.LB.speed_rpm;
		outputData.speed_rpm.data[3] = wheels.RB.speed_rpm;
	
}

bool FourSteeringWheel::judgeWheelPrepared(vec4f angleFeedback)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		wheelAngleError.data[i] = outputData.angle.data[i] - angleFeedback.data[i];
	}
	maxError = Math_Max(ABS(wheelAngleError.data[0]), ABS(wheelAngleError.data[1]), ABS(wheelAngleError.data[2]), ABS(wheelAngleError.data[3]), 0, 0);
	if(maxError > 15.0f)
	{
		return 0;
	}
	return 1;
}


void FourSteeringWheel::nowSpeed_calculate(vec4f steeringFeedback, vec4f wheelFeedback)
{
	//根据舵坐标系转换反馈角度 -> 弧度制
	for(u8 i = 0; i < 4; i++)
	{
		feedbackData.feedbackAngle.data[i] = PI * 3 / 4 - steeringFeedback.data[i] * DEGREE_TO_RADIAN;
	}
	feedbackData.feedbackSpeed.data[0] = (wheelFeedback.data[0] * my_sin_rad(feedbackData.feedbackAngle.data[0])
										 -wheelFeedback.data[1] * my_cos_rad(feedbackData.feedbackAngle.data[1])
										 +wheelFeedback.data[2] * my_sin_rad(feedbackData.feedbackAngle.data[2])
										 -wheelFeedback.data[3] * my_cos_rad(feedbackData.feedbackAngle.data[3]))
										 * MOTOR_DECELE_RATIO_3508 * steeringPara.wheel_perimeter / 60.f * sinf(PI / 4);
	feedbackData.feedbackSpeed.data[1] = (wheelFeedback.data[0] * my_sin_rad(feedbackData.feedbackAngle.data[0])
										 -wheelFeedback.data[1] * my_cos_rad(feedbackData.feedbackAngle.data[1])
										 +wheelFeedback.data[2] * my_sin_rad(feedbackData.feedbackAngle.data[2])
										 -wheelFeedback.data[3] * my_cos_rad(feedbackData.feedbackAngle.data[3]))
										 * MOTOR_DECELE_RATIO_3508 * steeringPara.wheel_perimeter / 60.f * sinf(PI / 4);
	feedbackData.feedbackSpeed.data[2] = (-wheelFeedback.data[0] * my_cos_rad(feedbackData.feedbackAngle.data[0])
										  -wheelFeedback.data[1] * my_sin_rad(feedbackData.feedbackAngle.data[1])
										  +wheelFeedback.data[2] * my_cos_rad(feedbackData.feedbackAngle.data[2])
										  +wheelFeedback.data[3] * my_sin_rad(feedbackData.feedbackAngle.data[3]))
										  * MOTOR_DECELE_RATIO_3508 * steeringPara.wheel_perimeter / 60.f / (steeringPara.W_Cdistance / 2);
	
}