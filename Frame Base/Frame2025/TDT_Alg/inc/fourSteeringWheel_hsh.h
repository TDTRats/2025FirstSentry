/*
 * @Author: hsh
 * @Date: 2022-11-24 21:22:20
 * @LastEditTime: 2022-11-24
 * @LastEditors: hsh
 * @Description: �Ķ����ٶȽ��㡪��ʸ�����ӷ����ο�2021���������ֲ�����Դ��
 */
#ifndef _FOURSTEERINGWHEEL_HSH_H_
#define _FOURSTEERINGWHEEL_HSH_H_
#include "board.h"
/************ �����˻������� ************/

/* the radius of wheel(mm) �뾶*/
#define RADIUS 50
/* the perimeter of wheel(mm) �ܳ�*/
#define PERIMETER 314

#define QUAN_PERIMETER  480

/* wheel track distance(mm) �־�*/
#define STEER_WHEELTRACK  420
#define QUAN_WHEELTRACK  532

/* wheelbase distance(mm) ���*/
#define WHEELBASE 339.41
/* gimbal is relative to chassis center x axis offset(mm) ��̨���Ĳ���*/
#define ROTATE_X_OFFSET 0
/* gimbal is relative to chassis center y axis offset(mm) */
#define ROTATE_Y_OFFSET 0

/* chassis motor use 3508 */
/* the deceleration ratio of chassis motor */
#define MOTOR_DECELE_RATIO_3508 (1.0f / 14.0f)
#define MOTOR_DECELE_RATIO_2006 (1.0f / 36.0f)
/* single 3508 motor maximum speed, unit is rpm */
#define MAX_WHEEL_RPM_3508 9600  //9550rpm = 4000mm/s
#define MAX_WHEEL_RPM_2006 18100 //18095rpm = 4000mm/s
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 4000
#define MAX_CHASSIS_VY_SPEED 4000
/* chassis maximum rotation speed, unit is	rad/s */
#define MAX_CHASSIS_VW_SPEED 8 * 3.1415926f

#define MOTOR_ENCODER_ACCURACY 8192.0f

#define STEERING_ENCODER_OFFSET_RF 121
#define STEERING_ENCODER_OFFSET_LF -64
#define STEERING_ENCODER_OFFSET_LB 92
#define STEERING_ENCODER_OFFSET_RB -124
/****************************************/

class FourSteeringWheel
{
private:
    /* data */
    struct steering_structure
    {
        float wheel_perimeter; /* the perimeter(mm) of wheel */
        float wheeltrack;      /* wheel track distance(mm) */
        float wheelbase;       /* wheelbase distance(mm) */
        float rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */
        float rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */
        float W_Cdistance;
    } steeringPara;
	
	vec4f wheelAngleError;
	float maxError;
	
public:
    FourSteeringWheel();
    struct SteeringSpeed
    {
        float vx; // +forward	-back
        float vy; // +left	-right
        float vw; // +anticlockwise	-clockwise
				float linearVelocity;//��ת�������ٶ� ��λ��mm/s
				float lineraVelocity2; //��תȫ�������ٶ�
    } steeringSpeed;
    struct steering
    {
        float angle,angle_last;     //��   0---360
        float speed_rpm; //rpm
        float speed;
        float angleFdb;
		int round; 
    };
    struct OutputData
    {
        vec4f speed_rpm;	//��������ֵ��ת��
        vec4f angle;		//������Ķ����Ƕ�
				vec4f meca_speed;
    } outputData;
	struct FeedbackData
	{
		vec4f feedbackAngle;//���ݶ�����ϵת����ĽǶ�
		vec4f feedbackSpeed;//��������ĵ����ٶ�
	}feedbackData;
	
    struct SteeringWheels
    {
        struct steering RF; //0
        struct steering LF; //1
        struct steering LB; //2
        struct steering RB; //3
			 
			  struct steering quan_RF;
				struct steering quan_LF;
				struct steering quan_LB;
				struct steering quan_RB;
    } wheels;
    /**
     * @brief ���ֽ���.F:forword; B:backword; L:left; R:right
     * @details ����every wheel speed(rpm) ��������every wheel angle(��) ��������
     * @param[in] speedDef ccx=+vx(mm/s)  ccy=+vy(mm/s)  ccw=+vw(deg/s) x�����ٶ�(��������Ϊ��) y�����ٶ�(��ͷ����Ϊ��) ��ת�ٶ�(��ʱ����תΪ��)
     * @param[in] wheelFeedback ccx=+vx(mm/s)  ccy=+vy(mm/s)  ccw=+vw(deg/s) x�����ٶ�(��������Ϊ��) y�����ٶ�(��ͷ����Ϊ��) ��ת�ٶ�(��ʱ����תΪ��)
	 * @note  1=FR 2=FL 3=BL 4=BR  ��������Ϊ��ʱ��˳��,ͬ����ϵ����
     */
    void fourSteeringWheel_calculate(vec3f speedDef, vec4f wheelFeedback);
    /**
     * �жϵ�ǰ�ĸ����ֽǶ��Ƿ񵽴��趨�Ƕ�����
     * @param[in] angleFeedback   ����Ƕȷ���ֵ
     * @return {*} true/false
     */
    bool judgeWheelPrepared(vec4f angleFeedback);
	/**
	 * @brief ���ݷ���ֵ�������ǰ�����ٶȣ��������ջ�������
	 */
	void nowSpeed_calculate(vec4f steeringFeedback, vec4f wheelFeedback);
	
};

#endif