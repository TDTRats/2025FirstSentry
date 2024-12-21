#ifndef __MIXWHEEL_H
#define __MIXWHEEL_H

#include "board.h"
#include "my_math.h"

#define WHEELBASE  0.5f//���
#define STEER_WHEELTRACK  0.5f//�־�
#define ROTATE_X_OFFSET  0.0f//��̨���Ĳ���
#define ROTATE_Y_OFFSET  0.0f//��̨���Ĳ���
#define PERIMETER  0.5f//���ܳ�
#define MOTOR_DECELE_RATIO_3508  (1.0f/19.0f) //3508���ٱ�
#define QUAN_PERIMETER  0.5f//ȫ�����ܳ�
#define QUAN_WHEELTRACK  0.5f
#define MAX_CHASSIS_VX_SPEED  0.5f
#define MAX_CHASSIS_VY_SPEED  0.5f
#define MAX_CHASSIS_VW_SPEED  0.5f
#define DEGREE_TO_RADIAN  0.0174532925199f
#define RADIAN_TO_DEGREE  57.295779513082f
#define WHEEL_CAN_BACK  0.5f
#define L 1.0f //�־�
#define RATIO 5

class mixWheel
{
public:
    mixWheel();
    void mixWheel_calculate(vec3f speedDef);
    //void nowSpeed_calculate(vec4f steeringFeedback, vec2f wheelFeedback);
    vec3f speedNow;//��ǰ�������ʵ���ٶ�
    vec2f steerAngleNow;//��ǰ�Ķ��
    vec4f wheelSpeed;//��Ƶ�����
    vec2f steerAngle;//��ƵĶ��
};


#endif