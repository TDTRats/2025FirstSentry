#ifndef __MIXWHEEL_H
#define __MIXWHEEL_H

#include "board.h"
#include "my_math.h"

#define WHEELBASE  0.5f//轴距
#define STEER_WHEELTRACK  0.5f//轮距
#define ROTATE_X_OFFSET  0.0f//云台中心补偿
#define ROTATE_Y_OFFSET  0.0f//云台中心补偿
#define PERIMETER  0.5f//轮周长
#define MOTOR_DECELE_RATIO_3508  (1.0f/19.0f) //3508减速比
#define QUAN_PERIMETER  0.5f//全向轮周长
#define QUAN_WHEELTRACK  0.5f
#define MAX_CHASSIS_VX_SPEED  0.5f
#define MAX_CHASSIS_VY_SPEED  0.5f
#define MAX_CHASSIS_VW_SPEED  0.5f
#define DEGREE_TO_RADIAN  0.0174532925199f
#define RADIAN_TO_DEGREE  57.295779513082f
#define WHEEL_CAN_BACK  0.5f
#define L 1.0f //轮距
#define RATIO 5

class mixWheel
{
public:
    mixWheel();
    void mixWheel_calculate(vec3f speedDef);
    //void nowSpeed_calculate(vec4f steeringFeedback, vec2f wheelFeedback);
    vec3f speedNow;//当前反解算的实际速度
    vec2f steerAngleNow;//当前的舵角
    vec4f wheelSpeed;//设计的轮速
    vec2f steerAngle;//设计的舵角
};


#endif