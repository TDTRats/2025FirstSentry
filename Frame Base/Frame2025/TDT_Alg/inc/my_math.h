/*****************************************************************************
File name: TDT_Alg\src\my_math.h
Description: 快速数学计算
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.16 肖银河-改写my_deathzoom函数-解决遥控器最大值会小于660问题
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#include "board.h"
/**
 * @ingroup TDT_ALG
 * @defgroup TDT_MY_MATH TDT_MY_MATH
 * @brief 该库包含了各种常用数学计算公式的快速解法
 * @{
 */

///最小线性二乘法拟合线性方程
class Linear
{
private:
	uint16_t getValueCnt;
	double l[4];

public:
	float kA;
	float kB;
	Linear()
	{
		getValueCnt = 1;
		memset(l, 0, sizeof(l));
	}
	void clear(void)
	{
		getValueCnt = 1;
		memset(l, 0, sizeof(l));
	}
	void linearRegression(float valueX, float valueY, float valueNum, float *_aResult = 0, float *_bResult = 0, u8 resetFlag = 0)
	{
		if (resetFlag == 1)
		{
			Linear();
		}
		if (++getValueCnt < valueNum)
		{
			l[0] += valueX;
			l[1] += valueY;
			l[2] += valueX * valueY;
			l[3] += valueX * valueX;
		}
		else
		{
			kA = (valueNum * l[2] - l[0] * l[1]) / (valueNum * l[3] - l[0] * l[0]);
			kB = l[1] / valueNum - kA * l[0] / valueNum;
			if (_aResult != 0 && _bResult != 0)
			{
				*_aResult = kA;
				*_bResult = kB;
			}
		}
	}
};


/// 取绝对值
#define ABS(x) ((x) > 0 ? (x) : -(x))
/// 输出最接近 (min, max) 的x的值
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
/// 输出a与b的最小值
#define MIN(a, b) ((a) < (b) ? (a) : (b))
/// 输出a与b的最大值
#define MAX(a, b) ((a) > (b) ? (a) : (b))

///tan列表的最小值
#define TAN_MAP_RES 0.003921569f /* (smallest non-zero value in table) */
///rad/°的算术值
#define RAD_PER_DEG 0.017453293f
///tan列表的大小
#define TAN_MAP_SIZE 256
#define MY_PPPIII 3.14159f
#define MY_PPPIII_HALF 1.570796f

///快速正切计算
float my_atan_angle(float xx, float yy);
float my_atan_rad(float xx, float yy);
float fast_atan2(float yy, float xx);
///平方
float my_pow(float a);
///快速开方
float my_sqrt(float number);
///快速正弦
double my_sin_angle(float angle);
double my_sin_rad(double rad);
///快速余弦
double my_cos_angle(float angle);
double my_cos_rad(double rad);

///死区，绝对值小于zoom的xx会返回0，否则返回xx
float my_deathzoom(float xx, float zoom);
///6值之中返回最大的
float Math_Max(float a, float b, float c, float d, float e, float f);
/** @brief 转速转换
 * @{ */
float rpmToDps(float rpm);
float dpsToRpm(float dps);
float rpmToRadps(float rpm);
float RadpsToRpm(float radps);
/** @} */

float Ramp_function_cos(double Input, double min, double max);
float Ramp_function_sin(double Input, double min, double max);
float differangle_caculate(float angle1, float angle2);
float normalizeAngle(float angle);

/** @} */

#endif
