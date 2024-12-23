/******************************
File name: TDT_Alg\src\my_math.cpp
Description: 快速数学计算
function:
	——————————————————————————————————————————————————————————————————————————
	float my_atan_angle(float xx, float yy)    float my_atan_rad(float xx, float yy)
	——————————————————————————————————————————————————————————————————————————
	double my_sin_angle(float angle)				double my_sin_rad(double rad)
	——————————————————————————————————————————————————————————————————————————
	double my_cos_angle(float angle)				double my_cos_rad(double rad)
	——————————————————————————————————————————————————————————————————————————
	float my_sqrt(float number)
	——————————————————————————————————————————————————————————————————————————
	float my_pow(float a)
	——————————————————————————————————————————————————————————————————————————
	float my_deathzoom(float xx,float zoom)
	——————————————————————————————————————————————————————————————————————————
	float Math_Max(float a,float b,float c,float d,float e,float f)
	——————————————————————————————————————————————————————————————————————————
	float normalizeAngle(float angle) 
	——————————————————————————————————————————————————————————————————————————
	float differangle_caculate(float angle1, float angle2)
	——————————————————————————————————————————————————————————————————————————
	float rpmToDps(float rpm);
	float dpsToRpm(float dps);
	float rpmToRadps(float rpm);
	float RadpsToRpm(float radps);
	——————————————————————————————————————————————————————————————————————————

****************************  */
#include "my_math.h"
#include "math.h"

#define ONE_PI (3.14159265)

float fast_atan_table[257] =
	{
		0.000000e+00, 3.921549e-03, 7.842976e-03, 1.176416e-02,
		1.568499e-02, 1.960533e-02, 2.352507e-02, 2.744409e-02,
		3.136226e-02, 3.527947e-02, 3.919560e-02, 4.311053e-02,
		4.702413e-02, 5.093629e-02, 5.484690e-02, 5.875582e-02,
		6.266295e-02, 6.656816e-02, 7.047134e-02, 7.437238e-02,
		7.827114e-02, 8.216752e-02, 8.606141e-02, 8.995267e-02,
		9.384121e-02, 9.772691e-02, 1.016096e-01, 1.054893e-01,
		1.093658e-01, 1.132390e-01, 1.171087e-01, 1.209750e-01,
		1.248376e-01, 1.286965e-01, 1.325515e-01, 1.364026e-01,
		1.402496e-01, 1.440924e-01, 1.479310e-01, 1.517652e-01,
		1.555948e-01, 1.594199e-01, 1.632403e-01, 1.670559e-01,
		1.708665e-01, 1.746722e-01, 1.784728e-01, 1.822681e-01,
		1.860582e-01, 1.898428e-01, 1.936220e-01, 1.973956e-01,
		2.011634e-01, 2.049255e-01, 2.086818e-01, 2.124320e-01,
		2.161762e-01, 2.199143e-01, 2.236461e-01, 2.273716e-01,
		2.310907e-01, 2.348033e-01, 2.385093e-01, 2.422086e-01,
		2.459012e-01, 2.495869e-01, 2.532658e-01, 2.569376e-01,
		2.606024e-01, 2.642600e-01, 2.679104e-01, 2.715535e-01,
		2.751892e-01, 2.788175e-01, 2.824383e-01, 2.860514e-01,
		2.896569e-01, 2.932547e-01, 2.968447e-01, 3.004268e-01,
		3.040009e-01, 3.075671e-01, 3.111252e-01, 3.146752e-01,
		3.182170e-01, 3.217506e-01, 3.252758e-01, 3.287927e-01,
		3.323012e-01, 3.358012e-01, 3.392926e-01, 3.427755e-01,
		3.462497e-01, 3.497153e-01, 3.531721e-01, 3.566201e-01,
		3.600593e-01, 3.634896e-01, 3.669110e-01, 3.703234e-01,
		3.737268e-01, 3.771211e-01, 3.805064e-01, 3.838825e-01,
		3.872494e-01, 3.906070e-01, 3.939555e-01, 3.972946e-01,
		4.006244e-01, 4.039448e-01, 4.072558e-01, 4.105574e-01,
		4.138496e-01, 4.171322e-01, 4.204054e-01, 4.236689e-01,
		4.269229e-01, 4.301673e-01, 4.334021e-01, 4.366272e-01,
		4.398426e-01, 4.430483e-01, 4.462443e-01, 4.494306e-01,
		4.526070e-01, 4.557738e-01, 4.589307e-01, 4.620778e-01,
		4.652150e-01, 4.683424e-01, 4.714600e-01, 4.745676e-01,
		4.776654e-01, 4.807532e-01, 4.838312e-01, 4.868992e-01,
		4.899573e-01, 4.930055e-01, 4.960437e-01, 4.990719e-01,
		5.020902e-01, 5.050985e-01, 5.080968e-01, 5.110852e-01,
		5.140636e-01, 5.170320e-01, 5.199904e-01, 5.229388e-01,
		5.258772e-01, 5.288056e-01, 5.317241e-01, 5.346325e-01,
		5.375310e-01, 5.404195e-01, 5.432980e-01, 5.461666e-01,
		5.490251e-01, 5.518738e-01, 5.547124e-01, 5.575411e-01,
		5.603599e-01, 5.631687e-01, 5.659676e-01, 5.687566e-01,
		5.715357e-01, 5.743048e-01, 5.770641e-01, 5.798135e-01,
		5.825531e-01, 5.852828e-01, 5.880026e-01, 5.907126e-01,
		5.934128e-01, 5.961032e-01, 5.987839e-01, 6.014547e-01,
		6.041158e-01, 6.067672e-01, 6.094088e-01, 6.120407e-01,
		6.146630e-01, 6.172755e-01, 6.198784e-01, 6.224717e-01,
		6.250554e-01, 6.276294e-01, 6.301939e-01, 6.327488e-01,
		6.352942e-01, 6.378301e-01, 6.403565e-01, 6.428734e-01,
		6.453808e-01, 6.478788e-01, 6.503674e-01, 6.528466e-01,
		6.553165e-01, 6.577770e-01, 6.602282e-01, 6.626701e-01,
		6.651027e-01, 6.675261e-01, 6.699402e-01, 6.723452e-01,
		6.747409e-01, 6.771276e-01, 6.795051e-01, 6.818735e-01,
		6.842328e-01, 6.865831e-01, 6.889244e-01, 6.912567e-01,
		6.935800e-01, 6.958943e-01, 6.981998e-01, 7.004964e-01,
		7.027841e-01, 7.050630e-01, 7.073330e-01, 7.095943e-01,
		7.118469e-01, 7.140907e-01, 7.163258e-01, 7.185523e-01,
		7.207701e-01, 7.229794e-01, 7.251800e-01, 7.273721e-01,
		7.295557e-01, 7.317307e-01, 7.338974e-01, 7.360555e-01,
		7.382053e-01, 7.403467e-01, 7.424797e-01, 7.446045e-01,
		7.467209e-01, 7.488291e-01, 7.509291e-01, 7.530208e-01,
		7.551044e-01, 7.571798e-01, 7.592472e-01, 7.613064e-01,
		7.633576e-01, 7.654008e-01, 7.674360e-01, 7.694633e-01,
		7.714826e-01, 7.734940e-01, 7.754975e-01, 7.774932e-01,
		7.794811e-01, 7.814612e-01, 7.834335e-01, 7.853983e-01,
		7.853983e-01};

		
/*
	计算tan，并返回弧度值（-pi，pi]	
		
	@warning 不可以有x，y同时为零的情况，并且当落在x负半轴上时返回的时pi而不是-pi
*/
float fast_atan2(float yy, float xx)
{
	float x_abs = ABS(xx);
  float y_abs = ABS(yy);
  float zz, base_angle, angle;
  int index;
	float alpha;

	if(xx == 0.0f && yy == 0.0f)
		return 0;
	
	if (xx == 0.0f) 
    return (yy >= 0.0f) ? MY_PPPIII_HALF : -MY_PPPIII_HALF; // 处理xx为0的情况

	if (yy == 0.0f) 
    return (xx >= 0.0f) ?  0.0f :3.14159265358979323846 ; // 处理yy为0的情况
	
		/* 归一化到 +/- 45 度范围 */
		zz = (y_abs < x_abs) ? (y_abs / x_abs) : (x_abs / y_abs);
		
		/* 当比值接近表的分辨率时，角度最好用自变量本身近似 */
		if (zz < TAN_MAP_RES)
			base_angle = zz;
		else
		{
			/* 找到索引和插值 */
			alpha = zz * (float)TAN_MAP_SIZE -0.5f;
			index = (int)alpha;
			alpha -= (float)index;
		/* 根据象限确定基础角度，并根据象限加或减表中的值 */
			base_angle = fast_atan_table[index];
			base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
		}

		if (x_abs > y_abs)
		{
			if (xx > 0.0f)
        angle = (yy >= 0.0f) ? base_angle : -base_angle; //-45~45
			else
				angle = (yy >= 0.0f) ? (3.14159265358979323846 - base_angle) : (base_angle - 3.14159265358979323846); //135~180,-135~-180
		}
		else
		{ 
			if (yy >= 0.0f) 
				angle = (xx >= 0.0f) ? (1.57079632679489661923 - base_angle) : (1.57079632679489661923+base_angle); // 45 -> 135
			else 
				angle = (xx >= 0.0f) ? (-1.57079632679489661923+base_angle) : (-1.57079632679489661923-base_angle); // -45 -> -135 
		}

	return angle;

}


/*
	计算tan，并返回弧度值（-pi，pi]	
		
	@warning 不可以有x，y同时为零的情况，并且当落在x负半轴上时返回的时pi而不是-pi
*/
float my_atan_rad(float xx, float yy)
{
	return fast_atan2(yy, xx);
}

/*
	计算tan，并返回角度值（-180，180]	
		
	@warning 不可以有x，y同时为零的情况，并且当落在x负半轴上时返回的时180而不是-180
*/
float my_atan_angle(float xx, float yy)
{
	return fast_atan2(yy, xx)*57.295780490442;
}

//计算浮点数平方
float my_pow(float a)
{
	return a * a;
}

//快速平方根算法
//函数名：invSqrt(void)
//描述：求平方根的倒数
//该函数是经典的Carmack求平方根算法，效率极高，使用魔数0x5f375a86
float my_sqrt(float number)
{
	long i;
	float xx, yy;
	const float f = 1.5F;
	xx = number * 0.5F;
	yy = number;
	i = *(long *)&yy;
	i = 0x5f375a86 - (i >> 1);

	yy = *(float *)&i;
	yy = yy * (f - (xx * yy * yy));
	yy = yy * (f - (xx * yy * yy));
	return number * yy;
}

double mx_sin(double rad)
{
	double sine;
	if (rad < 0)
		sine = rad * (1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine * (-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f * (sine - 1) + 1);
	return sine;
}

double my_sin_rad(double rad)
{
	s8 flag = 1;

	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}

	return mx_sin(rad) * flag;
}

double my_cos_rad(double rad)
{
	s8 flag = 1;
	rad += ONE_PI / 2.0;

	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}

	return my_sin_rad(rad) * flag;
}

double my_sin_angle(float angle)
{
    // 将角度转换为弧度
    double rad = angle * (ONE_PI / 180.0);
    
    s8 flag = 1;

    // 将 rad 限制在 [0, π) 范围内
    if (rad >= ONE_PI)
    {
        rad -= ONE_PI;
        flag = -1;
    }

    return mx_sin(rad) * flag; // 调用 mx_sin 函数并返回结果
}

double my_cos_angle(float angle)
{
	double rad = angle * (ONE_PI / 180.0);
	s8 flag = 1;
	
	rad += ONE_PI / 2.0;

	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}

	return my_sin_rad(rad) * flag;
}


float my_deathzoom(float xx, float zoom)
{
	if (xx > 0)
	{
		if (xx < zoom)
		{
			xx = 0;
		}
	}
	else
	{
		if (xx > -zoom)
		{
			xx = 0;
		}
	}
	return (xx);
}


float Math_Max(float a, float b, float c, float d, float e, float f)
{
	int i;
	float data[6] = {a, b, c, d, e, f};
	float result = 0;
	for (i = 0; i < 6; i++)
		if (data[i] > result)
			result = data[i];
	return result;
}


float rpmToDps(float rpm)
{
	return rpm * 0.166667f;
}

float dpsToRpm(float dps)
{
	return dps * 6.0f;
}

float rpmToRadps(float rpm)
{
	return rpm * 9.549297f;
}

float RadpsToRpm(float radps)
{
	return radps * 0.10472f;
}

/**
 * @brief 0--1 增函数 小端斜率为0
 * @param[in] Input 输入值
 * @param[in] min 	 斜坡计算最小值
 * @param[in] max  斜坡计算最大值
 * @return float 输出结果
 */
float Ramp_function_sin(double Input, double min, double max) //TODO
{
	if (Input < min)
		return 0;

	if (Input >= max)
		return 1;

	return LIMIT((my_sin_rad(MY_PPPIII * 0.5f / (max - min) * (Input - min))), 0.0f, 1.0f);
}

/**
 * @brief 0--1 减函数 大端斜率为0
 * @param[in] Input 输入值
 * @param[in] min 	 斜坡计算最小值
 * @param[in] max  斜坡计算最大值
 * @return float 输出结果
 */
float Ramp_function_cos(double Input, double min, double max) //TODO
{
	if (Input < min)
		return 1;

	if (Input >= max)
		return 0;

	return LIMIT((my_cos_rad(MY_PPPIII * 0.5f / (max - min) * (Input - min))), 0.0f, 1.0f);
}

/**
**
 * @brief 把所有角度规范成-180到180 前闭后开
 * 
 * @return float 输出结果
 */
float normalizeAngle(float angle)   
{
    angle = fmod(angle, 360.0f);  // 规范化到 [0, 360)
    if (angle < 0) angle += 360.0f;  // 处理负角度
    if (angle >= 180) angle -= 360.0f;  // 调整到 [-180, 180)
    return angle;
}


/**
* @brief 计算两个角度的差角，永远是劣弧，最短路径 ,必须规范顺时针为负，逆时针为正
 * 
 * @return float 输出结果 angle1 减去 angle2
 */

float differangle_caculate(float angle1, float angle2)
{
	
	float normalized1 = normalizeAngle(angle1);
    float normalized2 = normalizeAngle(angle2);
    float difference = normalized1 - normalized2;
	
	if (difference > 180) difference -= 360;
    else if (difference < -180) difference += 360;

    return difference;
	
}

/*
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.16 肖银河-改写my_deathzoom函数-解决遥控器最大值会小于660问题
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*/