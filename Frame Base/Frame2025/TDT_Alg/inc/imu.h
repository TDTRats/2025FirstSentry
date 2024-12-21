/*****************************************************************************
File name: TDT_Alg\src\imu.h
Description: 陀螺仪姿态解算算法
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19/11.19
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.19 首次记录
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _IMU_H
#define _IMU_H

#include "board.h"
#include "ahrs_lib.h"
#include "cycle.h"

/**
 * @addtogroup IMU_ALG 
 * @{
 */
#define USE_AHRS_LIB 1                ///<使能AHRS航资参考系统
#define COMPARE_SELF_LIB 1              ///<若使能则将AHRS输出到另一个数组上(AHRS_data.nowAngle)
#define OFFSET_THRESHOLD 300
#define OFFSET_TIME 1000
///默认板载旋转矩阵 单位矩阵
///@image html xyzPYR.jpg "陀螺仪正安装时的角度" width=100%
#define DEFAULT_BOARD_INSTALL_SPIN_MATRIX \
    {1.0f, 0.0f, 0.0f},                   \
    {0.0f, 1.0f, 0.0f},               \
    {0.0f, 0.0f, 1.0f}                 \

///陀螺仪互补滤波积分限幅
#define IMU_INTEGRAL_LIM (2.0f * ANGLE_TO_RAD)

///弧度转角度
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.324841f
#endif

///角度转弧度
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329f
#endif


///陀螺仪部分常量索引的命名空间
namespace imu
{

    const uint8_t A_X = 0;
    const uint8_t A_Y = 1;
    const uint8_t A_Z = 2;
    const uint8_t G_X = 3;
    const uint8_t G_Y = 4;
    const uint8_t G_Z = 5;

    const uint8_t x = 0;
    const uint8_t y = 1;
    const uint8_t z = 2;
}

///加速度数据结构体
typedef struct _accdata
{
    vec3int16 origin;  ///<原始值
    vec3f offset_max;  ///<零偏值最大值
    vec3f offset_min;  ///<零偏值最小值
    vec3f offset;      ///<零偏值
    vec3f calibration; ///<校准值
    vec3f filter;      ///<滑动平均滤波值
    vec3f accValue;    ///<加速度值，单位：m/s²
} accdata;

///角速度数据结构体
typedef struct _gyrodata
{
    vec3int16 origin;  ///<原始值
    vec3f offset_max;  ///<零偏值最大值
    vec3f offset_min;  ///<零偏值最小值
    vec3f offset;      ///<零偏值
    vec3f calibration; ///<校准值
    vec3f filter;      ///<滑动平均滤波值
    vec3f dps;         ///<度每秒
    vec3f radps;       ///<弧度每秒
    vec3f dynamicSum;  ///<校准时求和计算
} gyrodata;

///三轴欧拉角角度
typedef struct _eulerAngle
{
    float pitch;
    float roll;
    float yaw;
} eulerAngle;

///三轴欧拉角圈数
typedef struct
{
    int16_t roundYaw;
    int16_t roundPitch;
    int16_t roundRoll;
} angleRound;

///四元数结构体
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} insQuat;

///互补滤波算法——积分项
typedef struct
{
    float exInt;
    float eyInt;
    float ezInt;
} eInt;

///IMU解算类
struct ImuCalc
{
protected:
		///加速度旋转矩阵，将加速度统一成板载方向，使输出的PYR体现真实意义
    mat3f accScaleFactor;
    ///角度旋转矩阵，将角速度统一成板载方向，使输出的PYR体现真实意义
    mat3f gyroScaleFactor;
		///加速度转换单位
    float accValueFector=1;
    ///角速度转换单位
    float gyroDpsFector=1;
		
		
    int index;              // 每个对象的索引
public:
		static int objectCount; // 静态成员变量，跟踪对象数量
	struct My_Data
	{
		angleRound round;   ///三轴圈数
		insQuat quaternion; ///四元数
    eInt err;           /// scaled integral error
		eulerAngle LastAngle; ///<上一次不带过圈的角度
		eulerAngle nowAngle;  ///<不带过圈的角度
		///带过圈的角度
		///@warning 由于欧拉角定义，仅YAW带过圈
		eulerAngle Angle;
		eulerAngle AngleNoZero; ///<一直是无互补滤波的角度
		float persistent_YawOffset;   //初始化时，自动读取sram中的角度作为零点
		float persistent_Yaw;   //复位后也不会丢失的yaw角            //若无哨兵定位需求，慎用
	}My_data;

		accdata acc;   ///<加速度解算数据
    gyrodata gyro; ///<角速度解算数据
	
    u8 imuValid = 0; ///<此Imu初始化完成以及运行正常标志位

		struct  AHRS_DaTa
    {
      float mag[3];         ///<磁力计,并未用到，仅定义
      float ins_quat[4];    ///<四元数
      eulerAngle LastAngle; ///<上一次不带过圈的角度
      eulerAngle nowAngle;  ///<不带过圈的角度
      ///带过圈的角度
      ///@warning 由于欧拉角定义，仅YAW带过圈
      eulerAngle Angle;
			angleRound round;   ///<yaw轴圈数
			float persistent_YawOffset;   //初始化时，自动读取sram中的角度作为零点
			float persistent_Yaw;         //复位后也不会丢失的yaw角                      //若无哨兵定位需求，慎用
    } AHRS_data;
		
    ///ImuCalc构造器，进行圈数、四元数、偏差、加速度及角速度缩放单位、加速度及角速度旋转矩阵以及角度的初始化
    ImuCalc();
		
		 ///imu数据更新，包括获取值、零飘校准、数据旋转，互补滤波（AHRS）算法解算以及过圈处理
    ///@retval uint64_t 获取陀螺仪时的时间
    uint64_t TDT_IMU_update();

    ///陀螺仪的初始化。别的陀螺仪重写此函数建议包括陀螺仪初始化，并调用默认的构造函数( ImuCalc::init )
    ///@warning 默认仅进行周期始终的初始化
    virtual void init() { cycle.getCycleT(); };
		
    ///陀螺仪初始化，陀螺仪信息，角度初始化
    void TDT_ImuInit();

    ///过圈处理，获得总角度Angle
    void ImuCrossRoundHandle(eulerAngle &LastAngle, eulerAngle &nowAngle, eulerAngle &Angle, angleRound &round);
		

		//更新gyro acc
		void GyroAccUpDate(accdata &Acc,gyrodata &Gyro);
		virtual void GetOriginData()=0;     //子类重写
		void OrginToCali(vec3f &gcalibration,vec3f &acalibration,vec3int16 &gorigin,vec3int16 &aorigin);
		void UnitConvert(vec3f &accValue,vec3f &dps,vec3f &radps,vec3f &acalibration,vec3f &gcalibration);
		
		
		//更新AHRS_data
		void UpDateAHRS_data(struct AHRS_DaTa &AHRS_Data,vec3f &gyroLibIn,vec3f &accLibIn,float half_T);
		void AHRS_dataInit(struct AHRS_DaTa &AHRS_Data,float accLibIn[3],float mag[3]);
		
		//更新My_data
		void My_DaTaInit(struct My_Data &my_Data,vec3f a);
		void My_DaTaUpdate(struct My_Data &my_Data ,vec3f g,vec3f a,float half_T);
		void quaterForAngle(struct My_Data &my_Data,vec3f g,float half_T);
		vec3f accInjectgyro(struct My_Data &my_Data ,vec3f g,vec3f a,float half_T);
		
		//保存flash并重启
		void FlashSave(void);
		
		//不断重新校准并重启
		void FlashError_Handle(void);
	
    ///强制校准
    void getOffset();
		bool OffSetRun();
		
		
    ///周期时钟初始化，获取互补滤波和AHRS周期精确值
    Cycle cycle;

    ///设置角速度旋转矩阵
    inline void setGyroScaleFactor(float gyroScaleFactor[3][3]) { memcpy(&(this->gyroScaleFactor.data[0][0]), &(gyroScaleFactor[0][0]), sizeof(mat3f)); }
    ///设置加速度旋转矩阵
    inline void setAccScaleFactor(float accScaleFactor[3][3]) { memcpy(&(this->accScaleFactor.data[0][0]), &(accScaleFactor[0][0]), sizeof(mat3f)); }
		
		
};

/** @} */
#endif
