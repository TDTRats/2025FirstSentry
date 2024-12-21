/******************************
File name: TDT_Alg\src\imu.cpp
Description: 陀螺仪姿态解算算法
class:ImuCalc
	——————————————————————————————————————————————————————————————————————————
	Init :accScaleFactor gyroScaleFactor accValueFector gyroDpsFector
	——————————————————————————————————————————————————————————————————————————
	Use  : AHRS_data.Angle  My_data.Angle
	——————————————————————————————————————————————————————————————————————————
	virtual void init()  imu初始化,异常复位不会初始化imu，主动校准复位会全部重新初始化
	——————————————————————————————————————————————————————————————————————————
	virtual void GetOriginData(); 更新orgin数据重写函数
	——————————————————————————————————————————————————————————————————————————
Function:
	void getOffset();   校准并保存
	——————————————————————————————————————————————————————————————————————————
	uint64_t TDT_IMU_update();   子类更新数据，并运算 
	——————————————————————————————————————————————————————————————————————————
	void TDT_ImuInit();    子类陀螺仪初始化，陀螺仪信息，角度初始化
	——————————————————————————————————————————————————————————————————————————
state :
	FLASH读取失败 ：led慢闪并卡在初始化
	——————————————————————————————————————————————————————————————————————————
	FLASH保存 ：led常亮
	——————————————————————————————————————————————————————————————————————————
	正在校准 ：led快闪
	——————————————————————————————————————————————————————————————————————————
	
	*/
#include "imu.h"
#include "my_math.h"
#include "math.h"
#include "TimeMatch.h"
#include "flash_var.h"
#include "iwdg.h"
#include "led_task.h"
/**
 * @ingroup TDT_ALG
 * @defgroup IMU_ALG 陀螺仪解算算法
 * @brief 该类提供了传统的互补滤波算法，以及大疆开源的AHRS算法库的调用，同时作为父类供各种陀螺仪继承
 * @{
 */

#define MAX_IMU 3  //最大imu数量
using namespace imu;

///@note 二阶互补滤波系数，规律：基本时间常数tau得到基本系数a，Kp=2*a，Ki=a^2;
#define Kp 0.6f ///<proportional gain governs rate of convergence to accelerometer/magnetometer
///@note 二阶互补滤波系数，规律：基本时间常数tau得到基本系数a，Kp=2*a，Ki=a^2;
#define Ki 0.1f ///<integral gain governs rate of convergence of gyroscope biases

/** @}*/
int ImuCalc::objectCount = 0; // 定义并初始化静态成员变量

volatile float persistent_AHRS_yawRecord[MAX_IMU] __attribute__((section(".bss.NO_INIT")));
volatile float persistent_MY_yawRecord[MAX_IMU] __attribute__((section(".bss.NO_INIT")));

ImuCalc::ImuCalc() : accValueFector(1), gyroDpsFector(1)
{
	memset(&My_data, 0, sizeof(My_data));
	memset(&AHRS_data, 0, sizeof(AHRS_data));
	memset(&gyro,0,sizeof(gyro));
	memset(&acc,0,sizeof(acc));
	float defaultScaleFactor[3][3] = {DEFAULT_BOARD_INSTALL_SPIN_MATRIX};
	setGyroScaleFactor(defaultScaleFactor);
	setAccScaleFactor(defaultScaleFactor);
	index = objectCount; 
	objectCount++;
}

/**
* @brief ImuCal 角度初始化，读取flash，若读取失败自动校准重启
  * @warning 需要先获取加速度数据
  */
void ImuCalc::TDT_ImuInit()
{
	init();
	if(IFlash.read() !=0)    //FLASH读取失败
	FlashError_Handle();
	GyroAccUpDate(acc,gyro);
	
#if !USE_AHRS_LIB || (USE_AHRS_LIB && COMPARE_SELF_LIB)
	
	My_DaTaInit(My_data,acc.accValue);
	
#endif
#if USE_AHRS_LIB
	AHRS_dataInit(AHRS_data,acc.accValue.data,AHRS_data.mag);
#endif
}


/**
* @brief 读取数据并解算角度
  * 
  */
uint64_t ImuCalc::TDT_IMU_update()
{
	float half_T = cycle.getCycleT() / 2.0f;
	uint64_t readImuTime = getSysTimeUs();
	GyroAccUpDate(acc,gyro);
	
#if USE_AHRS_LIB
	
	UpDateAHRS_data(AHRS_data,gyro.radps,acc.accValue,half_T);
	
	#if !COMPARE_SELF_LIB
	memcpy(&Angle, &AHRS_data.Angle, sizeof(Angle));
	#endif
#endif
	
	//使用库并且比较
#if !USE_AHRS_LIB || (USE_AHRS_LIB && COMPARE_SELF_LIB)
	
		My_DaTaUpdate(My_data,gyro.radps,acc.accValue,half_T);
		
#endif

	return readImuTime;
}


/**
* @brief 存入Flash时led常亮 最后复位
  * 
  */
void ImuCalc::FlashSave()   
{
	ledtask.SetNow(LedES_ConstantLight);
	//看门狗复位时间1.5s
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	auto oldIWDG_P = IWDG->PR;
	auto oldIWDG_RL = IWDG->RLR;
	IWDG_SetPrescaler(IWDG_Prescaler_64); //设置IWDG分频系数
	IWDG_SetReload(750);				  //设置IWDG装载值
	IFlash.save();
	//恢复看门狗时间
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	IWDG_SetPrescaler(oldIWDG_P);				  //设置IWDG分频系数
	IWDG_SetReload(oldIWDG_RL);					  //设置IWDG装载值
	iwdgFeed();									  //reload
	__set_FAULTMASK(1);							  //关闭所有中断
	NVIC_SystemReset();							  //复位
	while (1)
	{
	} //仅等待复位
}

/**
	* @brief 不断重新校准并存入flash，led慢闪
  * 
  */
void ImuCalc::FlashError_Handle()  
{
		while (!OffSetRun())
		{
			ledtask.SetNow(LedES_BlinkSlow);
			laser.stateShow(LedES_BlinkSlow);
	
			GyroAccUpDate(acc,gyro);
			cycle.getCycleT();
			delayMs(2);
		}
		FlashSave();
}

/**
* @brief 不断重新校准并存入flash,led快闪
  * 
  */
extern volatile u8 bmi_OK;
void ImuCalc::getOffset()
{
	bmi_OK=0;
	persistent_AHRS_yawRecord[index]=0;
	persistent_MY_yawRecord[index]=0;
	while (!OffSetRun())
	{
		ledtask.SetNow(LedES_BlinkFast);  //状态灯快闪
		laser.stateShow(LedES_BlinkFast);
	
		GyroAccUpDate(acc,gyro);
		cycle.getCycleT();
		iwdgFeed();
		delayMs(2);
	}
	
	FlashSave();	
}


bool ImuCalc::OffSetRun()
{
	static uint16_t runningTimes;
	runningTimes++;
	// 更新角速度数据
	for (int i = 0; i < 3; i++) {
		gyro.dps.data[i] = gyro.origin.data[i] * gyroDpsFector;
	}
	
	// 初始化动态范围和累加器
	if (runningTimes == 0)
	{
		for (int i = 0; i < 3; i++) {
			gyro.dynamicSum.data[i] = 0;
			gyro.offset_max.data[i] = -FLT_MAX;
			gyro.offset_min.data[i] = FLT_MAX;
		}
	}

	// 更新最大值、最小值和动态累加
	for (int i = 0; i < 3; i++) 
	{
		gyro.offset_max.data[i] = std::max(gyro.offset_max.data[i], gyro.dps.data[i]);
		gyro.offset_min.data[i] = std::min(gyro.offset_min.data[i], gyro.dps.data[i]);
		gyro.dynamicSum.data[i] += gyro.origin.data[i];
	}
	
	// 检查动态范围是否超过阈值 超过则重新校准
	for (int i = 0; i < 3; i++) 
	{
		if (gyro.offset_max.data[i] - gyro.offset_min.data[i] > OFFSET_THRESHOLD) 
			runningTimes = 0;
	}

	if (runningTimes >= OFFSET_TIME)
	{
		gyro.offset.data[x] = (float)(gyro.dynamicSum.data[x]) / runningTimes;
		gyro.offset.data[y] = (float)(gyro.dynamicSum.data[y]) / runningTimes;
		gyro.offset.data[z] = (float)(gyro.dynamicSum.data[z]) / runningTimes;
		runningTimes = 0;
		return true;
	}
	return false;
}


void ImuCalc::ImuCrossRoundHandle(eulerAngle &LastAngle, eulerAngle &nowAngle, eulerAngle &Angle, angleRound &round)
{
	//过圈
	if (nowAngle.yaw - LastAngle.yaw > 180)
	{
		round.roundYaw--;
	}
	else if (nowAngle.yaw - LastAngle.yaw < -180)
	{
		round.roundYaw++;
	}

	Angle.yaw = round.roundYaw * 360 + nowAngle.yaw;
	Angle.pitch = nowAngle.pitch;
	Angle.roll = nowAngle.roll;
}

void ImuCalc::My_DaTaInit(struct My_Data &my_Data,vec3f a)
{
	eulerAngle initial; //初始欧拉角
	
	initial.yaw = 0;
	initial.pitch = atan2(-a.data[0], a.data[2]) * RAD_TO_ANGLE;
	initial.roll = atan2(a.data[1], a.data[2]) * RAD_TO_ANGLE;
	
	//三角函数运算较为耗时，现行运算存入缓存
	float dataBuf[2][3] = {cosf(initial.yaw / (2 * RAD_TO_ANGLE)), cosf(initial.pitch / (2 * RAD_TO_ANGLE)), cosf(initial.roll / (2 * RAD_TO_ANGLE)),
						   sinf(initial.yaw / (2 * RAD_TO_ANGLE)), sinf(initial.pitch / (2 * RAD_TO_ANGLE)), sinf(initial.roll / (2 * RAD_TO_ANGLE))};
	
	insQuat& quaternion=my_Data.quaternion;
	//更新四元数
	quaternion.q0 = dataBuf[0][0] * dataBuf[0][1] * dataBuf[0][2] + dataBuf[1][0] * dataBuf[1][1] * dataBuf[1][2];
	quaternion.q1 = dataBuf[0][0] * dataBuf[0][1] * dataBuf[1][2] - dataBuf[1][0] * dataBuf[1][1] * dataBuf[0][2];
	quaternion.q2 = dataBuf[0][0] * dataBuf[1][1] * dataBuf[0][2] + dataBuf[1][0] * dataBuf[0][1] * dataBuf[1][2];
	quaternion.q3 = dataBuf[1][0] * dataBuf[0][1] * dataBuf[0][2] - dataBuf[0][0] * dataBuf[1][1] * dataBuf[1][2];

	//规范化最新四元数（归一化）
	float norm = sqrt(quaternion.q0 * quaternion.q0 + quaternion.q1 * quaternion.q1 +
					  quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3);
	quaternion.q0 = quaternion.q0 / norm;
	quaternion.q1 = quaternion.q1 / norm;
	quaternion.q2 = quaternion.q2 / norm;
	quaternion.q3 = quaternion.q3 / norm;
	
	
	//直接更新欧拉角
	my_Data.Angle.roll = atan2(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1, -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * RAD_TO_ANGLE;
	my_Data.Angle.pitch = asin(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * RAD_TO_ANGLE;

	my_Data.round.roundYaw = 0;
	my_Data.nowAngle= my_Data.Angle;
	my_Data.persistent_YawOffset=persistent_MY_yawRecord[index];
	
}

void ImuCalc::My_DaTaUpdate(struct My_Data &my_Data ,vec3f g,vec3f a,float half_T)
{
	
	gyro.radps=accInjectgyro(My_data,gyro.radps,acc.accValue,half_T);
	quaterForAngle(My_data,gyro.radps,half_T);
	
}


/**
* @brief 互补滤波
  * 
  */
vec3f ImuCalc::accInjectgyro(struct My_Data &my_Data ,vec3f g,vec3f a,float half_T)
{
	float ax = a.data[x], ay = a.data[y], az = a.data[z];
  float gx = g.data[x], gy = g.data[y], gz = g.data[z];
	
	// 更新纯积分角度，可用作对比
	my_Data.AngleNoZero.pitch += gy * 2 * half_T;
	my_Data.AngleNoZero.roll += gx * 2 * half_T;
	my_Data.AngleNoZero.yaw += gz * 2 * half_T;
	
	//acc数据归一化
	float norm = my_sqrt(ax * ax + ay * ay + az * az);
	float ex, ey, ez;
	
	if (norm)
	{
		 // 归一化
      ax /= norm; ay /= norm; az /= norm;
		
			float vx, vy, vz; //(r系到b系的第三列)
			// estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
			vx = 2 * (my_Data.quaternion.q1 * my_Data.quaternion.q3 - my_Data.quaternion.q0 * my_Data.quaternion.q2); //四元素中xyz的表示
			vy = 2 * (my_Data.quaternion.q0 * my_Data.quaternion.q1 + my_Data.quaternion.q2 * my_Data.quaternion.q3);
			vz = 1 - 2 * (my_Data.quaternion.q1 * my_Data.quaternion.q1 + my_Data.quaternion.q2 * my_Data.quaternion.q2);

			// error is sum of cross product between reference direction of fields and direction measured by sensors
			ex = (ay * vz - az * vy); //向量外积在相减得到差分就是误差
			ey = (az * vx - ax * vz);
			ez = (ax * vy - ay * vx);

			my_Data.err.exInt += ex * Ki * 2 * half_T; //对误差进行积分
			my_Data.err.eyInt += ey * Ki * 2 * half_T;
			my_Data.err.ezInt += ez * Ki * 2 * half_T;

			// 积分限幅
			my_Data.err.exInt = LIMIT(my_Data.err.exInt, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
			my_Data.err.eyInt = LIMIT(my_Data.err.eyInt, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
			my_Data.err.ezInt = LIMIT(my_Data.err.ezInt, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);

			// adjusted gyroscope measurements
			gx = gx + Kp * (ex + my_Data.err.exInt);
			gy = gy + Kp * (ey + my_Data.err.eyInt);
			gz = gz + Kp * (ez + my_Data.err.ezInt);
	}
		return {gx, gy, gz};
	
}

/**
* @brief 四元数的更新以及解算角度
  * 
  */
void ImuCalc::quaterForAngle(struct My_Data &my_Data,vec3f g,float half_T)
{
		float gx = g.data[0], gy = g.data[1], gz = g.data[2];
		insQuat& quaternion=my_Data.quaternion;
		// integrate quaternion rate and normalise						   //四元素的更新
		insQuat tmp_q =
		{	
		 quaternion.q0 + (-quaternion.q1 * gx - quaternion.q2 * gy - quaternion.q3 * gz) * half_T,
		 quaternion.q1 + (quaternion.q0 * gx + quaternion.q2 * gz - quaternion.q3 * gy) * half_T,
		 quaternion.q2 + (quaternion.q0 * gy - quaternion.q1 * gz + quaternion.q3 * gx) * half_T,
		 quaternion.q3 + (quaternion.q0 * gz + quaternion.q1 * gy - quaternion.q2 * gx) * half_T
		};

		quaternion = tmp_q;

	  //四元素解算角度
		float norm = my_sqrt(quaternion.q0 * quaternion.q0 + quaternion.q1 * quaternion.q1 + quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3);
		quaternion.q0 = quaternion.q0 / norm;
		quaternion.q1 = quaternion.q1 / norm;
		quaternion.q2 = quaternion.q2 / norm;
		quaternion.q3 = quaternion.q3 / norm;
		
		memcpy(&my_Data.LastAngle, &my_Data.nowAngle, sizeof(my_Data.Angle));

		my_Data.nowAngle.yaw = fast_atan2(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3, -2 * quaternion.q2 * quaternion.q2 - 2 * quaternion.q3 * quaternion.q3 + 1) * 57.295780f;
		my_Data.nowAngle.roll = fast_atan2(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1, -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * 57.295780f;
		my_Data.nowAngle.pitch = asin(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * 57.295780f;
		
		ImuCrossRoundHandle(my_Data.LastAngle,my_Data.nowAngle,my_Data.Angle,my_Data.round);
		my_Data.persistent_Yaw=my_Data.Angle.yaw+my_Data.persistent_YawOffset;
		persistent_MY_yawRecord[index]= my_Data.persistent_Yaw ;   //时刻记录此时的yaw到sram，便于之后重启校0使用。
}


// 输出的地址，输入三轴加速度，三轴磁力计
void ImuCalc::AHRS_dataInit(struct AHRS_DaTa &AHRS_Data,float accLibIn[3],float mag[3])
{
	AHRS_init(AHRS_Data.ins_quat,accLibIn, mag);
	float angleLibOut[3] = {0};
	//航资参考系统获取角度，具体声明参考ahrs_lib.h
	get_angle(AHRS_Data.ins_quat, angleLibOut, angleLibOut + 1, angleLibOut + 2);
	//单位换算
	AHRS_Data.nowAngle.roll = angleLibOut[2] * RAD_TO_ANGLE;
	AHRS_Data.nowAngle.pitch = angleLibOut[1] * RAD_TO_ANGLE;
	AHRS_Data.nowAngle.yaw = angleLibOut[0] * RAD_TO_ANGLE;
	
	AHRS_Data.persistent_YawOffset=persistent_AHRS_yawRecord[index];
}

void ImuCalc::UpDateAHRS_data(struct AHRS_DaTa &AHRS_Data,vec3f &gyroLibIn,vec3f &accLibIn,float half_T)
{
	float angleLibOut[3] = {0};
	//航资参考系统数据更新，具体声明参考ahrs_lib.h
	AHRS_update(AHRS_Data.ins_quat, half_T * 2, gyroLibIn.data, accLibIn.data, AHRS_Data.mag);
	//记录上一次的值
	memcpy(&AHRS_Data.LastAngle, &AHRS_Data.nowAngle, sizeof(AHRS_Data.nowAngle));
	
	
	//航资参考系统获取角度，具体声明参考ahrs_lib.h
	get_angle(AHRS_Data.ins_quat, angleLibOut, angleLibOut + 1, angleLibOut + 2);
	//单位换算
	AHRS_Data.nowAngle.roll = angleLibOut[2] * RAD_TO_ANGLE;
	AHRS_Data.nowAngle.pitch = angleLibOut[1] * RAD_TO_ANGLE;
	AHRS_Data.nowAngle.yaw = angleLibOut[0] * RAD_TO_ANGLE;
	//过圈处理
	ImuCrossRoundHandle(AHRS_Data.LastAngle, AHRS_Data.nowAngle, AHRS_Data.Angle, AHRS_Data.round);
	AHRS_Data.persistent_Yaw=AHRS_Data.Angle.yaw+AHRS_Data.persistent_YawOffset;
	persistent_AHRS_yawRecord[index] = AHRS_Data.persistent_Yaw ;   //时刻记录此时的yaw到sram，便于之后重启校0使用。
}

void ImuCalc::GyroAccUpDate(accdata &Acc,gyrodata &Gyro)
{
	GetOriginData();
	OrginToCali(Gyro.calibration,Acc.calibration,Gyro.origin,Acc.origin);
	UnitConvert(Acc.accValue,Gyro.dps,Gyro.radps,Acc.calibration,Gyro.calibration); //进行单位换算
	
}
void ImuCalc::UnitConvert(vec3f &accValue,vec3f &dps,vec3f &radps,vec3f &acalibration,vec3f &gcalibration)
{
	 for (int i = 0; i < 3; ++i) 
	{
     accValue.data[i] = acalibration.data[i] * accValueFector;
		 dps.data[i] = gcalibration.data[i] * gyroDpsFector;
		 radps.data[i] = gyro.dps.data[i] * ANGLE_TO_RAD;
  }

}

void ImuCalc::OrginToCali(vec3f &gcalibration,vec3f &acalibration,vec3int16 &gorigin,vec3int16 &aorigin)
{
	for (uint8_t i = 0; i < 3; i++)
	{
		//角速度坐标转换
		gcalibration.data[i] =
			(gorigin.data[0] - gyro.offset.data[0]) * gyroScaleFactor.data[i][0] +
			(gorigin.data[1] - gyro.offset.data[1]) * gyroScaleFactor.data[i][1] +
			(gorigin.data[2] - gyro.offset.data[2]) * gyroScaleFactor.data[i][2];

		//加速度坐标转换
		acalibration.data[i] =
			(aorigin.data[0] - acc.offset.data[0]) * accScaleFactor.data[i][0] +
			(aorigin.data[1] - acc.offset.data[1]) * accScaleFactor.data[i][1] +
			(aorigin.data[2] - acc.offset.data[2]) * accScaleFactor.data[i][2];
	}
}


/**
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19/11.19
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.19 首次记录
	——————————————————————————————————————————————————————————————————————————
****************************  */
