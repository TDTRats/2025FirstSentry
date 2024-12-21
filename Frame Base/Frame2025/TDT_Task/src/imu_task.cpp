/******************************
File name: TDT_Task\src\imu_task.cpp
Description: 陀螺仪姿态解算任务
variable :
	——————————————————————————————————————————————————————————————————————————
	 boardImu     陀螺仪类
	——————————————————————————————————————————————————————————————————————————
	 boardImu->My_data   boardImu->AHRS_data     两种解算方法的角度值
	——————————————————————————————————————————————————————————————————————————
	 imutask.forceGetOffset 置1则校准陀螺仪并重启主控
	——————————————————————————————————————————————————————————————————————————
	 gyroScaleFactor[3][3]    accScaleFactor[3][3]  陀螺仪旋转矩阵
	——————————————————————————————————————————————————————————————————————————
	MPU6050_def   BMI088_def   Scha634_03_def  使能某个陀螺仪
	**/
#include "imu_task.h"
#include "mpu6050.h"
#include "scha634_03.h"
#include "bmi088.h"
#include "TimeMatch.h"
#include "cycle.h"
#include "flash_var.h"
extern TimeSimultaneity imuTimeMatch;
ImuCalc* boardImu;
ImuTask imutask;


ImuTask::ImuTask()
{
	status=taskStateRun;
	taskHz=_500hz;
}

/**
  * @brief 陀螺仪任务
  * @note 负责数据读取和解算
  */

void ImuTask::run()
{	
	if(imutask.forceGetOffset)
	{
			boardImu->getOffset();
	}
	/*读取*/
	uint64_t readImuTime = boardImu->TDT_IMU_update() ;
	imuTimeMatch.top(float(readImuTime) / 1e6f);

}

/**
  * @brief 陀螺仪初始化
  * @note 根据imu_task.h中宏定义初始化对应的陀螺仪
  */
void ImuTask::init()
{
	
#if MPU6050_def
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		boardImu = new Mpu6050(GPIOC, GPIO_Pin_2, GPIO_Pin_1);
		float gyroScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}};
		float accScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}};
		ParamToImu(boardImu,gyroScaleFactor,accScaleFactor);
#endif
#if Scha634_03_def
		boardImu = new Scha634_03(SPI3,SPI_BaudRatePrescaler_32,CsPin(),{GPIOA, GPIO_Pin_15});
		float gyroScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
		float accScaleFactor[3][3] = {{0.0f, -1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
		ParamToImu(boardImu,gyroScaleFactor,accScaleFactor);
#endif
#if BMI088_def
		boardImu = new Bmi088(SPI1,SPI_BaudRatePrescaler_64);
		float gyroScaleFactor[3][3] = { {0.0f, 1.0f, 0.0f},{0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f} };
		float accScaleFactor[3][3]  = { {0.0f, 1.0f, 0.0f},{0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f} };
		ParamToImu(boardImu,gyroScaleFactor,accScaleFactor);
#endif

		boardImu->TDT_ImuInit();
}



void ImuTask::ParamToImu(ImuCalc* boardImu,float GyroScaleFactor[3][3],float AccScaleFactor[3][3])
{
	boardImu->setGyroScaleFactor(GyroScaleFactor);
	boardImu->setAccScaleFactor(AccScaleFactor);
	IFlash.link(boardImu->gyro.offset, 2);
	IFlash.link(boardImu->acc.offset, 3);
}



/**
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
