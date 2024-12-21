/******************************
Description: TDT_LOOP任务分频
function:
	——————————————————————————————————————————————————————————————————————————
	void TDT_Loop(void)
	——————————————————————————————————————————————————————————————————————————
  void TDT_Loop_xxxHz(void)
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "schedule.h"
#include "dbus.h"
#include "imu_task.h"
#include "led_task.h"
#include "judgement.h"
#include "motor.h"
#include "pwm.h"
#include "adc.h"
#include "can.h"
#include "task_virtual.h"
#include "soft_pwm.h"
#include "vision.h"
#include "info_task.h"
#include "task_virtual.h"
#include "time.h"
#include "power.h"
#include "lk_motor.h"
#include "log.h"
Schedule robotSchedule;


void TDT_Loop_1000Hz(void) //1ms执行一次
{
	
#if USE_VISION
	Vision.run_1000hz();
#endif
	
#if USE_JUDGEMENT
	judgement.run_1000hz();
#endif
	
#if USE_TEMP_CTRL
	bmiTempCtrl.Simulate_PWM() ; //imu温控
#endif
	
#if USE_RC
	RC.run_1000Hz();
#else
	deforceFlag=0;
#endif

#if USE_POWER
	power.myPowerCanTx();
#endif
	
}
void TDT_Loop_500Hz(void) //2ms执行一次
{
#if USE_VISION
	Vision.vision_Send_Data();
#endif
	Motor::DjisendCanMsg();
	LKList_Send();
}


void TDT_Loop_200Hz(void) //5ms执行一次
{
}

void TDT_Loop_100Hz(void) //10ms执行一次
{
}

void TDT_Loop_50Hz(void) //20ms执行一次
{
}

void TDT_Loop_20Hz(void) //50ms执行一次
{
}


void TDT_Loop_10Hz(void) //100ms执行一次
{
	#if USE_LOG
	log_task.run();
	#endif
}

void TDT_Loop_2Hz(void) //500ms执行一次
{

}

void TDT_Loop_1Hz(void) //1000ms执行一次
{
}
/**
 * @ingroup TDT_Frame
 * @defgroup TDT_SCHEDULE_API schedule相关接口  1ms被调用一次
 * @brief 该模块展示了schedule的相关接口
 */

void TDT_Loop(void)
{
	
	cntUpdate(&robotSchedule);
	Run_UdTime(&robotSchedule);
	taskControl.PushAllTask();
	robotSchedule.CPU_usage = (robotSchedule.runTime_1ms*1000 + robotSchedule.runTime_2ms*500 + robotSchedule.runTime_5ms*200 + robotSchedule.runTime_10ms*100 + robotSchedule.runTime_20ms*50 + robotSchedule.runTime_50ms*20 + robotSchedule.runTime_100ms*10 + robotSchedule.runTime_500ms*2 + robotSchedule.runTime_1000ms)/1e6f;
}


/**
 * 
 * @brief 更新_robotSchedule的cnt变量，自动循环1-1000
 */
void cntUpdate(_Schedule* _robotSchedule)
{
	(_robotSchedule->cnt_ms==1000)?	_robotSchedule->cnt_ms=1	:_robotSchedule->cnt_ms++;
}

/**
* @brief 调度传入的loop以及传入的task列表中的函数，并且返回运行时间
 * 
 */

uint32_t LoopAndTask_Run(void (*taskFunc)(void),myvector<VirtualTask*,10> &task)
{	
	if(task.size()==0)
		return RunAndGetTime(taskFunc);
	
	uint32_t taskTime=0;
	for(int i=0 ;i<task.size();i++)
	{
		taskTime+=RunAndGetTime(task[i],&VirtualTask::run);
	}
	return RunAndGetTime(taskFunc)+taskTime;
}

/**
* @brief 根据cnt的值调度不同的loop函数和task函数，并且返回运行时间
 * 
 */
void Run_UdTime(_Schedule* _robotSchedule)
{
	_robotSchedule->runTime_1ms 		= LoopAndTask_Run(TDT_Loop_1000Hz,taskControl.taskFuncLists[0]);
	_robotSchedule->runTime_2ms 		= (_robotSchedule->cnt_ms % 2 == 0)?			LoopAndTask_Run(TDT_Loop_500Hz,taskControl.taskFuncLists[1]) : _robotSchedule->runTime_2ms;
	_robotSchedule->runTime_5ms 		= (_robotSchedule->cnt_ms % 5 == 0)?			LoopAndTask_Run(TDT_Loop_200Hz,taskControl.taskFuncLists[2]) : _robotSchedule->runTime_5ms;
	_robotSchedule->runTime_10ms 		= (_robotSchedule->cnt_ms % 10 == 0)?			LoopAndTask_Run(TDT_Loop_100Hz,taskControl.taskFuncLists[3]) : _robotSchedule->runTime_10ms;
	_robotSchedule->runTime_20ms 		= (_robotSchedule->cnt_ms % 20 == 0)?			LoopAndTask_Run(TDT_Loop_50Hz,taskControl.taskFuncLists[4])  : _robotSchedule->runTime_20ms;
	_robotSchedule->runTime_50ms 		=	(_robotSchedule->cnt_ms % 50 == 0)?			LoopAndTask_Run(TDT_Loop_20Hz,taskControl.taskFuncLists[5])  : _robotSchedule->runTime_50ms;
	_robotSchedule->runTime_100ms 	= (_robotSchedule->cnt_ms % 100 == 0)?		LoopAndTask_Run(TDT_Loop_10Hz,taskControl.taskFuncLists[6])  : _robotSchedule->runTime_100ms;
	_robotSchedule->runTime_500ms 	= (_robotSchedule->cnt_ms % 500 == 0)?		LoopAndTask_Run(TDT_Loop_2Hz,taskControl.taskFuncLists[7]) 	 : _robotSchedule->runTime_500ms;
	_robotSchedule->runTime_1000ms 	= (_robotSchedule->cnt_ms % 1000 == 0)?		LoopAndTask_Run(TDT_Loop_1Hz,taskControl.taskFuncLists[8]) 	 : _robotSchedule->runTime_1000ms;
}





