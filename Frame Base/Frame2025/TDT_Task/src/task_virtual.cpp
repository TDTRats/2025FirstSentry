/******************************
Description: 虚任务构造器，总任务调度
Class TaskControl
	{
		——————————————————————————————————————————————————————————————————————————
		VirtualTask **TaskList;     //任务列表
		——————————————————————————————————————————————————————————————————————————
		int taskNum;                //任务数量
		——————————————————————————————————————————————————————————————————————————
		void AllTask_Init(void);    //全部任务初始化流程
		——————————————————————————————————————————————————————————————————————————
		void PushAllTask(void);    	//全部正在运行的任务放到任务列表
		——————————————————————————————————————————————————————————————————————————
		taskFuncLists[10]						//十个频率的正在运行的任务函数指针
		——————————————————————————————————————————————————————————————————————————
	};
	
Class VirtualTask
	{
		——————————————————————————————————————————————————————————————————————————
		TaskStatus status;      //状态选择
		TaskHz taskHz;          //运行频率
		——————————————————————————————————————————————————————————————————————————
		virtual void init() = 0;  //初始化
		——————————————————————————————————————————————————————————————————————————
		virtual void run() = 0;   //主函数
		——————————————————————————————————————————————————————————————————————————
		virtual void remoteCtrlUpdate(){};  遥控数据更新时触发
		——————————————————————————————————————————————————————————————————————————
		virtual void deforceCallBack(){};   //脱力
		——————————————————————————————————————————————————————————————————————————
		virtual void deforceCancelCallBack(){};  //取消脱力
	};
****************************  */


#include "task_virtual.h"
#include "schedule.h"
//1、任务、对象都不允许删除
//2、只能在任务里面创建
//3、尽量不要调用关于该任务的挂起、恢复函数（避免多重挂起、恢复）

TaskControl taskControl;
VirtualTask::VirtualTask()
{
	if(taskControl.taskList == 0)
	{
		taskControl.taskList = (VirtualTask **)malloc(sizeof(VirtualTask *));
	}
	else
	{
		taskControl.taskList = (VirtualTask **)realloc(taskControl.taskList, sizeof(VirtualTask *) * (taskControl.taskNum+1));
	}
	
	taskControl.taskList[taskControl.taskNum] = this;
	taskControl.taskNum+=1;
}
/**
* @brief 全部任务初始化
 * 
 */
void TaskControl::AllTask_Init(void)
{
	for (u8 i = 0; i < taskNum; i++)
	{
		 if(taskList[i]->status==taskStateRun)
     taskList[i]->init();
	}
	
}

/**
* @brief 刷新tasklist 将正在运行的函数指针放到对应频率下的vector中
 * 
 */
void TaskControl::PushAllTask(void)
{
	for (u8 i = 0; i < 9; i++)
  {
		taskFuncLists[i].clear();
	}
	
	for (u8 i = 0; i < taskNum; i++)
  {
        //调用运行态任务的主函数
      if (taskList[i]->status == taskStateStop)  continue;
			taskFuncLists[taskList[i]->taskHz].push_back(taskList[i]);
	}
}

