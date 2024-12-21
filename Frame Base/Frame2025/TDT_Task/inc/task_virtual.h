#ifndef __TASK_VIRTUAL_H
#define __TASK_VIRTUAL_H

#include "board.h"

enum TaskStatus
{
    taskStateRun = 0,
    taskStateStop = 1,
};
enum TaskHz
{
	_1000hz = 0,
	 _500hz = 1,
	 _200hz = 2,
	 _100hz = 3,
	 _50hz = 4,
	 _20hz = 5,
	 _10hz = 6,
	 _2hz = 7,
	 _1hz = 8,
};

class VirtualTask
{
	public:
	TaskStatus status;
	TaskHz taskHz=_500hz;
	//构造函数
	VirtualTask();
	//初始化
  virtual void init() = 0;
  //主函数
  virtual void run() = 0;
	//可重写，遥控数据更新时触发
	virtual void remoteCtrlUpdate(){};
	//脱力
	virtual void deforceCallBack(){};
	//取消脱力
	virtual void deforceCancelCallBack(){};
		
		
};


class TaskControl
{
	public:
	//任务列表
	 VirtualTask **taskList=0;
	//任务数量
	 int taskNum=0;
	
	//全部任务初始化流程
	void AllTask_Init(void);
	//全部正在运行的任务放到任务列表
	void PushAllTask(void);
	//定义函数列表，列为不同频率1000hz->1hz ,并且开了最大每种频率下最多有10个任务
	myvector<VirtualTask*,10> taskFuncLists[9];
	
};

extern TaskControl taskControl;
#endif
