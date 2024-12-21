#ifndef _INFO_TASK_H_
#define _INFO_TASK_H_

#include "board.h"
#include "task_virtual.h"
#include "dbus.h"

enum CANWHAT
{
	ONCan1=0,
	ONCan2=1,
	Both=2
};

struct SendCanPackage 
	{
		void *data;        // 指向发送数据的指针
		uint16_t id;       // CAN消息ID
		CANWHAT canwhat;   //can1 can2
		uint16_t frequency; // 发送频率     频率之和不能超过1000HZ	
		bool ifBan=0;
		uint64_t nextTime=0; // 下次发送时间  自动补充，不需要填充
		bool ifBusy=0;
	};
	
struct RecCanPackage 
	{
		void *data;        // 指向发送数据的指针
		uint16_t id;       // CAN消息ID
		CANWHAT canwhat;   //can1 can2
		uint16_t frequency; // 接收频率
		uint32_t lastReceiveTime;  // 上次接收时间 (us)
		bool offline=1;
	};	
	

class CanSend
{
	public:
	uint64_t currentTime;
  myvector<SendCanPackage,15> packages;  //15个包
	
	myqueue<SendCanPackage*> Can1Sendpackages;
	myqueue<SendCanPackage*> Can2Sendpackages;
	void PushSendPackage(SendCanPackage* pkg,CANWHAT can);
	void SendHandle();
	void SendInfo();
};


class InfoProcess: public VirtualTask
{
private:
	CanSend canSend1,canSend2;
  myvector<RecCanPackage,30> recpackage; //可以接收30个包
	
public:
  void run() override;
	void init() override;
	void infoManage();
	void ReceiveManage(CanRxMsg canRxMsg,CAN_TypeDef *can_x);
	InfoProcess();
#pragma pack(1)

	struct _gimbalMsg
	{
		float angleDelta; //4BYTE
		uint8_t state: 4;
	}gimbalMsg;
	
	struct _RCmsg
	{
		
		
	}RCmsg;
	
#pragma pack()
};




extern InfoProcess infoProcess;
	


#endif
