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
		void *data;        // ָ�������ݵ�ָ��
		uint16_t id;       // CAN��ϢID
		CANWHAT canwhat;   //can1 can2
		uint16_t frequency; // ����Ƶ��     Ƶ��֮�Ͳ��ܳ���1000HZ	
		bool ifBan=0;
		uint64_t nextTime=0; // �´η���ʱ��  �Զ����䣬����Ҫ���
		bool ifBusy=0;
	};
	
struct RecCanPackage 
	{
		void *data;        // ָ�������ݵ�ָ��
		uint16_t id;       // CAN��ϢID
		CANWHAT canwhat;   //can1 can2
		uint16_t frequency; // ����Ƶ��
		uint32_t lastReceiveTime;  // �ϴν���ʱ�� (us)
		bool offline=1;
	};	
	

class CanSend
{
	public:
	uint64_t currentTime;
  myvector<SendCanPackage,15> packages;  //15����
	
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
  myvector<RecCanPackage,30> recpackage; //���Խ���30����
	
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
