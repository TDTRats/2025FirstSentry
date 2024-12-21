/******************************
Description:  infotask
Function:
	����������������������������������������������������������������������������������������������������������������������������������������������������
	canSend1.packages ֻҪ�����Ϣ���Ϳ����Զ���һ��Ƶ�ʷ��ͱ���Ϣ
	����������������������������������������������������������������������������������������������������������������������������������������������������
  �����ص㣺���ټ��1ms����һ����������Ƶ�ʼ���������1000hz���Զ���Ƶ��1000hz
	����������������������������������������������������������������������������������������������������������������������������������������������������
	canSend1��canSend2 ���ٲ��з���
	����������������������������������������������������������������������������������������������������������������������������������������������������
	packages[i].ifBan=1 ��ǿ�ƽ�ֹ����
	����������������������������������������������������������������������������������������������������������������������������������������������������
	ReceiveManage(CanRxMsg canRxMsg,CAN_TypeDef *can_x) ���մ���
	����������������������������������������������������������������������������������������������������������������������������������������������������
	recpackage[i] frequency ����Ƶ�� offline ����0.3sû�յ��Զ���1
	����������������������������������������������������������������������������������������������������������������������������������������������������
	eg: canSend1.packages = {{&test1,0x103,ONCan1,500}};         //��������1
	
	
	
****************************  */
#include "info_task.h"
#include "can.h"
#include "judgement.h"

InfoProcess infoProcess;

InfoProcess::InfoProcess()
{
		status=taskStateRun;
		taskHz=_1000hz;
	
	
    canSend1.packages = {/* ��ʼ�����ݰ�1 */};         //��������1
		canSend2.packages = {/* ��ʼ�����ݰ�2 */};         //��������2
		
    recpackage = {{&gimbalMsg,0x300,ONCan1,100}
										};
}


void InfoProcess::init()
{
}


void InfoProcess::run()
{
	
	uint64_t currentTime = getSysTimeUs();              //�������ߴ���
	
	 for (int i=0; i<recpackage.size(); i++)
	{
		if((currentTime -recpackage[i].lastReceiveTime)>0.3*1e6f)
			recpackage[i].offline=1;
		else
			recpackage[i].offline=0;
	}
	
	
	infoManage();
	canSend1.SendInfo();
	canSend2.SendInfo();
}


//�ṹ�����ݴ������
void InfoProcess::infoManage()
{
	
}	


void InfoProcess::ReceiveManage(CanRxMsg canRxMsg,CAN_TypeDef *can_x)
{
	uint64_t currentTime = getSysTimeUs();
	
	 for (int i=0; i<recpackage.size(); i++)
	{
		RecCanPackage& pkg=recpackage[i];
		
		if(can_x ==CAN1 && pkg.canwhat!=ONCan2 && canRxMsg.StdId==pkg.id )
		{
			memcpy(pkg.data,canRxMsg.Data,sizeof(canRxMsg.Data));
			pkg.frequency=1e6f/(currentTime - pkg.lastReceiveTime);  // ʱ��� (΢��)
			pkg.lastReceiveTime=currentTime;
	  }
		
		if(can_x ==CAN2 && pkg.canwhat!=ONCan1 && canRxMsg.StdId==pkg.id )
		{
			memcpy(pkg.data,canRxMsg.Data,sizeof(canRxMsg.Data));
			pkg.frequency=1e6f/(currentTime - pkg.lastReceiveTime);  // ʱ��� (΢��)
			pkg.lastReceiveTime=currentTime;
	  }
		
	}
}	



void  CanSend::SendInfo()
{
	 currentTime = getSysTimeUs();
	 for (int i=0; i<packages.size(); i++)
   {
      if (packages[i].ifBan==false && packages[i].ifBusy==false && currentTime >= packages[i].nextTime)
				PushSendPackage(&packages[i],packages[i].canwhat);
   }
	SendHandle();	
}


void  CanSend::PushSendPackage(SendCanPackage* pkg,CANWHAT can)
{
	
	pkg->ifBusy=true;
	if(can==ONCan1||can==Both)
	{
		Can1Sendpackages.push(pkg);
	}
	if(can==ONCan2||can==Both)
	{
		Can2Sendpackages.push(pkg);
	}
}

void  CanSend::SendHandle()
{
		if(!Can1Sendpackages.empty())
		{
			 // �����´η���ʱ��
		  SendCanPackage*package = Can1Sendpackages.front_element();
			canTx((u8*)(package->data),CAN1,package->id);
			package->ifBusy=false;
			package->nextTime= currentTime + (1e6f /	package->frequency);
			Can1Sendpackages.pop();	
		}
		
			if(!Can2Sendpackages.empty())
		{
		  SendCanPackage*package1= Can2Sendpackages.front_element();
			canTx((u8*)(package1->data),CAN2,package1->id);
			package1->ifBusy=false;
			package1->nextTime= currentTime + (1e6f /	package1->frequency);
			Can2Sendpackages.pop();
		}
}


/*
 * @Date: 2022-10-26
 * @History: handsome 2022-10-26
 *           codeartist 2024-6-9  2024_9_10
 * @FilePath: \Projectd:\TDT2022\TDT-sentry\TDT_Task\src\infoProcess_task.cpp
 * @Description:��Ϣ�������񣺽������������յ�����Ϣ���д����ţ����ҽ���Ҫ��������Ϣ���з���
 * @����Զ������Զ����գ���Ƶ���ͣ�ʹ����Ϣ����
 */














