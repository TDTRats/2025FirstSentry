/******************************
Description:  infotask
Function:
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
	canSend1.packages 峪勣野割佚連��祥辛參徭強梓匯協撞楕窟僕云佚連
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
  窟僕蒙泣�砦蘇拏筝�1ms窟僕匯訳��窟僕悳撞楕紗軟栖階狛1000hz夸徭強週撞欺1000hz
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
	canSend1��canSend2 蝕悦旺佩窟僕
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
	packages[i].ifBan=1 夸膿崙鋤峭窟僕
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
	ReceiveManage(CanRxMsg canRxMsg,CAN_TypeDef *can_x) 俊辺侃尖
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
	recpackage[i] frequency 俊辺撞楕 offline 階狛0.3s短辺欺徭強崔1
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
	eg: canSend1.packages = {{&test1,0x103,ONCan1,500}};         //窟公麼陣1
	
	
	
****************************  */
#include "info_task.h"
#include "can.h"
#include "judgement.h"

InfoProcess infoProcess;

InfoProcess::InfoProcess()
{
		status=taskStateRun;
		taskHz=_1000hz;
	
	
    canSend1.packages = {/* 兜兵晒方象淫1 */};         //窟公麼陣1
		canSend2.packages = {/* 兜兵晒方象淫2 */};         //窟公麼陣2
		
    recpackage = {{&gimbalMsg,0x300,ONCan1,100}
										};
}


void InfoProcess::init()
{
}


void InfoProcess::run()
{
	
	uint64_t currentTime = getSysTimeUs();              //俊辺宣�澳�尖
	
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


//潤更悶方象侃尖野割
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
			pkg.frequency=1e6f/(currentTime - pkg.lastReceiveTime);  // 扮寂餓 (裏昼)
			pkg.lastReceiveTime=currentTime;
	  }
		
		if(can_x ==CAN2 && pkg.canwhat!=ONCan1 && canRxMsg.StdId==pkg.id )
		{
			memcpy(pkg.data,canRxMsg.Data,sizeof(canRxMsg.Data));
			pkg.frequency=1e6f/(currentTime - pkg.lastReceiveTime);  // 扮寂餓 (裏昼)
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
			 // 厚仟和肝窟僕扮寂
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
 * @Description:佚連侃尖販暦�砂�字匂繁侭嗤辺欺議佚連序佩侃尖窟慧��旺拝繍俶勣住算議佚連序佩窟僕
 * @耶紗徭強窟僕徭強俊辺��餓撞窟僕��聞喘��連錦双
 */














