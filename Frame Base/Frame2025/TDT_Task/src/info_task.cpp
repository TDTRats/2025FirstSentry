/******************************
Description:  infotask
Function:
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
	canSend1.packages Ö»ÒªÌî³äÐÅÏ¢£¬¾Í¿ÉÒÔ×Ô¶¯°´Ò»¶¨ÆµÂÊ·¢ËÍ±¾ÐÅÏ¢
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
  ·¢ËÍÌØµã£ºÖÁÉÙ¼ä¸ô1ms·¢ËÍÒ»Ìõ£¬·¢ËÍ×ÜÆµÂÊ¼ÓÆðÀ´³¬¹ý1000hzÔò×Ô¶¯½µÆµµ½1000hz
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
	canSend1£¬canSend2 ¿ª±Ù²¢ÐÐ·¢ËÍ
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
	packages[i].ifBan=1 ÔòÇ¿ÖÆ½ûÖ¹·¢ËÍ
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
	ReceiveManage(CanRxMsg canRxMsg,CAN_TypeDef *can_x) ½ÓÊÕ´¦Àí
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
	recpackage[i] frequency ½ÓÊÕÆµÂÊ offline ³¬¹ý0.3sÃ»ÊÕµ½×Ô¶¯ÖÃ1
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
	eg: canSend1.packages = {{&test1,0x103,ONCan1,500}};         //·¢¸øÖ÷¿Ø1
	
	
	
****************************  */
#include "info_task.h"
#include "can.h"
#include "judgement.h"

InfoProcess infoProcess;

InfoProcess::InfoProcess()
{
		status=taskStateRun;
		taskHz=_1000hz;
	
	
    canSend1.packages = {/* ³õÊ¼»¯Êý¾Ý°ü1 */};         //·¢¸øÖ÷¿Ø1
		canSend2.packages = {/* ³õÊ¼»¯Êý¾Ý°ü2 */};         //·¢¸øÖ÷¿Ø2
		
    recpackage = {{&gimbalMsg,0x300,ONCan1,100}
										};
}


void InfoProcess::init()
{
}


void InfoProcess::run()
{
	
	uint64_t currentTime = getSysTimeUs();              //½ÓÊÕÀëÏß´¦Àí
	
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


//½á¹¹ÌåÊý¾Ý´¦ÀíÌî³ä
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
			pkg.frequency=1e6f/(currentTime - pkg.lastReceiveTime);  // Ê±¼ä²î (Î¢Ãë)
			pkg.lastReceiveTime=currentTime;
	  }
		
		if(can_x ==CAN2 && pkg.canwhat!=ONCan1 && canRxMsg.StdId==pkg.id )
		{
			memcpy(pkg.data,canRxMsg.Data,sizeof(canRxMsg.Data));
			pkg.frequency=1e6f/(currentTime - pkg.lastReceiveTime);  // Ê±¼ä²î (Î¢Ãë)
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
			 // ¸üÐÂÏÂ´Î·¢ËÍÊ±¼ä
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
 * @Description:ÐÅÏ¢´¦ÀíÈÎÎñ£º½«»úÆ÷ÈËËùÓÐÊÕµ½µÄÐÅÏ¢½øÐÐ´¦Àí·¢·Å£¬²¢ÇÒ½«ÐèÒª½»»»µÄÐÅÏ¢½øÐÐ·¢ËÍ
 * @Ìí¼Ó×Ô¶¯·¢ËÍ×Ô¶¯½ÓÊÕ£¬²îÆµ·¢ËÍ£¬Ê¹ÓÃÏûÏ¢¶ÓÁÐ
 */














