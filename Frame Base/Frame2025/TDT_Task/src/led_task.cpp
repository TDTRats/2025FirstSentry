/******************************
File name: TDT_Task\src\led_task.cpp
Description: 呼吸灯控制任务
function:
	——————————————————————————————————————————————————————————————————————————
	void ErrorLink(uint8_t* iferr,uint8_t priority, int8_t ES_State,,Color color); 设置err列表  当变量为1时 显示对应状态  当多个变量为1时，优先显示高优先级
	——————————————————————————————————————————————————————————————————————————
	priority 0最高优先级   9最低优先级
	——————————————————————————————————————————————————————————————————————————
	SetNow(int8_t ES_State)  无论现在在什么状态，立刻显示该状态  ，0.3秒没调用则自动返回主列表显示
	——————————————————————————————————————————————————————————————————————————
*/

/**led表：
红灯单闪：遥控器离线
红灯双闪：usb离线
红灯三闪：视觉串口离线
红灯四闪：功率离线
*/
#include "led_task.h"
#include "iwdg.h"
#include "vision.h"
#include "info_task.h"
#include "dbus.h"
#include "power.h"
#include "log.h"

#if IF_USE_V5_V5PLUS_V6
Led laser = Led(RCC_AHB1Periph_GPIOC , GPIOC, GPIO_Pin_8);
#endif

#if IF_USE_DJIC
Led laser = Led(RCC_AHB1Periph_GPIOC , GPIOC, GPIO_Pin_5);
#endif

Led_Task ledtask;

Led_Task::Led_Task()
{
	status=taskStateRun;
	taskHz=_20hz;
	
#if USE_VISION
	ErrorLink((u8*)&Vision.offlineFlag,7,LedES_BlinkTHREE,RED);
	
#if USB_UART
	ErrorLink((u8*)&Vision.USB_offline_Flag,6,LedES_BlinkTWO,RED);
#endif
	
#endif
	
#if USE_RC
	ErrorLink((u8*)&dbusOfflineFlag,5,LedES_BlinkONE,RED);
#endif
	
#if USE_POWER
	ErrorLink((u8*)&power.offlineFlag,8,LedES_BlinkFOUR,RED);
#endif
	
	ErrorLink((u8*)&log_task.full_error,9,LedES_BlinkONE,BLUE);
	
	
}









void Led_Task::init()
{
	
#if  IF_USE_DJIC
	boardLed_R.init();
	boardLed_R.setLHNagation(0);
	boardLed_G.init();
	boardLed_G.setLHNagation(0);
	boardLed_B.init();
	boardLed_B.setLHNagation(0);
#else
	/*LED初始化*/
	boardLed.init();
#endif
	
	laser.init();
	laser.setLHNagation(0);
	
	
}

/**
  * @brief LED任务函数
  * @note 负责LED的控制和喂狗
  * @warning 该函数为重写完成
  */
void Led_Task::run()
{
	
	//趁机喂狗
	iwdgFeed();
	//调用SetNow函数离线检测
	if(timeIntervalFrom_f(setNowTime/1e6f)>0.3)
	setNowFlag=0;	
	
	//若没调用，跑正常error表
	if(setNowFlag!=1)
	{
		ledtask.Nowstate=getError();
		#if  IF_USE_DJIC
			RGB_LED_control(getError(),colorNow);                                             
		#else
			boardLed.stateShow(getError());
		#endif
	}
	
	
	laser.stateShow(LedES_ConstantLight);
	
}

void Led_Task:: SetNow(int8_t ES_State,Color color)
{
		#if  IF_USE_DJIC
			RGB_LED_control(ES_State,color);                                   
		#else
			boardLed.stateShow(ES_State);
	#endif
	setNowFlag=1;
	setNowTime=getSysTimeUs();
	Nowstate=ES_State;
	
}

void Led_Task:: ErrorLink(uint8_t* iferr,uint8_t priority, int8_t ES_State,Color color)
{
		if (priority >= 10)
			return;
			ErrorCode[priority].ES_State  = ES_State;
			ErrorCode[priority].iferr=iferr;
			ErrorCode[priority].color=color;
}

int8_t Led_Task::getError()
{
		//LED只展示位置最先的异常（表现为位于数组的第一个有效数据）
		for (u8 i = 0; i < 10; i++)
		{
			if (ErrorCode[i].ES_State != LedES_Disable &&  ErrorCode[i].iferr != nullptr && *(ErrorCode[i].iferr)==1)
			{
				colorNow=ErrorCode[i].color;
				return ErrorCode[i].ES_State;
			}
		}
		colorNow=WHITE;
		return LedES_BlinkSlow ;
}



#if IF_USE_DJIC
void Led_Task::RGB_LED_control(int8_t ES_State,Color color)
{
	switch(color)
	{
		case RED:
			boardLed_R.stateShow(ES_State);
			boardLed_G.stateShow(LedES_ConstantDark);
			boardLed_B.stateShow(LedES_ConstantDark);
			break;
		case BLUE:
			boardLed_R.stateShow(LedES_ConstantDark);
			boardLed_G.stateShow(LedES_ConstantDark);
			boardLed_B.stateShow(ES_State);
			break;
		case GREEN:
			boardLed_G.stateShow(ES_State);
		  boardLed_R.stateShow(LedES_ConstantDark);
	  	boardLed_B.stateShow(LedES_ConstantDark);
			break;
		case PURPLE:
			boardLed_R.stateShow(ES_State);
			boardLed_B.stateShow(ES_State);
	  	boardLed_G.stateShow(LedES_ConstantDark);
			break;
		case WHITE:
			boardLed_R.stateShow(ES_State);
			boardLed_G.stateShow(ES_State);
			boardLed_B.stateShow(ES_State);
			break;
		default:
			boardLed_R.stateShow(ES_State);
			boardLed_G.stateShow(ES_State);
			boardLed_B.stateShow(ES_State);
		break;
	}
}
#endif




/**
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */

