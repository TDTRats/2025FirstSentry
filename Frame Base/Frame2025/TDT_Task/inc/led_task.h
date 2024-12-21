/*****************************************************************************
File name: TDT_Task\src\led_task.h
Description: 呼吸灯控制任务
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __LED_TASK_H__
#define __LED_TASK_H__

#include "board.h"
#include "led.h"
#include "task_virtual.h"

enum Color
{
	RED=0,
	BLUE,
	GREEN,
	PURPLE,
	WHITE
};


class Led_Task :public VirtualTask
{
private:
	bool setNowFlag;
	uint64_t setNowTime;
public:
	Led_Task();
	void run() override;
	void init() override;


#if IF_USE_V5_V5PLUS_V6
	Led boardLed = Led(RCC_AHB1Periph_GPIOB , GPIOB , GPIO_Pin_14);
#endif

#if IF_USE_DJIC
Led boardLed_B = Led(RCC_AHB1Periph_GPIOH , GPIOH, GPIO_Pin_10);
Led boardLed_G= Led(RCC_AHB1Periph_GPIOH , GPIOH, GPIO_Pin_11);
Led boardLed_R= Led(RCC_AHB1Periph_GPIOH , GPIOH, GPIO_Pin_12);
void RGB_LED_control(int8_t ES_State,Color color);
#endif
	struct  Error
	{
		int8_t ES_State;
		uint8_t* iferr;
		Color color;
	} ErrorCode[10]; ///<状态值缓存区，最多允许同时存在10个状态值,前面优先级高
	
	///@brief 获取优先级最高且有效的状态值
	///该方法会从最高优先级开始遍历所有状态码，当该优先级对应的状态码不为 LedES_Disable 时该优先级对应的状态码，若所有错误码都是 LedES_Disable 则返回 LedES_BlinkSlow
	///@sa LedEsState
	void ErrorLink(uint8_t* iferr,uint8_t priority, int8_t ES_State,Color color);
	void SetNow(int8_t ES_State,Color color=WHITE);
	
	
	int8_t getError();
	int8_t Nowstate;
	Color colorNow;
	
};

extern Led_Task ledtask;
extern Led laser ;
#endif
