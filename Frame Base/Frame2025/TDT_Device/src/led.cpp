/******************************
Description: led外设 led状态设定
function:
	——————————————————————————————————————————————————————————————————————————
	void stateShow(LedES state);	   调用接口，led闪烁状态控制 输入状态则闪烁相应状态
	——————————————————————————————————————————————————————————————————————————
	void init(void);     led初始化
	——————————————————————————————————————————————————————————————————————————
可设置参数：
	——————————————————————————————————————————————————————————————————————————
	inline void setLHNagation(u8 LHNagation) { this->LHNagation = LHNagation; }  是否取反
	inline void setStateOnSpan(u16 stateOnSpan) { this->stateOnSpan = stateOnSpan; };   闪烁时亮的时间
	inline void setStateOffSpan(u16 stateOffSpan) { this->stateOffSpan = stateOffSpan; };  闪烁时灭的时间
	inline void setStateResetTime(u16 stateResetTime) { this->stateResetTime = stateResetTime; };  一套闪烁后冷却时间
	inline void setSlowBlinkInterval(u16 slowBlinkInterval) { this->slowBlinkInterval = slowBlinkInterval; }; 慢闪时间
	inline void setFastBlinkInterval(u16 fastBlinkInterval) { this->fastBlinkInterval = fastBlinkInterval; }; 快闪时间
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "led.h"

Led::Led(uint32_t RCC_AHB1Periph_GPIOx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x) : RCC_AHB1Periph_GPIOx(RCC_AHB1Periph_GPIOx), GPIOx(GPIOx), GPIO_Pin_x(GPIO_Pin_x), stateOnSpan(80), stateOffSpan(150), slowBlinkInterval(500), fastBlinkInterval(150), stateResetTime(1000)
{
	;
}

/**
  * @brief  LED初始化
  */
void Led::init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	GPIOx->BSRRL = GPIO_Pin_x;
	show(0);
}

/**
  * @note	可根据状态值闪烁不同的次数
  */
void Led::stateShow(int8_t state)
{
	u16 intervalMs=cycle.getCycleT()*1000;
	blinkTimer += intervalMs;
	if (state == 0)
	{
		state = LedES_BlinkSlow ;
	}
	
	if (state != stateLast) //判断是否值改变
	{
		resetState();
		show(0);
		stateLast = state;
		return;
	}
		
	// 处理常亮和常暗状态
	if (state == LedES_ConstantLight || state == LedES_ConstantDark)			
	{
		show(state == LedES_ConstantLight);
		resetState();
		return;
	}
	
	// 处理慢闪和快闪状态
	if (state == LedES_BlinkSlow || state == LedES_BlinkFast)
	{
		u16 blinkInterval = (state == LedES_BlinkSlow) ? slowBlinkInterval : fastBlinkInterval;
		if (blinkTimer < blinkInterval)
			return;
		resetState();
		toggle();
		return;
	}

	if (blinkTimer > stateResetTime + stateOffSpan && stateCounter >= state)
	{
		stateCounter = 0;
	}

	if (showState == 0 && blinkTimer > stateOffSpan && stateCounter < state)
	{
		show(1);
		blinkTimer = 0;
		return;
	}

	if (showState == 1 && blinkTimer > stateOnSpan && stateCounter < state)
	{
		show(0);
		blinkTimer = 0;
		stateCounter++;
		return;
	}
}

void Led::show(u8 state)
{
	showState = state;
	if (state ^ LHNagation)
	{
		GPIOx->BSRRL = GPIO_Pin_x;
		return;
	}
	GPIOx->BSRRH = GPIO_Pin_x;
}


/**
 * @file led.cpp
 * @author 梁文生
 * @brief 
 * @version 0.1
 * @date 2021-10-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
