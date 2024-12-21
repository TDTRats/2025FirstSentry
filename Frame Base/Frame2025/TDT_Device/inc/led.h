/*****************************************************************************
File name: TDT_Device\src\led.h
Description: LED灯
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __LED_H__
#define __LED_H__

#include "board.h"
#include "cycle.h"
/**
 * @addtogroup LED 
 * @{
 */

///LED特殊状态码
typedef enum
{
	///状态码——快闪
	LedES_BlinkFast = -2,
	///状态码——慢闪
	LedES_BlinkSlow = -1,
	///状态码——常亮
	LedES_ConstantLight = -3,
	///状态码——常灭
	LedES_ConstantDark = -4,
	///状态码————闪烁次数
	LedES_BlinkONE =1,
	LedES_BlinkTWO =2,
	LedES_BlinkTHREE =3,
	LedES_BlinkFOUR =4,
	LedES_BlinkFIVE =5,
	LedES_BlinkSIX=6,
	///状态码——该状态码下的优先级无效
	LedES_Disable = 0,
} LedES;

/// @brief 该LED类除了基本的拉高拉低以外，对led基本的用途——状态显示进行了封装，并兼容控制阴极和控制阳极的两种LED不同接法
class Led
{
private:
	uint32_t RCC_AHB1Periph_GPIOx; ///<RCC_LED
	GPIO_TypeDef *GPIOx;		   ///<LED_PORT
	uint16_t GPIO_Pin_x;		   ///<LED_Pin

	int8_t stateLast;	 ///<上一次运行时的状态值
	int8_t stateCounter; ///<状态值计数器

	u8 showState;	   ///<当前led的状态
	u8 LHNagation = 1; ///<是否取反,若(state^LHNagation)则拉低，否则拉高

	///LED亮持续时间
	u16 stateOnSpan;
	///LED灭持续时间
	u16 stateOffSpan;
	///状态值显示冷却时间
	u16 stateResetTime;

	///慢闪持续间隔
	u16 slowBlinkInterval;
	///快闪持续间隔
	u16 fastBlinkInterval;

	u16 blinkTimer; ///<闪烁计数器
	Cycle cycle;
public:
	Led(uint32_t RCC_AHB1Periph_GPIOx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x);
	 
	///@brief led初始化
	///@details 根据构造器出入的参数对led的gpio外设初始化
	void init(void);

	///@brief 设置led输出是否取反 若led期望的亮灭状态与现实不同，建议取反LHNagation
	///@sa LHNagation
	inline void setLHNagation(u8 LHNagation) { this->LHNagation = LHNagation; }

	///@brief 设置stateOnSpan
	///@sa stateOnSpan
	inline void setStateOnSpan(u16 stateOnSpan) { this->stateOnSpan = stateOnSpan; };

	///@brief 设置stateOffSpan
	///@sa stateOffSpan
	inline void setStateOffSpan(u16 stateOffSpan) { this->stateOffSpan = stateOffSpan; };

	///@brief 设置slowBlinkInterval
	///@sa slowBlinkInterval
	inline void setSlowBlinkInterval(u16 slowBlinkInterval) { this->slowBlinkInterval = slowBlinkInterval; };

	///@brief 设置fastBlinkInterval
	///@sa fastBlinkInterval
	inline void setFastBlinkInterval(u16 fastBlinkInterval) { this->fastBlinkInterval = fastBlinkInterval; };

	///@brief 设置stateResetTime
	///@sa stateResetTime
	inline void setStateResetTime(u16 stateResetTime) { this->stateResetTime = stateResetTime; };
	
	inline void resetState() { blinkTimer = 0;stateCounter = 0; };

	///@brief led闪烁状态控制
	
	void stateShow(int8_t state);

	///@brief led的GPIO输出
	///@param state 1为亮, 0为灭
	void show(u8 state = 1);
	///@brief 获取led状态
	inline u8 getState() { return ((GPIOx->ODR & GPIO_Pin_x) == GPIO_Pin_x) ^ LHNagation; };
	///@brief led取反
	inline void toggle() { GPIOx->ODR ^= GPIO_Pin_x; };
};

/** @} */

#endif
