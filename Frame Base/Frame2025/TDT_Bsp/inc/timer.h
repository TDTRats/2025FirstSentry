/*****************************************************************************
File name: TDT_Bsp\src\timer.h
Description: 定时器
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _TIMER_H
#define _TIMER_H

#include "board.h"

uint64_t getSysTimeUs(void);
void delayUs(uint32_t us);
void delayMs(uint32_t ms);
float timeIntervalFrom_f(float timeFrom);
uint64_t timeIntervalFrom(uint64_t timeFrom);
void ConfigureTimeForRunTimeStats(void);
extern volatile uint32_t runTimeTicks;
uint32_t RunAndGetTime(void (*taskFunc)(void));

#ifdef __cplusplus    //声明c++库的时候写这个，在.c文件中自动屏蔽
//放入类的成员函数
template<typename T>
uint32_t RunAndGetTime(T* obj, void (T::*taskFunc)()) 
{
    uint64_t startTimeStamp = getSysTimeUs();
    
    // 调用类的成员函数
    (obj->*taskFunc)();
    
    uint64_t endTimeStamp = getSysTimeUs();
    return (endTimeStamp - startTimeStamp);
}
#endif
#ifdef __cplusplus
 extern "C" {
#endif	/*__cplusplus*/
	 
void TIM2_IRQHandler(void);
 
#ifdef __cplusplus
}
#endif	/*__cplusplus*/

#endif