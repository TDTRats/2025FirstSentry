#ifndef _SYSTICK_H
#define _SYSTICK_H


#include "board.h"


#ifdef __cplusplus
 extern "C" {
#endif	/*__cplusplus*/
	 
void sysTickInit(void);
void SysTick_Handler(void); 
extern volatile uint32_t sysTickUptime;	 
	 
#ifdef __cplusplus
 }
#endif
 
 
#endif