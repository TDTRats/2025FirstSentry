#ifndef _SOFT_PWM_H_
#define _SOFT_PWM_H_

#include "board.h"
#include "bmi088.h"
#include "pid.h"

class soft_pwm
{
	public:
		soft_pwm(uint16_t TempVal) ;
		void tempCtrlInit() ;
		void Simulate_PWM() ;
	private :
		int runTime ;
		uint16_t SetTemp ;
		int16_t Op_Rate ; //输出率，模拟占空比
		void Get_Temp() ;
		void PC5_Init() ;
		void Calc_Pid() ;
		Pid tempCtrlPid ;
		float tempfb ;
		PidParam tempPidPara ;
} ;

extern soft_pwm bmiTempCtrl ;

#endif