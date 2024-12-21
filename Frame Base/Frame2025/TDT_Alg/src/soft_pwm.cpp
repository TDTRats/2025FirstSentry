/*
Description: V6主控温控，使用1000Hz频率控制PC5的GPIO模拟占空比实现调温
Author: liuskywalkerjskd
*/
#include "soft_pwm.h"

soft_pwm bmiTempCtrl(40) ;//恒温30摄氏度

soft_pwm::soft_pwm(uint16_t TempVal):SetTemp(TempVal) //构造函数直接设定温度值
{
	
}

void soft_pwm::PC5_Init() 
{
	#if IF_USE_DJIC
		GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOC的时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    // 设置PC5引脚为输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // 设置为通用输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO速度
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使用上拉或下拉
    GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	#endif
	
	#if IF_USE_V5_V5PLUS_V6
		GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOC的时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // 设置PC5引脚为输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // 设置为通用输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO速度
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使用上拉或下拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	#endif
}

void soft_pwm::tempCtrlInit()
{
	PC5_Init() ;
	tempCtrlPid.setPlanNum(1) ;
	tempCtrlPid.fbValuePtr[0] = &tempfb ;
	tempCtrlPid.paramPtr = &tempPidPara ;
	
	tempPidPara.kp = 1.3 ;
	tempPidPara.ki = 0.6 ;
	tempPidPara.kd = 0.01 ;
	tempPidPara.integralErrorMax = 3 ;
	tempPidPara.resultMax = 10 ;
}

void soft_pwm::Get_Temp()
{
	tempfb = watchtemp ; //读取bmi088实时温度
}

void soft_pwm::Calc_Pid()
{
	Op_Rate = tempCtrlPid.Calculate(SetTemp) ; //计算温控pid
}

void soft_pwm::Simulate_PWM() //模拟PWM方波输出
{
	Get_Temp() ;
	Calc_Pid() ;
	if(Op_Rate < 0)
	{
		#if IF_USE_V5_V5PLUS_V6
		GPIO_SetBits(GPIOC, GPIO_Pin_5); // PC5设置为低电平
		#endif
		
		#if IF_USE_DJIC
		GPIO_ResetBits(GPIOF, GPIO_Pin_6); // PC5设置为低电平
		#endif
	}	
	else
	{
		runTime ++ ;
		if(runTime <= Op_Rate)
		{
			#if IF_USE_V5_V5PLUS_V6
			GPIO_ResetBits(GPIOC, GPIO_Pin_5); // PC5设置为高电平
			#endif
			
			#if IF_USE_DJIC
			GPIO_SetBits(GPIOF, GPIO_Pin_6); // PC5设置为低电平
			#endif
		}
		else
		{
			#if IF_USE_V5_V5PLUS_V6
			GPIO_SetBits(GPIOC, GPIO_Pin_5); // PC5设置为低电平
			#endif
			
			
			#if IF_USE_DJIC
			GPIO_ResetBits(GPIOF, GPIO_Pin_6); // PC5设置为低电平
			#endif
		}
		
		if(runTime >= 10)
		{
			runTime%=10 ;
		}
	}	
}