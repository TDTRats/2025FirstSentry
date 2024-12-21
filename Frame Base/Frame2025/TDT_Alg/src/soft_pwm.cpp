/*
Description: V6�����¿أ�ʹ��1000HzƵ�ʿ���PC5��GPIOģ��ռ�ձ�ʵ�ֵ���
Author: liuskywalkerjskd
*/
#include "soft_pwm.h"

soft_pwm bmiTempCtrl(40) ;//����30���϶�

soft_pwm::soft_pwm(uint16_t TempVal):SetTemp(TempVal) //���캯��ֱ���趨�¶�ֵ
{
	
}

void soft_pwm::PC5_Init() 
{
	#if IF_USE_DJIC
		GPIO_InitTypeDef GPIO_InitStructure;

    // ʹ��GPIOC��ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    // ����PC5����Ϊ���ģʽ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // ����Ϊͨ�����ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO�ٶ�
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ������������
    GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	#endif
	
	#if IF_USE_V5_V5PLUS_V6
		GPIO_InitTypeDef GPIO_InitStructure;

    // ʹ��GPIOC��ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // ����PC5����Ϊ���ģʽ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // ����Ϊͨ�����ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO�ٶ�
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // ��ʹ������������
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
	tempfb = watchtemp ; //��ȡbmi088ʵʱ�¶�
}

void soft_pwm::Calc_Pid()
{
	Op_Rate = tempCtrlPid.Calculate(SetTemp) ; //�����¿�pid
}

void soft_pwm::Simulate_PWM() //ģ��PWM�������
{
	Get_Temp() ;
	Calc_Pid() ;
	if(Op_Rate < 0)
	{
		#if IF_USE_V5_V5PLUS_V6
		GPIO_SetBits(GPIOC, GPIO_Pin_5); // PC5����Ϊ�͵�ƽ
		#endif
		
		#if IF_USE_DJIC
		GPIO_ResetBits(GPIOF, GPIO_Pin_6); // PC5����Ϊ�͵�ƽ
		#endif
	}	
	else
	{
		runTime ++ ;
		if(runTime <= Op_Rate)
		{
			#if IF_USE_V5_V5PLUS_V6
			GPIO_ResetBits(GPIOC, GPIO_Pin_5); // PC5����Ϊ�ߵ�ƽ
			#endif
			
			#if IF_USE_DJIC
			GPIO_SetBits(GPIOF, GPIO_Pin_6); // PC5����Ϊ�͵�ƽ
			#endif
		}
		else
		{
			#if IF_USE_V5_V5PLUS_V6
			GPIO_SetBits(GPIOC, GPIO_Pin_5); // PC5����Ϊ�͵�ƽ
			#endif
			
			
			#if IF_USE_DJIC
			GPIO_ResetBits(GPIOF, GPIO_Pin_6); // PC5����Ϊ�͵�ƽ
			#endif
		}
		
		if(runTime >= 10)
		{
			runTime%=10 ;
		}
	}	
}