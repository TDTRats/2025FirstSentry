/******************************
Description: �δ�ʱ��_1ms�������
function:
	����������������������������������������������������������������������������������������������������������������������������������������������������
	void sysTickInit(void)
	����������������������������������������������������������������������������������������������������������������������������������������������������
	void SysTick_Handler(void)   ����1ms�жϣ�����TDT_LOOP()
	����������������������������������������������������������������������������������������������������������������������������������������������������
variable:
	uint32_t sysTickUptime   

****************************  */

#include "systick.h" 
#include "schedule.h" 
/***�궨��***/
#define TICK_PER_SECOND 1000
#define TICK_US	(1000000/TICK_PER_SECOND)


//�δ�ʱ���������� ,49������
volatile uint32_t sysTickUptime=0;


/**
  * @brief ��ʼ���δ�ʱ��,����Ϊ1ms�ж�
  * @note ����޸��ⲿ����,�ǵ��޸� HSE_VALUE��PLL_M 
  */
void sysTickInit(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t           cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;//�δ�ʱ��1ms����һ���ж�
	//cnts=168000/8;=1ms
	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}


extern u8 Init_OK;
void SysTick_Handler(void)
{
	sysTickUptime++;
	if (Init_OK)
	{
		TDT_Loop();
	}
}