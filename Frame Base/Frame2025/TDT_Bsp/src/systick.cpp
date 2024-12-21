/******************************
Description: µÎ´ð¶¨Ê±Æ÷_1msÈÎÎñµ÷¶È
function:
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
	void sysTickInit(void)
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
	void SysTick_Handler(void)   ²úÉú1msÖÐ¶Ï£¬µ÷ÓÃTDT_LOOP()
	¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª
variable:
	uint32_t sysTickUptime   

****************************  */

#include "systick.h" 
#include "schedule.h" 
/***ºê¶¨Òå***/
#define TICK_PER_SECOND 1000
#define TICK_US	(1000000/TICK_PER_SECOND)


//µÎ´ð¶¨Ê±Æ÷¼ÆÊý±äÁ¿ ,49ÌìºóÒç³ö
volatile uint32_t sysTickUptime=0;


/**
  * @brief ³õÊ¼»¯µÎ´ð¶¨Ê±Æ÷,ÅäÖÃÎª1msÖÐ¶Ï
  * @note Èç¹ûÐÞ¸ÄÍâ²¿¾§Õñ,¼ÇµÃÐÞ¸Ä HSE_VALUE£¬PLL_M 
  */
void sysTickInit(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t           cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;//µÎ´ð¶¨Ê±Æ÷1ms´¥·¢Ò»´ÎÖÐ¶Ï
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