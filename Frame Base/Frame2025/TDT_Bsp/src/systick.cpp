/******************************
Description: 砧基協扮匂_1ms販暦距業
function:
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
	void sysTickInit(void)
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
	void SysTick_Handler(void)   恢伏1ms嶄僅��距喘TDT_LOOP()
	！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
variable:
	uint32_t sysTickUptime   

****************************  */

#include "systick.h" 
#include "schedule.h" 
/***崎協吶***/
#define TICK_PER_SECOND 1000
#define TICK_US	(1000000/TICK_PER_SECOND)


//砧基協扮匂柴方延楚 ,49爺朔吝竃
volatile uint32_t sysTickUptime=0;


/**
  * @brief 兜兵晒砧基協扮匂,塘崔葎1ms嶄僅
  * @note 泌惚俐個翌何唱尅,芝誼俐個 HSE_VALUE��PLL_M 
  */
void sysTickInit(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t           cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;//砧基協扮匂1ms乾窟匯肝嶄僅
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