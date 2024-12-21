/******************************
Description: 定时器_获取程序运行时间Us
function:
	——————————————————————————————————————————————————————————————————————————
	void ConfigureTimeForRunTimeStats(void)
	——————————————————————————————————————————————————————————————————————————
	uint64_t getSysTimeUs(void)                       获取us级时间
	——————————————————————————————————————————————————————————————————————————
	float timeIntervalFrom_f(float timeFrom)				  从timeFrom到目前的时间间隔，单位秒
	——————————————————————————————————————————————————————————————————————————
	uint64_t timeIntervalFrom(uint64_t timeFrom)      从timeFrom到目前的时间间隔，单位毫秒
	——————————————————————————————————————————————————————————————————————————
	void delayUs(uint32_t us)													US级延时
	——————————————————————————————————————————————————————————————————————————
	void delayMs(uint32_t ms)                         MS级延时
	——————————————————————————————————————————————————————————————————————————
	uint32_t RunAndGetTime(void (*taskFunc)(void))    运行并调用 只能调用全局或者静态函数
	——————————————————————————————————————————————————————————————————————————
	uint32_t RunAndGetTimee(std::function<void()> fuc); 可以调用类函数，放到function绑定
	——————————————————————————————————————————————————————————————————————————
	@warning 尽量不要使用TIM2来作为其他外设，除非重装载值和预分频都一样
****************************  */


#include "timer.h"
#include "led.h"

///时间统计所用的节拍计数器
volatile uint32_t runTimeTicks;
static void TIM2_Init(u32 arr, u16 psc);

/**
  * @brief 初始化TIM2使其为时间统计提供时基
  */
void ConfigureTimeForRunTimeStats(void)
{
	//定时器3初始化，定时器时钟为84M，分频系数为84-1，所以定时器3的频率
	//为84M/84=1M，自动重装载为3600000000-1，那么定时器周期就是1小时
	runTimeTicks = 0;
	TIM2_Init(3600000000 - 1, 84 - 1); //初始化TIM2
}

/**
  * @brief 定时器2中断初始化
  * @param[in] arr 自动重装值
  * @param[in] psc 时钟预分频数
  * @note 时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
			Ft=定时器工作频率,单位:Mhz
  */
static void TIM2_Init(u32 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); ///使能TIM2时钟

	TIM_TimeBaseInitStructure.TIM_Period = arr;						//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc;					//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure); //初始化TIM2

	TIM_Cmd(TIM2, ENABLE);						//使能定时器3
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	//允许定时器3更新中断
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //清除中断标志位

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				 //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;		 //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief 定时器2中断服务函数
  */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) //溢出中断
	{
		runTimeTicks++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //清除中断标志位
	}
}

/** @} */

/**
 * @ingroup TDT_SCHEDULE_API
 * @brief 获取us级时间
 * @return us级时间
 * @note 此版本已经将定时器从滴答定时器改为定时器2
 * @warning - 必须先调用TIM2_Init
 */
uint64_t getSysTimeUs(void)
{
	uint64_t hour;
	uint64_t value;
	if ((TIM2->SR & TIM_IT_Update) == SET) //溢出标志位
	{
		runTimeTicks++;
		TIM2->SR = (uint16_t)~TIM_IT_Update; //清除标志位
	}
	hour = runTimeTicks;
	value = hour * 3600000000 + TIM2->CNT;
	return value;
}

/**
 * @brief 从timeFrom到目前的时间间隔，单位秒
 * @param timeFrom 
 * @return float 
 */
float timeIntervalFrom_f(float timeFrom)
{
	return float(getSysTimeUs()) / 1e6f - timeFrom;
}

/**
 * @brief 从timeFrom到目前的时间间隔，单位毫秒
 * @param timeFrom 
 * @return uint64_t 
 */
uint64_t timeIntervalFrom(uint64_t timeFrom)
{
	return getSysTimeUs() - timeFrom;
}

/**
  * @brief US级延时
  * @param us 延时时长，单位US
  * @note 使用滴答延时会造成极大的开销，时间越长，开销越大，尽量在1毫秒以内
  * @warning 此函数在会算入CPU_usage中
  */
void delayUs(uint32_t us)
{
	uint64_t from = getSysTimeUs();
	while(!(getSysTimeUs() > us + from)){}
}

/**
  * @brief MS级延时
  * @param ms 延时时长，单位MS
  * @return
  * @note 注意在操作系统运行下使用此函数务必进入临界区，否则精度不保证
  */
void delayMs(uint32_t ms)
{
	delayUs(ms*1000);
}


uint32_t RunAndGetTime(void (*taskFunc)(void))
{
	uint64_t startTimeStamp = getSysTimeUs();
	taskFunc();
	uint64_t endTimeStamp = getSysTimeUs();
	return (endTimeStamp-startTimeStamp);
}





/**
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*/
