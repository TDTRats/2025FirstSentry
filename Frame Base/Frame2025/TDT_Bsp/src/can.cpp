/******************************
File name: TDT_Bsp\src\can.cpp
Description: can底层
function:
	——————————————————————————————————————————————————————————————————————————

	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.16 #合成can1.c和can2.c #修改接收函数，从队列传递改为直接解算
	——————————————————————————————————————————————————————————————————————————
	19.11.12 #首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "can.h"
#include "motor.h"
#include "can_calculate.h"
#include "dbus.h"
#include "info_task.h"
#include "vision.h"
#include "power.h"
#include "lk_motor.h"
/**
 * @ingroup TDT_BSP
 * @defgroup TDT_BSP_CAN CAN
 * @brief 提供了包括大疆电机支持的can配置、发送和接受
 * @{
 */

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/
CanRxMsg Can1RxMsg;
CanRxMsg Can2RxMsg;

///@}

/**
  * @brief can的GPIO和NVIC初始化
  * @param[in] can_x can口:CAN1/CAN2
  */
static void canGpioNvicInit(CAN_TypeDef *can_x)
{
	GPIO_InitTypeDef CanGpio; //CAN GPIO初始化结构体
	NVIC_InitTypeDef CanNvic; //CAN 中断结构体

	/*时钟初始化*/
	if (can_x == CAN1)
	{
		
#if IF_USE_V5_V5PLUS_V6
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //初始化GPIO时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  //初始化CNA1时钟
		//GPIO复用
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);
		/*GPIO初始化*/
		CanGpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
		CanGpio.GPIO_Mode = GPIO_Mode_AF;
		CanGpio.GPIO_OType = GPIO_OType_PP;
		CanGpio.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOB, &CanGpio);
#endif
#if IF_USE_DJIC
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); // 使能 GPIOD 时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);  // 使能 CAN1 时钟
		
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1); // PD0 复用为 CAN1 RX
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); // PD1 复用为 CAN1 TX
		
		CanGpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // PD0 和 PD1
		CanGpio.GPIO_Mode = GPIO_Mode_AF;           // 复用功能
		CanGpio.GPIO_OType = GPIO_OType_PP;         // 推挽输出
		CanGpio.GPIO_Speed = GPIO_Speed_100MHz;     // 100MHz
		GPIO_Init(GPIOD, &CanGpio);
		
		
		
#endif

		/*中断初始化*/
		CanNvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
		CanNvic.NVIC_IRQChannelPreemptionPriority = 3;
		CanNvic.NVIC_IRQChannelSubPriority = 1;
		CanNvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&CanNvic);

		CanNvic.NVIC_IRQChannel = CAN1_TX_IRQn;
		CanNvic.NVIC_IRQChannelPreemptionPriority = 1;
		CanNvic.NVIC_IRQChannelSubPriority = 1;
		CanNvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&CanNvic);
	}
	else if (can_x == CAN2)
	{
#if IF_USE_V5_V5PLUS_V6
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);

		CanGpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12;
		CanGpio.GPIO_Mode = GPIO_Mode_AF;
		//	CanGpio.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(GPIOB, &CanGpio);
#endif
		
#if IF_USE_DJIC
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能 GPIOB 时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);  // 使能 CAN2 时钟

		// 配置 PB5 和 PB6 为 CAN2 的复用功能
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2); // PB5 复用为 CAN2 RX
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2); // PB6 复用为 CAN2 TX

		// 配置 PB5 和 PB6 引脚的属性
		GPIO_InitTypeDef CanGpio;
		CanGpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; // 配置 PB5 和 PB6
		CanGpio.GPIO_Mode = GPIO_Mode_AF;           // 配置为复用功能
		GPIO_Init(GPIOB, &CanGpio);
		
#endif

		CanNvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
		CanNvic.NVIC_IRQChannelPreemptionPriority = 0;
		CanNvic.NVIC_IRQChannelSubPriority = 0;
		CanNvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&CanNvic);

		CanNvic.NVIC_IRQChannel = CAN2_TX_IRQn;
		CanNvic.NVIC_IRQChannelPreemptionPriority = 0;
		CanNvic.NVIC_IRQChannelSubPriority = 0;
		CanNvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&CanNvic);
	}
}

CAN_InitTypeDef Can;			 //CNA初始化结构体
CAN_FilterInitTypeDef CanFilter; //CNA过滤器初始化结构体
void canInit(CAN_TypeDef *can_x)
{
	//can的GPIO和NVIC初始化
	canGpioNvicInit(can_x);

	

	/*CAN初始化*/
	CAN_DeInit(can_x);	  //将外设 CAN 的全部寄存器重设为缺省值
	CAN_StructInit(&Can); //把 Can 结构体中的每一个参数按缺省值填入

	Can.CAN_TTCM = DISABLE;			// 时间触发通讯模式， ENABLE/DISABLE。
	Can.CAN_ABOM = ENABLE;			//  自动离线管理，  ENABLE/DISABLE。
	Can.CAN_AWUM = ENABLE;			// 自动唤醒模式， ENABLE/DISABLE。
	Can.CAN_NART = DISABLE;			//  非自动重传输模式， ENABLE/DISABLE。
	Can.CAN_RFLM = DISABLE;			//  接收FIFO锁定模式  ENABLE/DISABLE。
	Can.CAN_TXFP = ENABLE;			// 发送FIFO优先级， ENABLE/DISABLE。
	Can.CAN_Mode = CAN_Mode_Normal; //  CAN硬件工作模式，可选：CAN_Mode_Normal 正常模式； CAN_Mode_Silent 静默模式； CAN_Mode_LoopBack环回模式； CAN_Mode_Silent_LoopBack静默环回模式 。
#if   IF_USE_V5_V5PLUS_V6
	Can.CAN_SJW = CAN_SJW_1tq;		// 重新同步跳跃宽度(SJW)，范围CAN_SJW_1tq到CAN_SJW_4tq
	Can.CAN_BS1 = CAN_BS1_9tq;		// 时间段 1 的时间单位数目，范围CAN_BS1_1tq到CAN_BS1_16tq
	Can.CAN_BS2 = CAN_BS2_4tq;		// 时间段 2 的时间单位数目，范围CAN_BS2_1tq到CAN_BS2_8tq
	Can.CAN_Prescaler = 3;			// 一个时间单位的长度，范围 1 到 1024//42/(1+9+4)/3=1Mbps
#endif 
#if  IF_USE_DJIC
	Can.CAN_SJW = CAN_SJW_1tq;		// 重新同步跳跃宽度(SJW)，范围CAN_SJW_1tq到CAN_SJW_4tq
	Can.CAN_BS1 = CAN_BS1_10tq;		// 时间段 1 的时间单位数目，范围CAN_BS1_1tq到CAN_BS1_16tq
	Can.CAN_BS2 = CAN_BS2_3tq;		// 时间段 2 的时间单位数目，范围CAN_BS2_1tq到CAN_BS2_8tq
	Can.CAN_Prescaler = 3;			// 一个时间单位的长度，范围 1 到 1024//42/(1+9+4)/3=1Mbps
#endif
	CAN_Init(can_x, &Can);			//根据Can的参数初始化CAN1
	/*CAN过滤器配置*/
	if (can_x == CAN1)
	{
		CanFilter.CAN_FilterNumber = 0; //待初始化的过滤器，范围
	}
	else
	{
		CanFilter.CAN_FilterNumber = 14; //待初始化的过滤器，范围
	}

	CanFilter.CAN_FilterMode = CAN_FilterMode_IdMask;  //过滤器模式：CAN_FilterMode_IdMask 标识符屏蔽位模式 CAN_FilterMode_IdList 标识符列表模式
	CanFilter.CAN_FilterScale = CAN_FilterScale_32bit; //过滤器位宽：CAN_FilterScale_Two16bit 2 个 16 位过滤器 CAN_FilterScale_One32bit 1 个 32 位过滤器
	CanFilter.CAN_FilterIdHigh = 0x0000;			   //过滤器标识符（32 位位宽时为其高段位，16 位位宽时为第一个）。它的范围是 0x0000 到 0xFFFF
	CanFilter.CAN_FilterIdLow = 0x0000;				   //过滤器标识符（32 位位宽时为其低段位，16 位位宽时为第二个）。它的范围是 0x0000 到 0xFFFF
	CanFilter.CAN_FilterMaskIdHigh = 0x0000;		   //过滤器屏蔽标识符或者过滤器标识符（32 位位宽时为其高段位，16 位位宽时为第一个）。它的范围是 0x0000 到 0xFFFF。
	CanFilter.CAN_FilterMaskIdLow = 0x0000;			   //过滤器屏蔽标识符或者过滤器标识符（32 位位宽时为其低段位，16 位位 宽时为第二个）。它的范围是 0x0000 到 0xFFFF
	CanFilter.CAN_FilterFIFOAssignment = 0;			   //
	CanFilter.CAN_FilterActivation = ENABLE;		   //使能或者失能过滤器
	CAN_FilterInit(&CanFilter);						   //
	//使能或者失能指定的 CAN 中断
	CAN_ITConfig(can_x, CAN_IT_FMP0, ENABLE); //FIFO0消息挂号中断允许,每接收一次进一次中断
	CAN_ITConfig(can_x, CAN_IT_TME, ENABLE);  //
	
}


struct SelfCanFIFO
{
	SelfCanFIFO(CAN_TypeDef *can_x):can_x(can_x){};//构造器
	u8 push(CanTxMsg _canTxMsg);//往队列传入一个
	void trySend();//尝试发送
	inline u8 getFIFOCount(){return (rear - front + FIFOLenth)  %  FIFOLenth;};//获取队列个数
	inline u8 getFIFOLenth(){return FIFOLenth;}//获取队列容量
	inline CAN_TypeDef *getCanX(){return can_x;};//获取can
private:
	CAN_TypeDef *can_x;
	CanTxMsg canTxMsgList[8]={0};
	const u8 FIFOLenth = sizeof(canTxMsgList)/sizeof(CanTxMsg);
	u8 front = 0;
	u8 rear = 0;
	u8 FIFO_fullFlag = 0;
	u8 pop(CanTxMsg *);
};


u8 SelfCanFIFO::push(CanTxMsg _canTxMsg)
{
	u8 nextNear = (rear+1)%FIFOLenth;//如果压栈成功的队列头
	if(front == nextNear)//过圈
	{
		FIFO_fullFlag = 1;
		trySend();
		return 0;//邮箱满
	}
	canTxMsgList[rear] = _canTxMsg;
	rear = nextNear;
	trySend();
	return 1;
}

u8 SelfCanFIFO::pop(CanTxMsg *canTxMsg)
{
	if(front == rear)
	{
		return 0;
	}
	if(canTxMsg != NULL)
		*canTxMsg = canTxMsgList[front];
	front=(front+1)%FIFOLenth;//如果出栈成功的队列尾
	return 1;
}

void SelfCanFIFO::trySend()
{
	/* 禁止全局中断*/
	__disable_irq();
	while(1)
	{
		if(getFIFOCount() == 0)//头和尾相同
			break;
		if(CAN_Transmit(can_x, canTxMsgList + front) == CAN_TxStatus_NoMailBox)//无空邮箱
			break;
		pop(NULL);//弹栈
	}
	/*  使能全局中断 */
	__enable_irq();

}

SelfCanFIFO selfCanFIFO[2]{SelfCanFIFO(CAN1), SelfCanFIFO(CAN2)};


u8 self_CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage)
{

	/* Check the parameters */
	assert_param(IS_CAN_ALL_PERIPH(CANx));
	assert_param(IS_CAN_IDTYPE(TxMessage->IDE));
	assert_param(IS_CAN_RTR(TxMessage->RTR));
	assert_param(IS_CAN_DLC(TxMessage->DLC));

	if(CANx == CAN1)
	{
		return selfCanFIFO[0].push(*TxMessage);
	}
	return selfCanFIFO[1].push(*TxMessage);
}
/**
  * @brief CAN发送中断
  */
void CAN1_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
	{
		selfCanFIFO[0].trySend();
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
	}
}

void CAN2_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2, CAN_IT_TME) != RESET)
	{
		selfCanFIFO[1].trySend();
		CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
	}
}

/**
  * @brief CAN接收中断
  * @note 已经自动输入到对应电机，如果由特殊ID的消息，可以自己再switch
  */
DJI_Motor dji_motor_receive;

extern LK_Motor lkmotor;
void CAN1_RX0_IRQHandler(void)
{
		
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &Can1RxMsg);
		//CAN信息处理
		dji_motor_receive.Dji_Motor_Calculate(Can_1, &Can1RxMsg);
		infoProcess.ReceiveManage(Can1RxMsg,CAN1);
		LKList_Receive(Can_1, &Can1RxMsg);
		#if USE_POWER
			if(power.whereIsPower==onCAN1)
				power.myPowerCanRx(Can1RxMsg);
		#endif
		switch (Can1RxMsg.StdId)
		{
		default:
			break;
		}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	}
}

void CAN2_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
	{
		CAN_Receive(CAN2, CAN_FIFO0, &Can2RxMsg);
		//CAN信息处理
		dji_motor_receive.Dji_Motor_Calculate(Can_2, &Can2RxMsg);
		infoProcess.ReceiveManage(Can2RxMsg,CAN2);
		LKList_Receive(Can_2, &Can2RxMsg);
		#if USE_POWER
			if(power.whereIsPower==onCAN2)
				power.myPowerCanRx(Can2RxMsg);
		#endif
		switch (Can2RxMsg.StdId)
		{
		default:
			break;
		}
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	}
}

/**
  * @brief  can1发送数据
  * @param[in] value 四维数据结构体
  * @param[in] can_x CAN:CAN1/CAN2
  *	@param[in] id ID
  */
void canTx(vec4f *value, CAN_TypeDef *can_x, uint32_t id)
{
	CanTxMsg Can1TxMsg;

	Can1TxMsg.IDE = CAN_Id_Standard; //标准帧 CAN_Id_Standard 使用标准标识符 CAN_Id_Extended 使用标准标识符 + 扩展标识符
	Can1TxMsg.RTR = CAN_RTR_Data;	 //数据帧 CAN_RTR_Data 数据帧 CAN_RTR_Remote 远程帧
	Can1TxMsg.DLC = 8;				 //帧长度 范围是 0 到 0x8

	Can1TxMsg.StdId = id; //范围为 0 到 0x7FF

	Can1TxMsg.Data[0] = (u8)((int16_t)value->data[0] >> 8);
	Can1TxMsg.Data[1] = (u8)(value->data[0]);
	Can1TxMsg.Data[2] = (u8)((int16_t)value->data[1] >> 8);
	Can1TxMsg.Data[3] = (u8)(value->data[1]);
	Can1TxMsg.Data[4] = (u8)((int16_t)value->data[2] >> 8);
	Can1TxMsg.Data[5] = (u8)(value->data[2]);
	Can1TxMsg.Data[6] = (u8)((int16_t)value->data[3] >> 8);
	Can1TxMsg.Data[7] = (u8)(value->data[3]);
	u8 mbox = self_CAN_Transmit(can_x, &Can1TxMsg);
}

void canTx(float *data, CAN_TypeDef *can_x, uint32_t id)
{
	CanTxMsg Can1TxMsg;

	Can1TxMsg.IDE = CAN_Id_Standard; //标准帧 CAN_Id_Standard 使用标准标识符 CAN_Id_Extended 使用标准标识符 + 扩展标识符
	Can1TxMsg.RTR = CAN_RTR_Data;	 //数据帧 CAN_RTR_Data 数据帧 CAN_RTR_Remote 远程帧
	Can1TxMsg.DLC = 8;				 //帧长度 范围是 0 到 0x8

	Can1TxMsg.StdId = id; //范围为 0 到 0x7FF

	Can1TxMsg.Data[0] = (u8)((int16_t)*data >> 8);
	Can1TxMsg.Data[1] = (u8)(*data);
	Can1TxMsg.Data[2] = (u8)((int16_t) * (data + 1) >> 8);
	Can1TxMsg.Data[3] = (u8)(*(data + 1));
	Can1TxMsg.Data[4] = (u8)((int16_t) * (data + 2) >> 8);
	Can1TxMsg.Data[5] = (u8)(*(data + 2));
	Can1TxMsg.Data[6] = (u8)((int16_t) * (data + 3) >> 8);
	Can1TxMsg.Data[7] = (u8)(*(data + 3));
	u8 mbox = self_CAN_Transmit(can_x, &Can1TxMsg);
}

void canTx(u8 data[8], CAN_TypeDef *can_x, uint32_t id)
{
	CanTxMsg Can1TxMsg;

	Can1TxMsg.IDE = CAN_Id_Standard; //标准帧 CAN_Id_Standard 使用标准标识符 CAN_Id_Extended 使用标准标识符 + 扩展标识符
	Can1TxMsg.RTR = CAN_RTR_Data;	 //数据帧 CAN_RTR_Data 数据帧 CAN_RTR_Remote 远程帧
	Can1TxMsg.DLC = 8;				 //帧长度 范围是 0 到 0x8

	Can1TxMsg.StdId = id; //范围为 0 到 0x7FF

	memcpy(Can1TxMsg.Data, data, 8);

	u8 mbox = self_CAN_Transmit(can_x, &Can1TxMsg);
}