#include "judgement.h"
#include "crc.h"

#if USE_JUDGEMENT
Judgement judgement; 
volatile bool juartIdle = true;
static u16 jgmtOfflineCheck = 0;
// 定义消息队列和串口空闲标志


Judgement::Judgement()
{
	recPackage = {
			{ (u8 *)&gameStatus, 							sizeof(GameStatus), 							0x0001 },
			{ (u8 *)&gameResult, 							sizeof(GameResult), 							0x0002 },
			{ (u8 *)&gameRobotHP, 						sizeof(GameRobotHP), 							0x0003 },
			{ (u8 *)&eventData, 							sizeof(EventData), 								0x0101 },
			{ (u8 *)&supplyProjectileAction, 	sizeof(SupplyProjectileAction), 	0x0102 },
			{ (u8 *)&refereeWarning, 					sizeof(RefereeWarning), 					0x0104 },
			{ (u8 *)&dartRemainingTime, 			sizeof(DartRemainingTime), 				0x0105 },
			{ (u8 *)&gameRobotStatus, 				sizeof(GameRobotStatus), 					0x0201 },
			{ (u8 *)&powerHeatData, 					sizeof(PowerHeatData), 						0x0202 },
			{ (u8 *)&gameRobotPos, 						sizeof(GameRobotPos), 						0x0203 },
			{ (u8 *)&buff, 										sizeof(Buff), 										0x0204 },
			{ (u8 *)&aerialRobotEnergy, 			sizeof(AerialRobotEnergy), 				0x0205 },
			{ (u8 *)&robotHurt, 							sizeof(RobotHurt), 								0x0206 },
			{ (u8 *)&shootData, 							sizeof(ShootData), 								0x0207 },
			{ (u8 *)&bulletRemaining, 				sizeof(BulletRemaining), 					0x0208 },
			{ (u8 *)&rfidStatus, 							sizeof(RfidStatus), 							0x0209 },
			{ (u8 *)&dartClientCmd, 					sizeof(DartClientCmd), 						0x020A },
			{ (u8 *)&ground_robot_position_t, sizeof(Ground_robot_position_t), 	0X020B },
			{ (u8 *)&sentry_info_t, 					sizeof(Sentry_info_t), 						0X020D },
			{ (u8 *)&mapCommand_orgin, 				sizeof(mapCommand_orgin), 				0X0303 }
	};
}

/**
 * @brief 队列处理
 */
void Judgement::run_1000hz()
{
	if(jgmtOfflineCheck > 500)//500ms
		judgement.jgmtOffline = 1;
	else
		jgmtOfflineCheck++;
	
	judgement.ringQueue();
	judgement.AutoSend();
}

/**
 * @brief 初始化
 * 
 */
void Judgement::init()
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

#if IF_USE_V5_V5PLUS_V6
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
	
#if IF_USE_DJIC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
#endif

	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &USART_InitStructure);
	USART_Cmd(USART6, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judgeDataBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = JUDGE_BUFFER_LENGTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);
}

/**
 * @brief 填充消息队列
*/
void Judgement::uartSendBytes(uint8_t *data,uint16_t size)
{
	 sendPackage.push({data, size});
}

/**
 * @brief 发送机器人交互链路数据 填写数据包里的 子命令，发送端，接收端，数据长度
 * 
 * @param dataLength 交互内容的长度
 */
void Judgement::robotsCommunication(uint16_t dataCmdId, RobotIdDef robotIdDef, u8 dataLength)
{
	sendUnionData.dataCmdId = dataCmdId;
	sendUnionData.receiverId = robotIdDef;
	int count = sizeof(FrameHeader) + 6 + dataLength + 2;
	sendUnionData.frameHeader.cmdid = 0x0301;
	customSend(count);
}


///**
// * @brief 发送客户端UI命令
// * 
// * @param sendGraphics 一次发送的图形个数
// */
//void Judgement::graphicDraw(u8 sendGraphics)
//{
//	if (sendGraphics <= 1)
//	{
//		sendUnionData.dataCmdId = 0x101;
//		sendGraphics = 1;
//	}
//	else if (sendGraphics <= 2)
//	{
//		sendUnionData.dataCmdId = 0x102;
//		sendGraphics = 2;
//	}
//	else if (sendGraphics <= 5)
//	{
//		sendUnionData.dataCmdId = 0x103;
//		sendGraphics = 5;
//	}
//	else if (sendGraphics <= 7)
//	{
//		sendUnionData.dataCmdId = 0x104;
//		sendGraphics = 7;
//	}
//	sendUnionData.frameHeader.cmdid = 0x0301;

//	int count = sizeof(FrameHeader) + 6 + sendGraphics * sizeof(GraphicDataStruct) + 2;
//	sendUnionData.receiverId = ClientId();
//	sendUnionData.senderId = gameRobotStatus.robotId;

//	customSend(count);
//}

///**
// * @brief 字符绘制
// * 
// */
//void Judgement::characterDraw()
//{
//	sendUnionData.frameHeader.cmdid = 0x0301;
//	sendUnionData.dataCmdId = 0x110;
//	int count = sizeof(FrameHeader) + 6 + sizeof(GraphicDataStruct) + 30 + 2;
//	sendUnionData.receiverId = ClientId();
//	sendUnionData.senderId = gameRobotStatus.robotId;

//	customSend(count);
//}

///**
// * @brief 删除图形
// * 
// */
//void Judgement::graphicDel()
//{
//	sendUnionData.frameHeader.cmdid = 0x0301;
//	sendUnionData.dataCmdId = 0x100;
//	int count = sizeof(FrameHeader) + 6 + sizeof(ClientCustomGraphicDelete) + 2;
//	sendUnionData.receiverId = ClientId();
//	sendUnionData.senderId = gameRobotStatus.robotId;

//	customSend(count);
//}


/*
  发给云台手路径规划
*/

void Judgement::PathSend()
{
	
	pathPlanningData.frameHeader.sof = 0xa5;
	pathPlanningData.frameHeader.dataLength = 105 ;
	pathPlanningData.frameHeader.cmdid = 0x0307 ;
	
	pathPlanningData.sender_id=IdToMate(ID_GUARD);
	
	Append_CRC8_Check_Sum((unsigned char *)&pathPlanningData , 5);
	Append_CRC16_Check_Sum((unsigned char *)&pathPlanningData , sizeof(pathPlanningData));
	
	uartSendBytes((uint8_t*)&pathPlanningData,sizeof(pathPlanningData));
}

/**
* @brief 发送评论区
 * 
 */
void Judgement::messageSend()
{
	custom_info.frameHeader.cmdid = 0x0308;
	int count = sizeof(FrameHeader)+ sizeof(Custom)+2;
	custom_info.frameHeader.sof = 0xa5;
	custom_info.frameHeader.dataLength = count - sizeof(FrameHeader)-2;
	Append_CRC8_Check_Sum((unsigned char *)&custom_info, 5);
	Append_CRC16_Check_Sum((unsigned char *)&custom_info, count);
	uartSendBytes((uint8_t*)&custom_info,sizeof(custom_info));
}

/**
 * @brief 针对0x301的联合体数据进行发送
 * 补充填写帧头，命令，crc16
 * @param count 整个发送数据包的长度
 */
void Judgement::customSend(u8 count)
{
	sendUnionData.frameHeader.sof = 0xa5;									//帧头
	sendUnionData.frameHeader.dataLength = count - sizeof(FrameHeader) - 2; //数据长度
	Append_CRC8_Check_Sum((unsigned char *)&sendUnionData, 5);				//CRC8校验
	
	
	sendUnionData.senderId = gameRobotStatus.robotId;						//发送方ID
 
	if (sendUnionData.receiverId != ClientId() && sendUnionData.receiverId!=0x8080 )						   //不是客户端
		sendUnionData.receiverId = IdToMate(sendUnionData.receiverId); //转化为友方

	Append_CRC16_Check_Sum((unsigned char *)&sendUnionData, count);

	uartSendBytes((uint8_t*)&sendUnionData,sizeof(sendUnionData));
}


/**
 * @brief 将ID枚举转换为友方机器人id
 * 
 * @param robotIdDef ID枚举
 * @return uint16_t 友方机器人id
 */
uint16_t Judgement::IdToMate(RobotIdDef robotIdDef)
{
	if (gameRobotStatus.robotId > RED_BLUE_ID_DIFF)
	{
		return (uint16_t)robotIdDef + RED_BLUE_ID_DIFF;
	}
	return (uint16_t)robotIdDef;
}

/**
 * @brief 将机器人id转换为友方机器人id
 * 
 * @param robotId 有效的机器人id
 * @return uint16_t 友方机器人id
 */
uint16_t Judgement::IdToMate(uint16_t robotId)
{
	return IdToMate(RobotIdDef(robotId % RED_BLUE_ID_DIFF));
}

/**
 * @brief 根据自身id返还客户端id
 * 
 * @return uint16_t 客户端id
 */
uint16_t Judgement::ClientId()
{
	return gameRobotStatus.robotId + ROBOT_CLIENT_ID_DIFF;
}


/**
* @brief 根据机器人id返还友方客户端id
 * 
 * @return uint16_t 客户端id
 */
uint16_t Judgement::ClientId(RobotIdDef robotIdDef)
{
	if (gameRobotStatus.robotId > RED_BLUE_ID_DIFF)
	{
		return (uint16_t)robotIdDef + RED_BLUE_ID_DIFF+ROBOT_CLIENT_ID_DIFF;
	}
	return (uint16_t)robotIdDef+ROBOT_CLIENT_ID_DIFF;
	
}

/**
 * @brief 将ID枚举转换为敌方机器人id
 * 
 * @param robotIdDef ID枚举
 * @return uint16_t 敌方机器人id
 */
uint16_t Judgement::IdToEnemy(RobotIdDef robotIdDef)
{
	if (gameRobotStatus.robotId < RED_BLUE_ID_DIFF)
	{
		return (uint16_t)robotIdDef + RED_BLUE_ID_DIFF;
	}
	return robotIdDef;
}

/**
 * @brief 将机器人id转换为敌方机器人id
 * 
 * @param robotId 有效的机器人id
 * @return uint16_t 敌方机器人id
 */
uint16_t Judgement::IdToEnemy(uint16_t robotId)
{
	return IdToEnemy(RobotIdDef(robotId % RED_BLUE_ID_DIFF));
}


//**************************************以下为发送配置部分
void Judgement::AutoSend(void)
{
	if ( !sendPackage.empty()&& juartIdle)
		{ 
			SendMessage msg2=sendPackage.front_element();
			sendPackage.pop();
	    DMA_Send(msg2.data,msg2.size);
   }	
}

/**
 * @brief 发送多字节数据（采用DMA）
 * 
 * @param ptr 地址
 * @param len 长度
 */

void Judgement::DMA_Send(uint8_t *data,uint16_t size)
{
	static DMA_InitTypeDef DMA_InitStructure;
	static NVIC_InitTypeDef NVIC_InitStructure;
	static u8 firstLoad = 1; //首次加载时加载配置缺省值
	if (firstLoad == 1)
	{
		firstLoad = 0;

		DMA_InitStructure.DMA_Channel = DMA_Channel_5; // 配置为 USART6 的 DMA 通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(USART6->DR); // 外设基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // 数据传输方向：内存到外设

    // 配置 DMA 缓存大小和地址递增模式
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // 内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 数据宽度为八位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // 数据宽度为八位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; // 工作模式为正常
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; // DMA 通道拥有高优先级

    // NVIC 配置
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn; // 修改为 DMA2_Stream6 的中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 设定抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 设定子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能中断
		DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
    NVIC_Init(&NVIC_InitStructure);
	}

    // 设置 DMA 内存基地址和缓存大小
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)data; // 内存基地址
    DMA_InitStructure.DMA_BufferSize = size; // DMA 缓存大小

    // 清除 DMA 标志位
    DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6); // 清除传输完成标志位

    // 初始化 DMA2 Stream6
    DMA_Init(DMA2_Stream6, &DMA_InitStructure);
    
    // 启用 DMA2 Stream6
    DMA_Cmd(DMA2_Stream6, ENABLE);
		juartIdle = false;
}


extern "C" void DMA2_Stream6_IRQHandler(void) 
{
    if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6)) // 修改为 Stream6 的中断标志位
    {
        DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6); // 清除中断挂起位
        DMA_ClearFlag(DMA2_Stream6, DMA_IT_TCIF6); // 清除 Stream6 的标志位
        juartIdle = true; // 设置 UART 空闲状态      
    }
}


//**********************************************************接收部分
/**
 * @brief 接收DMA定义
 * @note  由于裁判系统发送一帧的中间会停止发送，故不能用空闲中断+DMA进行读取
 * 		  而使用DMA循环接收+各个字节轮询遍历的方法
 */
extern "C" void DMA2_Stream1_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1))
	{
		jgmtOfflineCheck = 0;
		judgement.jgmtOffline = 0;
		judgement.addFullCount(); //说明到末尾了，需要进行过圈处理

		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
	}
}



static u8 CRC8_Check = 0xff;		  //参与CRC8校验的计算变量
static uint16_t CRC16_Check = 0xffff; //参与CRC16校验的计算变量
static CRC_chose crcCalcState = CRC_NO_CALC_STATUS;			  //CRC判断到底进行什么校验

static int64_t read_len;	  //已经处理的长度
static int64_t rx_len;		  //已经读取的长度
static uint16_t wholePackLen; //当前解算的帧的总长度
static int32_t index = 0;	  //当前解算的帧在fullDataBuffer的索引
static u8 lastSeq;			  //上一次包序号

void Judgement::ringQueue()
{
	rx_len = JUDGE_BUFFER_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1) + judgementFullCount * JUDGE_BUFFER_LENGTH; //获取裁判当前已经接收的长度
	if (rx_len > read_len + JUDGE_BUFFER_LENGTH + 1)
		wrongStatusCnt.CYCLE_Wrong_cnt++; //如果读取的长度比接收的长度短一圈（少JUDGE_BUFFER_LENGTH），认为已经过圈，过圈计数器加一

	while (rx_len > read_len + 1) //当没读满时
	{
		 
		u8 byte = judgeDataBuffer[ read_len % JUDGE_BUFFER_LENGTH ];		   //当前应该读取的字节在judgeDataBuffer的位置,将该字节取出
		read_len++; //认为该字节已经读取，位置往后移
		
		/* 独立计算CRC，平摊算力 */
		if (crcCalcState == CRC8_CRC16_CALC_STATUS)
		{
			CRC8_Check = CRC8_TAB[CRC8_Check ^ (byte)];
			CRC16_Check = ((uint16_t)(CRC16_Check) >> 8) ^ wCRC_Table[((uint16_t)(CRC16_Check) ^ (uint16_t)(byte)) & 0x00ff];
		}

		if (crcCalcState == CRC16_CALC_STATUS)
			CRC16_Check = ((uint16_t)(CRC16_Check) >> 8) ^ wCRC_Table[((uint16_t)(CRC16_Check) ^ (uint16_t)(byte)) & 0x00ff];
		
		
		u8 dataLenFromCmdid;
		
		switch (judgementStep)
		{
		case STEP_HEADER: //帧头
			if (index == 0 && byte != 0xA5)
			{
				resetParseState();
				break;
			}
			
			fullDataBuffer[index++] = byte;
			if (index == 4) //包序号
			{
					lastSeq++;
					if (((FrameHeader *)fullDataBuffer)->seq != lastSeq) //上一帧的包序号不等于这一帧的包序号+1
					{
						wrongStatusCnt.SEQ_Wrong_cnt++; //认为包序号错误，计数器+1
						lastSeq = byte;
					}
					//包序号错误仍进行处理
					crcCalcState = CRC16_CALC_STATUS; //下一帧起不再计算CRC8
					judgementStep = STEP_HEADER_CRC8;
			}			
			break;
		case STEP_HEADER_CRC8:
			fullDataBuffer[index++] = byte;
			if (((FrameHeader *)fullDataBuffer)->crc8 != CRC8_Check) //CRC8校验错误
			{
				wrongStatusCnt.CRC8_Wrong_cnt++;	   //CRC8校验错误计数器+1
				resetParseState();
				break;
			}
			judgementStep = STEP_CMDID_GET; //获取命令码
			CRC8_Check = 0xff;				//初始化CRC8校验
			break;
		case STEP_CMDID_GET:
			fullDataBuffer[index++] = byte;
			if (index == 7) //未获取完命令码
			{
				dataLenFromCmdid= getLength((FrameHeader *)fullDataBuffer);
				wholePackLen = 7 + getLength((FrameHeader *)fullDataBuffer) + 2;
				if (dataLenFromCmdid == 0xFF || dataLenFromCmdid != ((FrameHeader *)fullDataBuffer)->dataLength) //命令错误
					resetParseState();
				else
					judgementStep = STEP_DATA_TRANSFER;
			}
			break;
		case STEP_DATA_TRANSFER:
			fullDataBuffer[index++] = byte;
			if (index == (wholePackLen - 2)) //数据传输完，待校验
			{
				crcCalcState = CRC_NO_CALC_STATUS; //下帧起不计算CRC16，避免将帧尾的校验位也计算
				judgementStep = STEP_DATA_CRC16;
			}
			break;
		case STEP_DATA_CRC16:
			fullDataBuffer[index++] = byte;
			if (index == (wholePackLen))
			{
				uint8_t CRC16_char[2];
				CRC16_char[0] = (u8)(CRC16_Check & 0x00ff);
				CRC16_char[1] = (u8)((CRC16_Check >> 8) & 0x00ff);
				if (CRC16_char[0] == fullDataBuffer[index - 2] && CRC16_char[1] == fullDataBuffer[index - 1]) //CRC16校验通过
					getJudgeData();
				else
					wrongStatusCnt.CRC16_Wrong_cnt++; //CRC16校验失败则计数器+1

				//无论是否通过都重置
				resetParseState();
			}
			break;
		default:
			resetParseState();
			break;
		}
		rx_len = JUDGE_BUFFER_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1) + 
			judgementFullCount * JUDGE_BUFFER_LENGTH; //重新获取已经接收的长度
	}
	if (rx_len % JUDGE_BUFFER_LENGTH > (JUDGE_BUFFER_LENGTH / 3) && 
		rx_len % JUDGE_BUFFER_LENGTH < (2 * JUDGE_BUFFER_LENGTH / 3)) //防止judgementFullCount溢出，过早清除与过晚清除都可能会导致包圈
	{
		read_len -= JUDGE_BUFFER_LENGTH * judgementFullCount;
		judgementFullCount = 0;
	}
}

void Judgement::resetParseState() {
    judgementStep = STEP_HEADER;
    index = 0;
    crcCalcState = CRC8_CRC16_CALC_STATUS;
    CRC8_Check = 0xff;
    CRC16_Check = 0xffff;
}

/**
 * @brief 获取当前帧对应的长度
 * @param frameHeader 帧头结构体
 * @return uint8_t 长度
 */
uint8_t Judgement::getLength(FrameHeader *frameHeader)
{
	 for (int i=0; i<recPackage.size(); i++)
    {
        if (recPackage[i].id == frameHeader->cmdid)
        {
            return recPackage[i].size; // 返回对应的长度
        }
    }
    return 0xff; 
}

/**
 * @brief 结构体复制
 * 
 */
void Judgement::getJudgeData()
{
	static NVIC_InitTypeDef NVIC_InitStructure;
	static DMA_InitTypeDef DMA_InitStructure;
	static u8 firstLoad = 1; //首次加载时加载配置缺省值
	if (firstLoad == 1)
	{
		firstLoad = 0;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)fullDataBuffer + sizeof(FrameHeader); //外设基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;										   //数据传输方向：外设到内存
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;							   //外设地址递增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									   //内置地址递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;					   //数据宽度为八位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;							   //数据宽度为八位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;											   //不执行循环模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;										   //dma通道拥有高优先级

		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, DISABLE); //失能传输完成中断中断
	}
	
	 unsigned char *judgeData_ADD= nullptr;
	 for (int i=0; i<recPackage.size(); i++)
    {
        if (recPackage[i].id == ((FrameHeader *)fullDataBuffer)->cmdid)
        {
            judgeData_ADD = recPackage[i].data; // 获取数据地址
            break;
        }
    }

	DMA_DeInit(DMA2_Stream3);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judgeData_ADD;			 //内存基地址
	DMA_InitStructure.DMA_BufferSize = getLength((FrameHeader *)fullDataBuffer); //dma缓存大小
	DMA_Init(DMA2_Stream3, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream3, ENABLE);
	NVIC_Init(&NVIC_InitStructure);
}

#endif