#ifndef _VISION_H
#define _VISION_H
#include "board.h"

#pragma pack(1) //自行添加数据包需要一字节对齐，添加在两行pack之间
///主控发送（MCU->NUC）.
struct vision_Send_Struct_t
{
	uint8_t header = 0xA5; //包头，默认0XA5
	uint8_t type = 3; //用于在多收/多发串口中区分数据包类别

	float yaw; //yaw角度
	float pitch; //pitch角度
	/*
	根据实际需求自行增减串口数据包内容，确保上位机和下位机做出同步的修改
	*/
	float time_stamp; //时间戳，用于双方时间同步
	int16_t frame_id; //包序号
	uint16_t CRC16CheckSum; //校验码
};

///主控接收（NUC->MCU）.
struct vision_Recv_Struct_t
{
	uint8_t header = 0xA5; //包头，默认0XA5
	uint8_t type = 2; //用于在多收/多发串口中区分数据包类型

	float yaw; // yaw角度
	float pitch; // pitch角度
	/*
	根据实际需求自行增减串口数据包内容，确保上位机和下位机做出同样的修改
	*/
	uint8_t no_obj : 1; //无目标的标志位
	uint8_t beat : 1; //开火指令
	double time_stamp; //视觉时间戳，用于时间同步
	int16_t frame_id; //包序号
	uint16_t CRC16CheckSum; //校验码

};
 
#pragma pack()


class USART_DMA {
public:
    USART_DMA( GPIO_TypeDef* txPort, uint16_t txPin, uint8_t txPinSource, uint8_t txAF,
               GPIO_TypeDef* rxPort, uint16_t rxPin, uint8_t rxPinSource, uint8_t rxAF,
               USART_TypeDef* usart, uint32_t baudRate, 
               uint32_t dmaChannelRx, DMA_Stream_TypeDef* dmaStreamRx,
               uint32_t dmaChannelTx, DMA_Stream_TypeDef* dmaStreamTx,
               IRQn_Type irqnRx, IRQn_Type irqnTx,uint32_t TX_Flag,int32_t TX_IT_Flag,
               uint32_t clockGPIO, uint32_t clockUSART, uint32_t clockDMA,
               uint8_t* recvBuffer, uint32_t bufferSize)  //注意传入的buf的地址
    {
			this->txPort = txPort; this->txPin = txPin; this->txPinSource = txPinSource; this->txAF = txAF;
			this->rxPort = rxPort; this->rxPin = rxPin; this->rxPinSource = rxPinSource; this->rxAF = rxAF;
			this->usart = usart; this->baudRate = baudRate;

			this->dmaChannelRx = dmaChannelRx; this->dmaStreamRx = dmaStreamRx; 
			this->dmaChannelTx = dmaChannelTx; this->dmaStreamTx = dmaStreamTx; 

			this->irqnRx = irqnRx; this->irqnTx = irqnTx; this->TX_IT_Flag = TX_IT_Flag;this->TX_Flag=TX_Flag;
			this->clockGPIO = clockGPIO; this->clockUSART = clockUSART; this->clockDMA = clockDMA;
			this->recvBuffer = recvBuffer; this->bufferSize = bufferSize;
    }
		
		bool offlineFlag;
		uint32_t offline_cnt;
		
		void Handle_UARTIdle_Interrupt();  //接收中断
		void Handle_DMAIC_Interrupt();     //发送中断
		
		void init();        //初始化
		void run_1000hz();  //运行函数,检查队列里有无数据，有则自动调用发送
		
		void SendMessage( uint8_t* data, uint16_t size); //队列发送函数                 
		void DMA_SendData(uint8_t* data, uint16_t size);//非队列发送函数  直接发送数据的地址和大小
		
		virtual	void ProcessReceivedData()=0;  //在中断处理时运行 recvBuf时接收dma接收的值
		uint32_t baudRate;
private:
    GPIO_TypeDef* txPort; uint16_t txPin; uint8_t txPinSource; uint8_t txAF;
    GPIO_TypeDef* rxPort; uint16_t rxPin; uint8_t rxPinSource;uint8_t rxAF;
    USART_TypeDef* usart;
    uint32_t dmaChannelRx; DMA_Stream_TypeDef* dmaStreamRx;
    uint32_t dmaChannelTx;DMA_Stream_TypeDef* dmaStreamTx;
    IRQn_Type irqnRx; IRQn_Type irqnTx; uint32_t TX_IT_Flag;uint32_t TX_Flag;
    uint32_t clockGPIO;
    uint32_t clockUSART;
    uint32_t clockDMA;
    uint8_t* recvBuffer;uint32_t  bufferSize;
		DMA_InitTypeDef DMA_InitStructure_rx,DMA_InitStructure_tx;

		volatile bool uartIdle = true ; //串口空闲的标志

		struct Message 
		{ 
			uint8_t* data;
			uint16_t size; 
		};
		myqueue<Message> messageQueue ; //串口消息队列	
};

class VISION:public USART_DMA
{
public:
	 VISION(GPIO_TypeDef* txPort, uint16_t txPin, uint8_t txPinSource, uint8_t txAF,
                    GPIO_TypeDef* rxPort, uint16_t rxPin, uint8_t rxPinSource, uint8_t rxAF,
                    USART_TypeDef* usart, uint32_t baudRate,
                    uint32_t dmaChannelRx, DMA_Stream_TypeDef* dmaStreamRx,
                    uint32_t dmaChannelTx, DMA_Stream_TypeDef* dmaStreamTx,
                    IRQn_Type irqnRx, IRQn_Type irqnTx, uint32_t TX_Flag, int32_t TX_IT_Flag,
                    uint32_t clockGPIO, uint32_t clockUSART, uint32_t clockDMA,
                    uint8_t* recvBuffer, uint32_t bufferSize)
        : USART_DMA(txPort, txPin, txPinSource, txAF,
                    rxPort, rxPin, rxPinSource, rxAF,
                    usart, baudRate,
                    dmaChannelRx, dmaStreamRx, dmaChannelTx, dmaStreamTx,
                    irqnRx, irqnTx, TX_Flag, TX_IT_Flag,
                    clockGPIO, clockUSART, clockDMA, recvBuffer, bufferSize){}

	void ProcessReceivedData() override;
																				
	void USB_init();
	uint32_t USB_offline_cnt;										
	bool USB_offline_Flag;
											
	void vision_Send_Data();										
};



extern VISION Vision;

extern vision_Recv_Struct_t vision_RecvStruct ;
extern vision_Send_Struct_t vision_SendStruct ;

#endif
