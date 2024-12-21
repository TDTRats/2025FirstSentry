#ifndef __DrEmpower_can__
#define __DrEmpower_can__

#include "board.h"
#include "motor.h"
#include "can_calculate.h"
#include "can.h"

typedef struct  
{
    float value_data[4];//若将五种类型的数据转换为byte，则赋值
    unsigned char byte_data[8];//若将byte转换为五种数据的类型，则赋值
    int type_data[4];//类型名赋值（必要）
    int length;//数据个数赋值（必要）
}format_data_struct;  //定义全局变量 data_list，用于进行CAN数据encode和decode过程中存储参数输入及结果输出

void format_data(float *value_data, int *type_data,int length, char * str);
void byte2value();
void value2byte();



class DR_Motor_Rec :public Virtual_Motor_Calculate
{
	public:
	void GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg) override;
};



class DR_Motor:public Virtual_Motor
{
public:
	uint32_t CanId;
	DR_Motor_Rec  dr_Motor_Rec;
	
	DR_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t CanID,uint16_t degree):Virtual_Motor(motorType, _Canx, CanID)
	{
		this->CanId=CanID;
		otherInfo.degree=degree;
		Send_Current_Agreement=&DR_Motor::SendCurrentAgreement;
	}
	
  void SendCurrentAgreement(uint8_t canbuff[8],float canResult) ;

	void Motor_Send()
	{	
		cnt=(cnt++)%2;
		if(cnt)
		{
			control_id= (CanId <<5) + 0x1d;
			sendThisCanMsg(control_id);//发送函数
		}
		
		else
		{
			readCommand();
			control_id= (CanId <<5) + 0x1E;
			sendThisCanMsg(control_id,canTxBuff);//发送函数
		}
	} 
	
	void Motor_Receive(CanRxMsg *_CanRxMsg)  //接收函数
	{
	 if (_CanRxMsg->StdId == (CanId <<5) + 0x1E)  //如果反馈数据包包头是大然电机反馈帧包头
		dr_Motor_Rec.motor_Rec_Calculate(&canInfo,_CanRxMsg,otherInfo.degree,1);
	}
	

	private:
	u8 canTxBuff[8];
	u32 control_id;
	u32 cnt=0;
	void readCommand();
};





#endif /* __DrEmpower_can__ */
