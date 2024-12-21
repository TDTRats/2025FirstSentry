#ifndef __DrEmpower_can__
#define __DrEmpower_can__

#include "board.h"
#include "motor.h"
#include "can_calculate.h"
#include "can.h"

typedef struct  
{
    float value_data[4];//�����������͵�����ת��Ϊbyte����ֵ
    unsigned char byte_data[8];//����byteת��Ϊ�������ݵ����ͣ���ֵ
    int type_data[4];//��������ֵ����Ҫ��
    int length;//���ݸ�����ֵ����Ҫ��
}format_data_struct;  //����ȫ�ֱ��� data_list�����ڽ���CAN����encode��decode�����д洢�������뼰������

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
			sendThisCanMsg(control_id);//���ͺ���
		}
		
		else
		{
			readCommand();
			control_id= (CanId <<5) + 0x1E;
			sendThisCanMsg(control_id,canTxBuff);//���ͺ���
		}
	} 
	
	void Motor_Receive(CanRxMsg *_CanRxMsg)  //���պ���
	{
	 if (_CanRxMsg->StdId == (CanId <<5) + 0x1E)  //����������ݰ���ͷ�Ǵ�Ȼ�������֡��ͷ
		dr_Motor_Rec.motor_Rec_Calculate(&canInfo,_CanRxMsg,otherInfo.degree,1);
	}
	

	private:
	u8 canTxBuff[8];
	u32 control_id;
	u32 cnt=0;
	void readCommand();
};





#endif /* __DrEmpower_can__ */
