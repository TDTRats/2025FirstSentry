/*
	ά������JY-ME02-CAN������
	��Ʒ˵���飺https://wit-motion.yuque.com/wumwnr/docs/qcgu36
	����λ�����ñ����������̣�
		1.�޸�can�����е�Can.CAN_PrescalerΪ12�������շ�������Ϊ250kbps -> 1024//42/(1+9+4)/12=0.25Mbps
		2.����������ҳ��˵�����б༭��Ӧ���ݰ����б����������ʣ��ش�Ƶ�ʵȵĸ���
		3.�޸����֮��ͨ��can����������
*/
#include "JY-ME02-CAN.h"

/*
	��������������ֵ([��Ȧ�Ƕ�-���ٶ�-Ȧ��-�¶�]��ʽ)
*/
void JYencoder::FbmsgProcess(CanRxMsg *RxCanMsg)
{
	//can��Ϣ����
	if(RxCanMsg->Data[0] == 0x55 && RxCanMsg->Data[1] == 0x55)
	{
		this->FbAngle = (float)((int16_t)((RxCanMsg->Data[3] << 8) | RxCanMsg->Data[2] ) ) * 360.0f / 32768.0f ;
		this->Fbdps = (float)((int16_t)((RxCanMsg->Data[5] << 8) | RxCanMsg->Data[4])) * 360.0f / 32768.0f / SampleTime_Dps ;
		this->Fbroll = (float)((int16_t)((RxCanMsg->Data[7] << 8) | RxCanMsg->Data[6]));
	}
	if(RxCanMsg->Data[0] == 0x55 && RxCanMsg->Data[1] == 0x56)
	{
		this->Fbtemp = (float)(int16_t)((RxCanMsg->Data[3] << 8) | RxCanMsg->Data[2]) / 100.0f ;
	}
	//�Ƕ�ת������
	this->FbAngle - this->mechzero > 0 ? this->trans_angle = this->FbAngle - this->mechzero : this->trans_angle = 360 + this->FbAngle - this->mechzero ;
	this->trans_angle - 180 >= 0 ? this->trans_angle -= 360 : this->trans_angle -= 0 ;
	if(this->trans_angle - this->lastsingleangle > 300) 
	//��Ϊ�����˷����Ȧ(-180->180)
	{
		my_roll -= 1 ;
	}
	if(this->trans_angle - this->lastsingleangle < -300) 
	//��Ϊ�����������Ȧ(180->-180)
	{
		my_roll += 1 ;
	}
	this->TotalAngle = my_roll * 360 + this->trans_angle ;
	this->lastsingleangle = this->trans_angle ;
}
/*
	���ñ������ڲ�
*/
void JYencoder::ParamSet(float sample_t , float m_zero)
{
	this->SampleTime_Dps = sample_t ;
	this->mechzero = m_zero ;
}