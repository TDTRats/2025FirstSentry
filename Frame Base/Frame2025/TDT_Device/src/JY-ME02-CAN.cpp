/*
	维特智能JY-ME02-CAN编码器
	产品说明书：https://wit-motion.yuque.com/wumwnr/docs/qcgu36
	无上位机设置编码器的流程：
		1.修改can配置中的Can.CAN_Prescaler为12，配置收发波特率为250kbps -> 1024//42/(1+9+4)/12=0.25Mbps
		2.按照上述网页中说明自行编辑相应数据包进行编码器波特率，回传频率等的更改
		3.修改完毕之后通过can包保存配置
*/
#include "JY-ME02-CAN.h"

/*
	解析编码器反馈值([单圈角度-角速度-圈数-温度]形式)
*/
void JYencoder::FbmsgProcess(CanRxMsg *RxCanMsg)
{
	//can信息解析
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
	//角度转换处理
	this->FbAngle - this->mechzero > 0 ? this->trans_angle = this->FbAngle - this->mechzero : this->trans_angle = 360 + this->FbAngle - this->mechzero ;
	this->trans_angle - 180 >= 0 ? this->trans_angle -= 360 : this->trans_angle -= 0 ;
	if(this->trans_angle - this->lastsingleangle > 300) 
	//认为出现了反向过圈(-180->180)
	{
		my_roll -= 1 ;
	}
	if(this->trans_angle - this->lastsingleangle < -300) 
	//认为出现了正向过圈(180->-180)
	{
		my_roll += 1 ;
	}
	this->TotalAngle = my_roll * 360 + this->trans_angle ;
	this->lastsingleangle = this->trans_angle ;
}
/*
	设置编码器内参
*/
void JYencoder::ParamSet(float sample_t , float m_zero)
{
	this->SampleTime_Dps = sample_t ;
	this->mechzero = m_zero ;
}