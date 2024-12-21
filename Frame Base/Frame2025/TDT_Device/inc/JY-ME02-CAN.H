#pragma once

#include "can.h"
#include "JY-ME02-CAN.h"

class JYencoder
{
	public :
		//处理接收信息
		void FbmsgProcess(CanRxMsg *RxCanMsg) ;
		//编码器内参设定
		void ParamSet(float sample_t , float m_zero) ;
		//角度反馈
		float FbAngle ;
		//角速度反馈
		float Fbdps ;
		//圈数反馈
		float Fbroll ;
		//温度反馈
		float Fbtemp ;
		//换算之后的角度(-180~180)
		float trans_angle ;
		//换算之后的过圈角度
		float TotalAngle ;
	
	private :
		//角速度采样时间(s)，默认0.1
		float SampleTime_Dps = 0.1 ;
		//机械零点
		float mechzero ;
		//上一帧的单圈角度
		float lastsingleangle ;
		//认为的过圈
		float my_roll ;
};