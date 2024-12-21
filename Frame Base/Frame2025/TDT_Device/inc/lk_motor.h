#ifndef _LK_MOTOR_
#define _LK_MOTOR_
#include "board.h"
#include "motor.h"
#include "can_calculate.h"
#include "can.h"
#define Torque_constant 0.32   //Ť�س���

class LK_Motor_Rec :public Virtual_Motor_Calculate
{
	private:
	int64_t originAngle : 56; //ֻ�����鿴��Ȧ�Ƕ�ԭʼֵ��(һ��������)
	public:
	void GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg) override;
};

class LK_Motor:public Virtual_Motor
{
public:
	bool use_my_send;
	LK_Motor_Rec  lk_Motor_Rec;
	LK_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t control_id,uint32_t _Std_ID);
	
	static void SendCurrentAgreement(uint8_t canbuff[8],float canResult) ;

	void Motor_Send();

	void Motor_Receive(u8 can_x,CanRxMsg *_CanRxMsg, u8 ifsetzero);//���պ���

//����Ϊ����������ã�����Ϊ�������ĵ�������
	
		void motorRead_totalAngle(void);  //�������Ȧ������
		void clearErrorFlag(void);//����쳣��־
		void motorClose(void);//����ر�����
		void motorStop(void);//���ֹͣ����
		void motorRun(void);//�����������               //��Щ�������������ִ��
	
	
		//ֻ���ڶ�����use_my_send=1ʱ�ſ���ʹ��һ�º�����������Ч
		void Auto_ctrlSpeed(float ctrlspeed);//����ٶȿ���
		//ע�⣬���µ�����ٶȲ���Ϊ0��
		void Auto_ctrlPosition(float ctrlposition,float maxspeed=1000);  //��Ȧλ�ÿ��� ,��Ҫ����motorRead_totalAngle ��������õĽǶ�ʹ��
			
	
	private:
	uint32_t control_id;
	u8 canTxBuff[8];
	u8 readcnt;
	
};

void LKList_Send();
void LKList_Receive(u8 can_x,CanRxMsg *_CanRxMsg);


#endif