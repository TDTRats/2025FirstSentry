#ifndef _POWER_
#define _POWER_

#include "board.h"
#include "can.h"

enum WhereIsPower
{
	onCAN1=0,
	onCAN2=1
};

class Power
{
public:
	Power(WhereIsPower can)
	{
		this->whereIsPower=can;
		useCap=1;
		Check_Mode=0;
		ULTS_Mode=0;
	}
	
	
	float powerLimitKp;					//��������ϵ��

	float underVoltageProtectKp = 1;	//Ƿѹ����ϵ��
	float overFlowKp = 1;				//��������ϵ��

	union{	//0X110:����ϵͳ�����������
		u8 txData_110[8];
		struct {
			float chassisPower;				//���̹���
			uint16_t chassisPowerBuffer;	//���̻�������
			uint16_t max_chassis_power;		//�����		
		};
	};
	
	union{	//0X120:����״̬����  ���°湦�ʲ����л�����ֵ��
		u8 txData_120[8];
		struct {
			uint8_t useCap:1;			//ʹ�õ���[·���л�]
			uint8_t Check_Mode:1;		//��¼ģʽ
			uint8_t ULTS_Mode:1;		//��������ģʽ
		};	
	};
	
	union{	//0X130:����ģ�鷴������
		u8 rxData_130[8];
		struct {
			uint16_t capacitance_percentage_t;	//���������ٷֱ�[��ͨѶ�ã�ģ�ⶨ����]
			uint16_t maxCurrent_t;				//������[��ͨѶ�ã�ģ�ⶨ����]
			uint16_t currentNow_t;				//��ǰ����[��ͨѶ�ã�ģ�ⶨ����]
			uint16_t capHealth:1;				//��ǰ���ݵ�ѹ���ͣ�����ֱͨ״̬
			uint16_t errorCode:3;				//���ʴ���
			uint16_t reserved:12;
		};	
	};
	float capacitance_percentage;	//���ݰٷֱ�
	float currentNow;				//��ǰ����
	float maxCurrent;				//������

	uint8_t powerDownLimit;			//���ʿ�������
	uint8_t powerLimitRange;		//���ʿ��Ʒ�Χ
	
	uint8_t speedUpLevel;	//���ٵȼ� 0:��̬|1:��ͨ����|2:��������|3:���¼���
	
	bool useTooMuchCapFlag;			//����ʹ�ó��ޱ�־λ

	WhereIsPower whereIsPower;//����ģ�����������
	bool offlineFlag=1;
	
	void myPowerCanRx(CanRxMsg CanRxMsg);
	void myPowerCanTx(void);			//����CAN��Ϣ����
	
	void getPowerKp(void);	//���ݼ��ٵȼ��õ���������ϵ��
	void setPowerLimit(float downLimit,float upLimit = 0,uint8_t range = 10);
private:
	u16 powerOfflineCnt;
	void getPowerLimit(void);			//���ݼ��ٵȼ�ѡ��������ֵ
	void powerOverFlowCal(void);		//�������ʼ���
	void underVoltageProtection(void);	//Ƿѹ����ϵ������
	
};

extern Power power;


//ͨ��˫�����
typedef struct _doubleKickJudge{
public:
	//_doubleKickJudge(float doublekickJudgeThreshold = 0.5f){}
	
	//�޼�ֵ����0����������1��˫������2
	uint8_t doubleKickVal(uint8_t keyVal)
	{
		doublekickJudgeThreshold = 0.3f; //˫�����ʱ����
		if(!keyVal)//�޼�ֵʱ����0
			thisKeyVal = 0;
		if(keyVal>lastKeyVal)//��ֵ�½��ؼ��
		{
			lastKickTimeStamp = thisKickTimeStamp; //��¼�ϴ�ʱ���
			thisKickTimeStamp = getSysTimeUs() / 1e6f; //����µ�ʱ���
			//������ʱ����������ֵ��,����2[˫��]�����򷵻�1[����]
			thisKeyVal = 1 + (timeIntervalFrom_f(lastKickTimeStamp) < doublekickJudgeThreshold);
		}
		lastKeyVal = keyVal;
		return thisKeyVal;
	};
private:
	float doublekickJudgeThreshold;//˫����ֵ(s),Ĭ��0.3��
	float thisKickTimeStamp;//����һ�ε��ʱ���(s)
	float lastKickTimeStamp;//��һ�ε��ʱ���(s)
	uint8_t lastKeyVal;
	uint8_t thisKeyVal;
}doubleKickJudge;


#endif
