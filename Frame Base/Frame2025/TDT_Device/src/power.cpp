#include "power.h"
#include "judgement.h"
#include "my_math.h"
#include "info_task.h"
#if USE_POWER
Power power(onCAN1);

//�õ������޷�ϵ��"powerLimitKp"
void Power::getPowerKp(void)
{
	
	useTooMuchCapFlag 	 = powerDownLimit >= capacitance_percentage;
	
	//*******����δ���ߣ����ݽ�����ʹ�õ��ݻ򻺳�����>40ʱ->��������ģʽ********//
    if (offlineFlag==0 && ( capHealth || chassisPowerBuffer >= 40))
    {
			getPowerLimit();//�õ��������Ʒ�Χ
			powerLimitKp = (capacitance_percentage - powerDownLimit) / powerLimitRange;//ͨ����������������ϵ��
			powerLimitKp = LIMIT(powerLimitKp, 0.02, 1.0f); //ϵ���޷����ͱ�ϵ������֤���̲������ܲ���
    }
	//******************�����쳣������ϵͳ����->��Ӳ����ͬ����******************//
		else if(!judgement.jgmtOffline)
		{
			powerOverFlowCal();	//����ϵ������
			powerLimitKp = (chassisPowerBuffer - 10.0f) / 50.0f;//ͨ������������������ϵ��[����������10J~60J](60J/250J)
			powerLimitKp*= overFlowKp;//ʹ�ó���ϵ������������ǿ��
			powerLimitKp = LIMIT(powerLimitKp, 0.5, 0.8f);
		}
	//*****************�����쳣������ϵͳ����->���ñ�������ϵ��*****************//
	else 						 
		powerLimitKp = 0.1; 
	
	underVoltageProtection();//Ƿѹ����
	
}

//����ϵ������
//�����쳣������ϵͳ��������Ӳ������޹���ʱ��
//���overFlowKp
void Power::powerOverFlowCal(void)
{
	static u8 ctrlCnt;

	ctrlCnt = ctrlCnt%100;//��Ƶ����100:1

	if(!ctrlCnt++ && judgement.powerHeatData.chassisPowerBuffer  <= 20)//��������ʹ�õ��������£���������ǿ��
	{
		overFlowKp -= 0.03f;
		overFlowKp = LIMIT(overFlowKp,0.8f,1.0f);
	}
}

//Ƿѹ����ϵ������
//��ѹ���͵��µ������
//����Ƿѹ����ϵ��underVoltageProtectKp������powerLimitKp
void Power::underVoltageProtection(void)
{
	if(offlineFlag||!currentNow)//�������߻�ǰ����Ϊ0ʱ������
		return;
	underVoltageProtectKp = LIMIT(maxCurrent/currentNow,0,1.0f);
	powerLimitKp *= underVoltageProtectKp;
}


//�������Ʒ�Χ�趨
//����д����ʱ�ɷ�Χ�����޾���
//��ΧĬ��Ϊ10,��0ʱ������Χ��ֵ
void Power::setPowerLimit(float downLimit,float upLimit,uint8_t range)
{
	powerDownLimit = downLimit;
	if(upLimit)
		powerLimitRange = upLimit-downLimit;
	else if(range)
		powerLimitRange = range;
}

//�õ��������Ʒ�Χ
//����ģʽӵ��������ȼ���������̶�����30%~50%
//���ù��ʽ������ڵ��ݵ���55%ʱ��ʹ�ü���ʱ������������̶�����30%~50%��ǿ��ʹ�û��ͷ�3%�ĵ���
//ʹ�ö������ٺ�һ�����ͷų����������������̶�����50%~60%
//ʹ�ü��ٺ��ͷ�10%�ĵ��ݣ����������50%
//�ڹ��ʼ����б�����

void Power::getPowerLimit(void)
{
static u8 lastSpeedUpLevel;

	if(speedUpLevel == 3)//��������[����]
	{
		setPowerLimit(40,50);
		goto GOT_THE_LIMIT;
	}
	else if(speedUpLevel)//һ����������
	{
		if(!lastSpeedUpLevel)
		{
			setPowerLimit(MAX(40,capacitance_percentage-20));
			goto GOT_THE_LIMIT;
		}
	}
	else//�����ڼ�����
	{
		if(lastSpeedUpLevel == 1)//�Ӽ������˳�ʱ���ͷ�3%������[50%-90%]
			setPowerLimit(LIMIT(capacitance_percentage-5,50,90),0,0);
		
		setPowerLimit(30,50);
		/*�������ƻָ�*/
		//����>93%ʱ������[90%-100%]
		if(capacitance_percentage >= 93)
		{
			setPowerLimit(90,100);
			goto GOT_THE_LIMIT;
		}
		else
		{
			//�����Զ��ָ�����ǰ����ֵ����������10%
			setPowerLimit(LIMIT(capacitance_percentage - 20,powerDownLimit, 90));
			goto GOT_THE_LIMIT;
		}
	}
GOT_THE_LIMIT:	
	lastSpeedUpLevel = speedUpLevel;
}


//����CAN��Ϣ����(1kHz)
//������������
void Power::myPowerCanTx(void)
{
	if(	(powerOfflineCnt++) > 250)   	//�����ж���ֵ:250ms
	{
		powerOfflineCnt=0;
		offlineFlag=1;
	}
	
	static u8 txCnt;
	txCnt = !txCnt;
	
	//������Ϣ500Hzһ��,������ϵͳ����,��֡���͹���״̬
	if(!judgement.jgmtOffline && txCnt)  //����ϵͳ���߲�����,������,�ɹ���ģ�鴦��
	{ 
		chassisPower  = judgement.powerHeatData.chassisPower ;
		chassisPowerBuffer = LIMIT(judgement.powerHeatData.chassisPowerBuffer, 0, 60);   //���̻�������,�޷�����ֹ�䲻��ȥ��?
		max_chassis_power = LIMIT(judgement.gameRobotStatus.chassisPowerLimit  , 50, 100);  //����ʣ��޷�����ֹ����ϵͳ�ҷ�������ͨ������
		canTx(txData_110,(whereIsPower == onCAN1)?CAN1:CAN2,0X110);//����ϵͳ���
	}
	else
		canTx(txData_120,(whereIsPower == onCAN1)?CAN1:CAN2,0X120);//����״̬
}

//����CAN��Ϣ����
//����CAN�����ж�
void Power::myPowerCanRx(CanRxMsg canRxMsg)
{
	if(canRxMsg.StdId!=0x130)
		return;
	memcpy(rxData_130,canRxMsg.Data,8);
	
	//����ת��
	capacitance_percentage = capacitance_percentage_t/300.0f;
	maxCurrent = maxCurrent_t/2000.0f;
	currentNow = currentNow_t/2000.0f;
	
	powerOfflineCnt = 0;	//������߼�����
	offlineFlag=0;
}
#endif
