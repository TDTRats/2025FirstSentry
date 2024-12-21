#include "power.h"
#include "judgement.h"
#include "my_math.h"
#include "info_task.h"
#if USE_POWER
Power power(onCAN1);

//得到功率限幅系数"powerLimitKp"
void Power::getPowerKp(void)
{
	
	useTooMuchCapFlag 	 = powerDownLimit >= capacitance_percentage;
	
	//*******功率未离线，电容健康，使用电容或缓冲能量>40时->常规限制模式********//
    if (offlineFlag==0 && ( capHealth || chassisPowerBuffer >= 40))
    {
			getPowerLimit();//得到功率限制范围
			powerLimitKp = (capacitance_percentage - powerDownLimit) / powerLimitRange;//通过电容量给出限制系数
			powerLimitKp = LIMIT(powerLimitKp, 0.02, 1.0f); //系数限幅，低保系数，保证底盘不至于跑不了
    }
	//******************功率异常，裁判系统正常->软硬件共同限制******************//
		else if(!judgement.jgmtOffline)
		{
			powerOverFlowCal();	//超限系数计算
			powerLimitKp = (chassisPowerBuffer - 10.0f) / 50.0f;//通过缓冲能量给出限制系数[期望限制在10J~60J](60J/250J)
			powerLimitKp*= overFlowKp;//使用超限系数，增大限制强度
			powerLimitKp = LIMIT(powerLimitKp, 0.5, 0.8f);
		}
	//*****************功率异常，裁判系统离线->采用本地限制系数*****************//
	else 						 
		powerLimitKp = 0.1; 
	
	underVoltageProtection();//欠压保护
	
}

//超限系数计算
//功率异常，裁判系统正常，软硬件混合限功率时用
//输出overFlowKp
void Power::powerOverFlowCal(void)
{
	static u8 ctrlCnt;

	ctrlCnt = ctrlCnt%100;//差频控制100:1

	if(!ctrlCnt++ && judgement.powerHeatData.chassisPowerBuffer  <= 20)//缓冲能量使用到限制以下，增大限制强度
	{
		overFlowKp -= 0.03f;
		overFlowKp = LIMIT(overFlowKp,0.8f,1.0f);
	}
}

//欠压保护系数计算
//电压过低导致电调重启
//计算欠压保护系数underVoltageProtectKp并修正powerLimitKp
void Power::underVoltageProtection(void)
{
	if(offlineFlag||!currentNow)//功率离线或当前电流为0时不修正
		return;
	underVoltageProtectKp = LIMIT(maxCurrent/currentNow,0,1.0f);
	powerLimitKp *= underVoltageProtectKp;
}


//电容限制范围设定
//不填写上限时由范围和下限决定
//范围默认为10,填0时不给范围赋值
void Power::setPowerLimit(float downLimit,float upLimit,uint8_t range)
{
	powerDownLimit = downLimit;
	if(upLimit)
		powerLimitRange = upLimit-downLimit;
	else if(range)
		powerLimitRange = range;
}

//得到功率限制范围
//飞坡模式拥有最高优先级，开启后固定降至30%~50%
//备用功率仅允许在电容低于55%时或使用加速时开启，开启后固定降至30%~50%，强制使用会释放3%的电容
//使用二级加速后及一次性释放常规电容量，开启后固定降至50%~60%
//使用加速后释放10%的电容，但不会低于50%
//在功率计算中被调用

void Power::getPowerLimit(void)
{
static u8 lastSpeedUpLevel;

	if(speedUpLevel == 3)//三级加速[飞坡]
	{
		setPowerLimit(40,50);
		goto GOT_THE_LIMIT;
	}
	else if(speedUpLevel)//一、二级加速
	{
		if(!lastSpeedUpLevel)
		{
			setPowerLimit(MAX(40,capacitance_percentage-20));
			goto GOT_THE_LIMIT;
		}
	}
	else//不处于加速中
	{
		if(lastSpeedUpLevel == 1)//从加速中退出时，释放3%的能量[50%-90%]
			setPowerLimit(LIMIT(capacitance_percentage-5,50,90),0,0);
		
		setPowerLimit(30,50);
		/*电容限制恢复*/
		//电容>93%时控制在[90%-100%]
		if(capacitance_percentage >= 93)
		{
			setPowerLimit(90,100);
			goto GOT_THE_LIMIT;
		}
		else
		{
			//下限自动恢复至当前电容值，上限上推10%
			setPowerLimit(LIMIT(capacitance_percentage - 20,powerDownLimit, 90));
			goto GOT_THE_LIMIT;
		}
	}
GOT_THE_LIMIT:	
	lastSpeedUpLevel = speedUpLevel;
}


//功率CAN信息发送(1kHz)
//两个包轮流发
void Power::myPowerCanTx(void)
{
	if(	(powerOfflineCnt++) > 250)   	//离线判定阈值:250ms
	{
		powerOfflineCnt=0;
		offlineFlag=1;
	}
	
	static u8 txCnt;
	txCnt = !txCnt;
	
	//单条信息500Hz一发,若裁判系统离线,满帧发送功率状态
	if(!judgement.jgmtOffline && txCnt)  //裁判系统离线不处理,不发送,由功率模块处理
	{ 
		chassisPower  = judgement.powerHeatData.chassisPower ;
		chassisPowerBuffer = LIMIT(judgement.powerHeatData.chassisPowerBuffer, 0, 60);   //底盘缓冲能量,限幅，防止充不进去电?
		max_chassis_power = LIMIT(judgement.gameRobotStatus.chassisPowerLimit  , 50, 100);  //最大功率，限幅，防止裁判系统乱发数导致通道超载
		canTx(txData_110,(whereIsPower == onCAN1)?CAN1:CAN2,0X110);//裁判系统相关
	}
	else
		canTx(txData_120,(whereIsPower == onCAN1)?CAN1:CAN2,0X120);//功率状态
}

//功率CAN信息接收
//放在CAN接收中断
void Power::myPowerCanRx(CanRxMsg canRxMsg)
{
	if(canRxMsg.StdId!=0x130)
		return;
	memcpy(rxData_130,canRxMsg.Data,8);
	
	//数据转换
	capacitance_percentage = capacitance_percentage_t/300.0f;
	maxCurrent = maxCurrent_t/2000.0f;
	currentNow = currentNow_t/2000.0f;
	
	powerOfflineCnt = 0;	//清空离线计数器
	offlineFlag=0;
}
#endif
