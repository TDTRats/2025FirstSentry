/*
Author: skywalker&codeeartist
TIME:2024_3_30

*exp:
*   Motor1 = new LK_Motor(MF9025, CAN2, 0x203);  //创建motor
*****************************************************
		Motor_Send(); 发送函数
		Motor_Receive(CanRxMsg *_CanRxMsg, u8 ifsetzero=0) 接收函数
		其余都和motor一样
*****************************************************使用电机内置反馈值
		Motor1->Auto_ctrlSpeed(50);
		Motor1->Auto_ctrlPosition(30);
		
		以及一堆命令
*/

#include "lk_motor.h"
#include "dbus.h"

#define MAX_NUM 10  //定义最大不会有10个lkmotor同时被声明
uint8_t valid_motorNum=0;   //一共定义的lk电机对象
LK_Motor* lkList[MAX_NUM];

enum _CMD
{
	ERROR_CLEAR_FB = 0X9A,
  ERROR_CLEAR = 0X9B,
  MOTOR_CLOSE = 0X80,
  MOTOR_STOP = 0X81,
  MOTOR_RUN = 0X88,
  CTRL_IQ = 0XA1,    //控制力矩模式
  CTRL_SPEED = 0XA2, //速度模式
	MUL_POSITION=0XA4, //多圈位置模式
	MUL_POSI_READ=0X92 //读多圈位置
};

LK_Motor::LK_Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t control_id,uint32_t _Std_ID):Virtual_Motor(motorType, _Canx, _Std_ID)
{
	this->control_id=control_id;
	Send_Current_Agreement=&LK_Motor::SendCurrentAgreement;
	
	lkList[valid_motorNum]=this;
	valid_motorNum++;
}

void LK_Motor::Motor_Send()
{
		if(readcnt++==5)	
		{
		 motorRead_totalAngle();
		 readcnt=0;
		 return;
		} //5帧读一次多圈角度（一般电机可以不用）
		
		if(use_my_send)
			sendThisCanMsg(control_id,canTxBuff);//自定义发送函数
		else
			sendThisCanMsg(control_id);//发送函数
}

void LK_Motor:: Motor_Receive(u8 can_x,CanRxMsg *_CanRxMsg, u8 ifsetzero=0) 
{
		if(_CanRxMsg->StdId != getStd_Id() || can_x!=getCan_x())
    return;
		lk_Motor_Rec.motor_Rec_Calculate(&canInfo,_CanRxMsg,otherInfo.degree,ifsetzero);
}



/**
 * @brief 电机速度控制
 * @param 控制的速度  dps
*/
void LK_Motor::Auto_ctrlSpeed(float ctrlspeed)
{
	if(enableFlag.oneDeforce || deforceFlag || canInfo.lostFlag)
	{
		ctrlspeed=0;
		Send_Current_Agreement(canTxBuff,0);
		return;
	}
	
	int32_t speed=ctrlspeed*100;
    /* 清除发送区 */
    memset(canTxBuff,0,8);
    /* 命令给定 */
    canTxBuff[0] = CTRL_SPEED;
    /* 数据填充 */
	canTxBuff[4] = (uint8_t)((uint32_t)speed&0XFF);
	canTxBuff[5] = (uint8_t)((uint32_t)speed>>8&0XFF);
	canTxBuff[6] = (uint8_t)((uint32_t)speed>>16&0XFF);
	canTxBuff[7] = (uint8_t)((uint32_t)speed>>24&0XFF);
	
}

/**
 * @brief 电机多圈角度控制
 * @param 控制的角度 为 1degree/LSB
maxspeed为最大速度 1 dps/LSB 可以决定速度和力
*/
void LK_Motor::Auto_ctrlPosition(float ctrlposition,float maxspeed)  //使用这个函数时需要注意电机是否有减速比
{
	if(enableFlag.oneDeforce || deforceFlag || canInfo.lostFlag)
	{
		Send_Current_Agreement(canTxBuff,0);
		return;
	}
	if(maxspeed<0)
		maxspeed=10;
	int32_t position=ctrlposition*100;

  canTxBuff[0] = MUL_POSITION;
  /* 数据填充 */
	canTxBuff[2] = (uint8_t)((uint32_t)maxspeed&0XFF);
	canTxBuff[3] = (uint8_t)((uint32_t)maxspeed>>8&0XFF);
	
	canTxBuff[4] = (uint8_t)((uint32_t)position&0XFF);
	canTxBuff[5] = (uint8_t)((uint32_t)position>>8&0XFF);
	canTxBuff[6] = (uint8_t)((uint32_t)position>>16&0XFF);
	canTxBuff[7] = (uint8_t)((uint32_t)position>>24&0XFF);
}


/**
 * @brief 清除异常标志
*/
void LK_Motor::clearErrorFlag(void)
{
	u8 canTxBuf[8];
	memset(canTxBuf,0,8);
  canTxBuf[0] = ERROR_CLEAR;
	sendThisCanMsg(control_id,canTxBuf);
}

/**
 * @brief 电机关闭命令
*/
void LK_Motor::motorClose(void)
{
	u8 canTxBuf[8];
	memset(canTxBuf,0,8);
  canTxBuf[0] = MOTOR_CLOSE;
	sendThisCanMsg(control_id,canTxBuf);
}

/**
 * @brief 电机停止命令
*/
void LK_Motor::motorStop(void)
{
  u8 canTxBuf[8];
	memset(canTxBuf,0,8);
  canTxBuf[0] = MOTOR_STOP;
	sendThisCanMsg(control_id,canTxBuf);
}

/**
 * @brief 电机运行命令
*/
void LK_Motor::motorRun(void)
{
  u8 canTxBuf[8];
	canTxBuf[0] = MOTOR_RUN;
	sendThisCanMsg(control_id,canTxBuf);
	
}

/**
 * @brief 读取多圈角度
*/

void LK_Motor::motorRead_totalAngle(void)
{
    u8 canTxBuf[8];
	  memset(canTxBuf,0,8);
    /* 命令给定 */
    canTxBuf[0] =MUL_POSI_READ;
		sendThisCanMsg(control_id,canTxBuf);
}


void LK_Motor_Rec::GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg)
{
	 /* 根据命令ID进行相应处理 */
    switch(_CanRxMsg->Data[0])
    {
        case CTRL_IQ:      
        case CTRL_SPEED:
				case MUL_POSITION:
            Info->temperature = _CanRxMsg->Data[1];
            Info->trueCurrent  = (int16_t)((uint16_t)(_CanRxMsg->Data[3]<<8)|_CanRxMsg->Data[2]);    //转矩电流
            Info->speed = (uint16_t)((uint16_t)(_CanRxMsg->Data[5]<<8)|_CanRxMsg->Data[4]);
            Info->encoder = (uint16_t)((uint16_t)(_CanRxMsg->Data[7]<<8)|_CanRxMsg->Data[6]); //编码器位置
            break;
			case MUL_POSI_READ:
				originAngle=0;
				originAngle |= (int64_t)_CanRxMsg->Data[1] << 0;   // 将第1字节移动到正确的位置
				originAngle |= (int64_t)_CanRxMsg->Data[2] << 8;   // 将第2字节移动到正确的位置
				originAngle |= (int64_t)_CanRxMsg->Data[3] << 16;  // 将第3字节移动到正确的位置
				originAngle |= (int64_t)_CanRxMsg->Data[4] << 24;  // 将第4字节移动到正确的位置
				originAngle |= (int64_t)_CanRxMsg->Data[5] << 32;  // 将第5字节移动到正确的位置
				originAngle |= (int64_t)_CanRxMsg->Data[6] << 40;  // 将第6字节移动到正确的位置
				originAngle |= (int64_t)_CanRxMsg->Data[7] << 48;  // 将第7字节移动到正确的位置
				Info->Auto_totalAngle  = ((int64_t)originAngle)/100.0f;
				break;
        default:
            break;
    }	
}

void LK_Motor::SendCurrentAgreement(uint8_t canbuff[8],float canResult)
{
	memset(canbuff,0,8);
	canbuff[0] = CTRL_IQ;
	int16_t current =(int16_t)(canResult);
	canbuff[4] = current & 0xFF;
	canbuff[5] = (current >> 8) & 0xFF;
}


void LKList_Send()
{
	if(valid_motorNum<=0)
		return;
	for(int i=0;i<valid_motorNum;i++)
	{
		lkList[i]->Motor_Send();
	}
}

void LKList_Receive(u8 can_x,CanRxMsg *_CanRxMsg)
{
	if(valid_motorNum<=0)
		return;
	
	for(int i=0;i<valid_motorNum;i++)
	{
		lkList[i]->Motor_Receive(can_x,_CanRxMsg);
	}
}


