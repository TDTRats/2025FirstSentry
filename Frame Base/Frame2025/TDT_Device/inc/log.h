#ifndef _LOG_H_
#define _LOG_H_

#include "board.h"
#include "time.h"
#include "task_virtual.h"
#include "motor.h"
#include "imu_task.h"
#include "bmi088.h"
#include "dbus.h"
// ������ṹ�壬���� id����ַ�ʹ�С
struct DataItem {
    uint8_t id;            // ���ݵ� ID
    const uint8_t* data;   // ���ݵĵ�ַ
    uint8_t size;          // ���ݵĴ�С
};


class Log_task
{
public:
  Log_task();

	void run();
	void Log_Auto_update();

	void init();
	void Log_Autolink();

	void link(uint8_t id, const uint8_t* data, uint8_t size);	
	
	void logData();
	bool full_error=0;           //��������
	bool repeatId=0;             //���ظ�id
private:
	

    myvector<DataItem,80> data_list;  //�����б� ,���80����λ

#pragma pack(1)		
		struct
		{
			uint8_t header = 0xA5; //��ͷ��Ĭ��0XA5
			u8 type=6;
			
			uint8_t Tx_Buf[600];   //600���ֽڵķ��ͻ�����
			
			uint64_t time_stamp; 
			int16_t frame_id; //�����
			uint8_t endheader = 0xB9; 
			uint16_t CRC16CheckSum; //У����
		}Tx_Struct;
		
		
struct  Motor_Log
{
	u8 type;	 ///<�������
	u8 can_x;	 ///<����CAN����[CAN_1 / CAN_2]
	uint32_t std_ID; ///<���CAN���߷���StdID
	uint16_t encoder;				///<ԭʼ��е������ֵ
	int16_t speed;					///<ԭʼ��еת��
	int32_t trueCurrent;			//ʵ�ʵ���
	int16_t temperature;			//����¶�
	float totalAngle_f;				///<�ܽǶȣ����㣩����mpu6050��λ���
	int32_t totalRound;				///<��Ȧ��
	u8 lostFlag;					//���߱�־λ
	CanInfo*CanInfo;
};

myvector<Motor_Log,20>  motor_Log_list;

struct	Imu_Log
{
	float acc[3];   ///<���ٶȽ�������
  float  gyro[3]; ///<���ٶȽ�������
	float Angle[3];
	float watchtemp;  //�������¶�
}imu_Log;

struct RC_Log
{
	int16_t CH[13];
	int32_t wheelCode;//�����ֱ���ֵ
	
	uint16_t W : 1;		///<0x0001
	uint16_t S : 1;		///<0x0002
	uint16_t A : 1;		///<0x0004
	uint16_t D : 1;		///<0x0008
	uint16_t SHIFT : 1; ///<0x0010
	uint16_t CTRL : 1;	///<0x0020
	uint16_t Q : 1;		///<0x0040
	uint16_t E : 1;		///<0x0080
	uint16_t R : 1;		///<0x0100
	uint16_t F : 1;		///<0x0200
	uint16_t G : 1;		///<0x0400
	uint16_t Z : 1;		///<0x0800
	uint16_t X : 1;		///<0x1000
	uint16_t C : 1;		///<0x2000
	uint16_t V : 1;		///<0x4000
	uint16_t B : 1;		///<0x8000
	
	u8 offlineFlag;
}rc_Log;

struct Power_Log
{
	uint16_t capacitance_percentage_t;	//���������ٷֱ�[��ͨѶ�ã�ģ�ⶨ����]
	uint16_t maxCurrent_t;				//������[��ͨѶ�ã�ģ�ⶨ����]
	uint16_t currentNow_t;				//��ǰ����[��ͨѶ�ã�ģ�ⶨ����]
	uint16_t capHealth;				//��ǰ���ݵ�ѹ���ͣ�����ֱͨ״̬
	uint16_t errorCode;				//���ʴ���
	u8 offlineFlag;
} power_Log;
	

#pragma pack()	
    size_t current_offset; // ��ǰƫ����
};


extern Log_task log_task;



#endif