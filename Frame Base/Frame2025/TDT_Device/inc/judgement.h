#ifndef __JUDGEMENT_H
#define __JUDGEMENT_H
#include "board.h"

static const u16 RED_BLUE_ID_DIFF = 100;	 //红蓝ID差
static const u16 ROBOT_CLIENT_ID_DIFF = 256; //机器人与客户端ID差
enum RobotIdDef
{
	ID_HERO = 1,
	ID_ENGINEER = 2,
	ID_INFANTRY1 = 3,
	ID_INFANTRY2 = 4,
	ID_INFANTRY3 = 5,
	ID_AIR = 6,
	ID_GUARD = 7,
	ID_RADAR = 9,
	REFEREE = 0x8080  //服务器端，自主决策用
};

enum JudgementStep
{
	STEP_HEADER = 0,		//<帧头数据获取
	STEP_HEADER_CRC8 = 1,	//<CRC8校验
	STEP_CMDID_GET = 2,		//<获取命令码
	STEP_DATA_TRANSFER = 3, //<数据转移（从JudgeDataBuffer转移至cmd_id对应的联合体中）
	STEP_DATA_CRC16 = 4,	//<CRC16校验（暂未加校验）
};

enum CRC_chose
{
	CRC_NO_CALC_STATUS =0, //不计算CRC16和CRC8
	CRC8_CRC16_CALC_STATUS=1, //同时计算CRC16和CRC8
  CRC16_CALC_STATUS=2,	 //仅计算CRC16
};

#define JUDGE_BUFFER_LENGTH 255


#pragma pack(1) //用于结构体的内存对齐， 方便使用联合体调用
class Judgement
{
	struct FrameHeader
	{
		unsigned char sof;		   ///<帧头
		unsigned short dataLength; ///<数据长度（0x301包含内容ID、发送者ID以及接收者ID）
		unsigned char seq;		   ///<包序号，可填0
		unsigned char crc8;		   ///<CRC8-校验
		unsigned short cmdid;	   ///<命令ID
	};

public:
	Judgement();
	/******************以下函数调用频率最高为10Hz***************/
	/// 0x301联合体数据发送，需将dataCmdId、senderId、receiverId填写完，一般不调用此函数
	void customSend(u8 count);
	/// 填写与机器人交互的数据后调用此函数可直接发送
	void robotsCommunication(uint16_t dataCmdId, RobotIdDef robotIdDef, u8 dataLenth);
	/// 发送sendGraphics个图形
	void graphicDraw(u8 sendGraphics);
	/// 发送字符传
	void characterDraw();
	/// 删除图层
	void graphicDel();

	
	/******************以上函数调用频率最高为10Hz***************/
	///发送地图命令
	void PathSend();
	void messageSend();
	void AutoSend(void);

	void uartSendBytes(uint8_t *data,uint16_t size);  //队列发送
	void DMA_Send(uint8_t *data,uint16_t size);       //非队列发送
	//初始化
	void init();
	//正常运行（离线，队列，接收队列)
	void run_1000hz();

	//队列解析
	void ringQueue();

	uint16_t IdToMate(RobotIdDef robotIdDef);
	uint16_t IdToMate(uint16_t robotId);
	uint16_t ClientId();
	uint16_t ClientId(RobotIdDef robotIdDef);
	uint16_t IdToEnemy(RobotIdDef robotIdDef);
	uint16_t IdToEnemy(uint16_t robotId);

	struct GameStatus
	{
		uint8_t gameType : 4;
		uint8_t gameProgress : 4;
		uint16_t stageRemainTime;
		uint64_t syncTimeStamp;
	} gameStatus;

	struct GameResult
	{
		uint8_t winner;
	} gameResult;

	union GameRobotHP
	{
		u16 teamHp[2][8];
		u16 allHp[16];
		struct
		{
			uint16_t red_1RobotHp;
			uint16_t red_2RobotHp;
			uint16_t red_3RobotHp;
			uint16_t red_4RobotHp;
			uint16_t red_5RobotHp;
			uint16_t red_7RobotHp;
			uint16_t redOutpostHp;
			uint16_t redBaseHp;
			uint16_t blue_1RobotHp;
			uint16_t blue_2RobotHp;
			uint16_t blue_3RobotHp;
			uint16_t blue_4RobotHp;
			uint16_t blue_5RobotHp;
			uint16_t blue_7RobotHp;
			uint16_t blueOutpostHp;
			uint16_t blueBaseHp;
		} singleHp;
	} gameRobotHP;

	

	struct EventData
	{
		uint8_t enrich_forward:1;
		uint8_t enrich_inside :1;
		uint8_t enrich:1;
		uint8_t energy_active:1;
		uint8_t small_energy:1;
		uint8_t big_energy :1;
		uint8_t annular_highland:2;
		uint8_t ladder3_highland:2;
		uint8_t ladder4_highland:2;
		uint8_t virtual_shield :7;
		uint16_t dart_time :9;
		uint8_t dart_target:2;
		uint8_t center_buff :2;	
	} eventData;

	struct SupplyProjectileAction
	{
		uint8_t supplyProjectileId;
		uint8_t supplyRobotId;
		uint8_t supplyProjectileStep;
		uint8_t supplyProjectileNum;
	} supplyProjectileAction;

	struct RefereeWarning
	{
		uint8_t level;
		uint8_t foulRobotId;
		uint8_t count;
	} refereeWarning;

	struct DartRemainingTime
	{
		uint8_t time;
		uint8_t have_hitten_target:2;
		uint8_t sum_hit:3;
		uint8_t now_target:2;
		uint16_t none:9;
	} dartRemainingTime;

	struct GameRobotStatus
	{
		uint8_t robotId;
		uint8_t robotLevel;
		uint16_t remainHp;
		uint16_t maxHp;
		uint16_t shooter_barrel_cooling_value;
		uint16_t shooter_barrel_heat_limit;
		uint16_t chassisPowerLimit;
		uint8_t mainsPowerGimbalOutput : 1;
		uint8_t mainsPowerChassisOutput : 1;
		uint8_t mainsPowerShooterOutput : 1;
	} gameRobotStatus;

	struct PowerHeatData
	{
		uint16_t chassisVolt;
		uint16_t chassisCurrent;
		float chassisPower;
		uint16_t chassisPowerBuffer;
		uint16_t shooterId1_17mmCoolingHeat;
		uint16_t shooterId2_17mmCoolingHeat;
		uint16_t shooterId1_42mmCoolingHeat;
	} powerHeatData;

	struct GameRobotPos
	{
		float x;
		float y;
		float yaw;
	} gameRobotPos;

	struct Buff
	{
		uint8_t recovery_buff;
		uint8_t cooling_buff;
		uint8_t defence_buff;
		uint8_t vulnerability_buff;
		uint16_t attack_buff;
	} buff;

	struct AerialRobotEnergy
	{
		uint8_t airforce_status;
		uint8_t time_remain;
	} aerialRobotEnergy;

	struct RobotHurt
	{
		uint8_t armorId : 4;
		uint8_t hurtType : 4;
	} robotHurt;

	struct ShootData
	{
		uint8_t bulletType;
		uint8_t shooterId;
		uint8_t bulletFreq;
		float bulletSpeed;
	} shootData;

	struct BulletRemaining
	{
		uint16_t bulletRemainingNum_17mm;
		uint16_t bulletRemainingNum_42mm;
		uint16_t coinRemainingNum;
	} bulletRemaining;

	struct RfidStatus
	{
		uint32_t rfidStatus;
	} rfidStatus;

	struct DartClientCmd
	{
		uint8_t dart_launch_opening_status;
		uint8_t reserved;
		uint16_t target_change_time;
		uint16_t latest_launch_cmd_time;
	} dartClientCmd;
	
	
	
	union Ground_robot_position_t
	{
		float alliedposall[10] ;
		struct 
	{
		float hero_x;
		float hero_y;
		float engineer_x;
		float engineer_y;
		float standard_3_x;
		float standard_3_y;
		float standard_4_x;
		float standard_4_y;
		float standard_5_x;
		float standard_5_y;
	}alliedpos;
	}ground_robot_position_t;
	
	
	struct  Sentry_info_t
	{
		uint16_t buy_bullet:11;
		uint8_t remote_buy_bullet :4;
		uint8_t remote_buy_blood :4;
		uint8_t if_revive :1;
		uint8_t if_revive_sudden :1;
		uint16_t sudden_revive_money:10;
		uint8_t none :1;
		uint8_t flee_state :1;
		uint16_t all_bullet_remain:11;
		uint8_t nonee :4;
	} sentry_info_t;

	
	struct MapCommand
	{
		float target_position_x;
		float target_position_y;
		uint8_t cmdKeyboard;
		uint8_t targetRobotId;
		uint16_t cmd_source;
	} mapCommand_orgin;//接收地图指令
	
	struct AirCommandMsg
	{
		uint16_t targetx ;
		uint16_t targety ;
		u8 aircommand ;
		u8 targetrobot ;
	} mapCommand; //云台手点位（处理后的);

	//*****************************************以下为图传链路，接收端
	struct StudentInteractiveData
	{
		uint16_t dataCmdId;
		uint16_t senderId;
		uint16_t receiverId;
		uint8_t data[113];
	} studentRecviveData;
	
	//*****************************************以下为发送端
	
	struct Sentry_cmd
	{
		uint8_t revive:1;
		uint8_t revive_sudden :1;
		uint16_t emption_bullet :11;
		uint8_t emption_bullet_distant :4;
		uint8_t emption_blood_distant :4;
		uint16_t reserve_bit :11; //保留位
	};

	struct SendUnionData
	{
		struct FrameHeader frameHeader;
		uint16_t dataCmdId;
		uint16_t senderId;
		uint16_t receiverId;

		union
		{
			uint8_t studentSendData[113];
//			struct ClientCustomGraphicDelete clientCustomGraphicDelete;
//			struct GraphicDataStruct graphicDataStruct[7];
//			struct ClientCustomCharacter clientCustomCharacter;
			struct Sentry_cmd sentry_cmd;
		};
		uint16_t CRC16;
	} sendUnionData;
	
//哨兵给云台手发送包
	struct PathPlanningData
	{
		struct FrameHeader frameHeader;
		
		/*
			data
		*///////////////
		uint8_t intention ;
		uint16_t start_position_x ;
		uint16_t start_position_y ;
		int8_t delta_x[49] ;
		int8_t delta_y[49] ;
		uint16_t sender_id ;
		////////////////
		
		uint16_t CRC16;
	} pathPlanningData ;
	
	
	
//评论区发送包	
	struct Custom
	{
		uint16_t sender_id;
		uint16_t receiver_id;
		uint8_t user_data[30];
	};
	
	struct Custom_info
	{
		struct FrameHeader frameHeader;
		struct Custom custom;
		uint16_t CRC16;
	}custom_info;
	
	
	
	struct WrongStatusCnt
	{
		int CRC8_Wrong_cnt;	 //CRC8校验错误次数累计
		int CRC16_Wrong_cnt; //CRC16校验错误次数累计
		int CYCLE_Wrong_cnt; //包圈错误次数累计
		int SEQ_Wrong_cnt;	 //包序号错误次数累计
							 /*******若以上错误同时出现，则需提高裁判系统解析频率与优先级（防止丢包）******/
	} wrongStatusCnt;

	
	
#pragma pack()

	u8 jgmtOffline = 0; ///<裁判系统离线
	int hurtCount = 0;	///<裁判系统受到伤害才更新装甲板,此次与上一次hurtCount不相同即为受攻击

	inline void addFullCount() { judgementFullCount++; }

private:
	
	JudgementStep judgementStep;

	uint8_t judgeDataBuffer[JUDGE_BUFFER_LENGTH];
	uint16_t judgementFullCount;

	uint8_t fullDataBuffer[128];
	void resetParseState();
	
	unsigned char getLength(FrameHeader *frameHeader);
	void getJudgeData();
	
	struct SendMessage 
	{ 
		uint8_t* data;
		uint16_t size; 
	};
	myqueue<SendMessage> sendPackage;
	
	struct RecMessage
	{
		uint8_t* data;
		uint8_t size;
		uint16_t id;
	};
	myvector<RecMessage,30> recPackage;
	
};

void ringQueue();
extern Judgement judgement;

#endif