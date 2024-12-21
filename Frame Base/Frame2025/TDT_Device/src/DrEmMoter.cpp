/*
Author: skywalker&codeeartist
TIME:2024_3_30

*exp:
*   
		以及一堆命令
*/

#include "DrEmMoter.h"
#include "dbus.h"
format_data_struct data_list;
uint8_t rx_buffer[8];
static inline void uint16_to_data(uint16_t val, uint8_t *data)
{
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint16_t data_to_uint16(uint8_t *data)
{
    uint16_t tmp_uint16;
    tmp_uint16 = (((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint16;
}
static inline void uint_to_data(uint32_t val, uint8_t *data)
{
    data[3] = (uint8_t)(val >> 24);
    data[2] = (uint8_t)(val >> 16);
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint32_t data_to_uint(uint8_t *data)
{
    uint32_t tmp_uint;
    tmp_uint = (((uint32_t)data[3] << 24) + ((uint32_t)data[2] << 16) + ((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint;
}
static inline void int16_to_data(int16_t val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
}
static inline int16_t data_to_int16(uint8_t *data)
{
    int16_t tmp_int16;
    *(((uint8_t*)(&tmp_int16)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int16)) + 1) = data[1];
    return tmp_int16;
}
static inline void int_to_data(int val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline int data_to_int(uint8_t *data)
{
    int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}
static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

//参数结构体指针，要做的操作（五种type转byte输入“encode”，byte转五种type输入“decode”）
void format_data(float *value_data, int *type_data,int length, char * str)
{
    data_list.length=length;
    for (int i = 0; i < length; i++)
    {
        data_list.value_data[i]= value_data[i];
        data_list.type_data[i] = type_data[i];
    }
    if (strcmp(str,"encode")==0)
    {
        value2byte();
    }
    if (strcmp(str,"decode")==0)
    {
        for (int i = 0; i < 8; i++)
        {
            data_list.byte_data[i]=rx_buffer[i];
        }
        byte2value();
    }
}

//不需主动调用
void byte2value()
{
    int value_index = 0;
    int byte_index = 0;
    while (1)
    {
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            data_list.value_data[value_index] = (float)data_to_float(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 1:
        {
            data_list.value_data[value_index] = (float)data_to_uint16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 2:
        {
            data_list.value_data[value_index] = (float)data_to_int16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 3:
        {
            data_list.value_data[value_index] = (float)data_to_uint(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 4:
        {
            data_list.value_data[value_index] = (float)data_to_int(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index>=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}
//不需主动调用
void value2byte()
{
    int byte_index=0;
    int value_index=0;
    while(1)
    {
        if (data_list.type_data[value_index]==0)
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 1:
        {
            uint16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 2:
        {
            int16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 3:
        {
            uint_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 4:
        {
            int_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index >=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}


void DR_Motor_Rec::GetOriginData(CanInfo *Info,CanRxMsg *_CanRxMsg)
{
	
	rx_buffer[0] = _CanRxMsg->Data[0];
	rx_buffer[1] = _CanRxMsg->Data[1];
	rx_buffer[2] = _CanRxMsg->Data[2];
	rx_buffer[3] = _CanRxMsg->Data[3];
	rx_buffer[4] = _CanRxMsg->Data[4];
	rx_buffer[5] = _CanRxMsg->Data[5];
	rx_buffer[6] = _CanRxMsg->Data[6];
	rx_buffer[7] = _CanRxMsg->Data[7];
	
	float factor = 0.01;
	float value_data[3]= {0,0,0};
	int type_data[3]= {0,2,2};
	format_data(value_data,type_data,3,(char *)"decode");
	
	
	Info->encoder = data_list.value_data[0];
	Info->speed = data_list.value_data[1]*factor;
}

void DR_Motor::SendCurrentAgreement(uint8_t canbuff[8],float canResult)
{
	memset(canbuff,0,8);
  uint8_t id_num = CanId;
  float factor = 0.01;
  int u16_input_mode,s16_ramp_rate;
  float f_torque = canResult;
  u16_input_mode = 6;
  s16_ramp_rate = 0;
  float value_data[3]= {f_torque, (float)s16_ramp_rate,(float)u16_input_mode};
  int type_data[3]= {0,2,1};
  format_data(value_data,type_data,3,(char *)"encode");
	
	memcpy(canbuff,data_list.byte_data,8);
}


void DR_Motor::readCommand()
{
  float value_data[3]= {0x00,0x00,0};
  int type_data[3]= {1,1,3};
  format_data(value_data,type_data,3,(char *)"encode");
  memcpy(canTxBuff,data_list.byte_data,8);
}

