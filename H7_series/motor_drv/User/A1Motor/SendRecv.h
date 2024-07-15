#ifndef SENDRECV_H
#define SENDRECV_H
#include "unitreeMotor.h"
#include "crc32.h"  
#include "stm32h7xx_it.h"
typedef __PACKED_STRUCT
{
    float Pos,T,W;
}Can_Recv;

extern uint8_t SendData[34],RecvData[78];
extern uint32_t	SystemTimer;
extern void modify_data(MOTOR_send *motor_s);
extern bool extract_data(uint8_t id,MOTOR_recv* motor_r);


extern int correct_time1,correct_time2,correct_time;

#endif

