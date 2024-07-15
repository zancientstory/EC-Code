/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "bsp_can.h"
#include "chassis_task.h"

// rm motor data
typedef enum
{
  CAN_CHASSIS_SLAVE_ID = 0X200,
  CAN_3508_LEFTFORWARD_ID = 0x201, /*从左上开始逆时针旋转*/
  CAN_3508_LEFTBACKWARD_ID = 0x202,
  CAN_3508_RIGHTBACKWARD_ID = 0x203,
  CAN_3508_RIGHTFORWARD_ID = 0x204,
} can_msg_id_e;
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} motor_measure_t;
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void ChassisMotorMeasureUpdate(void);
#endif
