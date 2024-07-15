#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H
#include "Motor.h"
#include "Remote.h"
#include "main.h"
#include "pid.h"
#include "cmsis_os.h"

#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Y_CHANNEL 0
#define CHASSIS_Z_CHANNEL 2

#define CHASSIS_VX_RC_SENSITIVITY 100.0f
#define CHASSIS_VY_RC_SENSITIVITY 100.0F
#define CHASSIS_WZ_RC_SENSITIVITY 150.0f

#define MOTOR_DISTANCE_TO_CENTER 0.2F
// 底盘电机PID
#define CHASSIS_3508_SPEED_KP 1.5F
#define CHASSIS_3508_SPEED_KI 0.0F
#define CHASSIS_3508_SPEED_KD 0.0F

// yaw轴的角度控制pid
#define YAW_ANGEL_KP 0.2F
#define YAW_ANGEL_KI 0.0F
#define YAW_ANGEL_KD 0.0F

#define M3508_MAX_OUTPUT 16384
#define M3508_MAX_IOUTPUT 6000

typedef enum
{
    CHASSIS_NO_FORCE = 0X00,
    CHASSIS_FORCE,
    CHASSIS_CONTROL_IN_BODY,
    CHASSIS_CONTROL_IN_WORD
} ChassisStateMachine_e;
typedef struct
{
    int16_t WheelLeftForward;
    int16_t WheelLeftBackward;
    int16_t WheelRightBackward;
    int16_t WheelRightForward;
} ChassisMotorMeasure_t;

typedef struct
{
    float WheelLeftForward;
    float WheelLeftBackward;
    float WheelRightBackward;
    float WheelRightForward;
} ChassisCommand_t;
typedef struct
{
    pid_type_def WheelLeftForward;
    pid_type_def WheelLeftBackward;
    pid_type_def WheelRightBackward;
    pid_type_def WheelRightForward;
} ChassisPid_t;

typedef struct
{
    int16_t WheelLeftForward;
    int16_t WheelLeftBackward;
    int16_t WheelRightBackward;
    int16_t WheelRightForward;
} ChassisOutput_t;

typedef struct
{
    ChassisStateMachine_e StateMachine;        // 状态机
    ChassisMotorMeasure_t ChassisMotorMeasure; // 电机测量
    ChassisCommand_t ChassisCommand;           // 底盘指令
    ChassisPid_t ChassisPid;                   // 底盘PID
    ChassisOutput_t ChassisOutput;             // 底盘输出
} Chassis_t;

extern Chassis_t chassis;
extern void StartChassisTask(void *argument);
extern uint16_t offline_time;

#endif
