#include "chassis_task.h"
#include "ins_task.h"
#include "math.h"
void ChassisStateMachineUpdate(void);
void ChassisCommandUpdate(void);
void ChassisOutputUpdate(void);
float loop_fp32_constrain(float Input, float minValue, float maxValue);
/***
 * @brief chassis task
 */
RC_ctrl_t remote_control;
Chassis_t chassis;
uint16_t offline_time = 0;

pid_type_def Yaw_Angle_pid;

float Command_Yaw_Angle = 0, Yaw_Angle = 0;
float CHASSIS_3508_SPEED_PID[3] = {CHASSIS_3508_SPEED_KP, CHASSIS_3508_SPEED_KI, CHASSIS_3508_SPEED_KD};
float Yaw_Angle_PID[3] = {YAW_ANGEL_KP, YAW_ANGEL_KI, YAW_ANGEL_KD};

void StartChassisTask(void *argument)
{
    PID_init(&chassis.ChassisPid.WheelLeftBackward, PID_POSITION, CHASSIS_3508_SPEED_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    PID_init(&chassis.ChassisPid.WheelLeftForward, PID_POSITION, CHASSIS_3508_SPEED_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    PID_init(&chassis.ChassisPid.WheelRightBackward, PID_POSITION, CHASSIS_3508_SPEED_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    PID_init(&chassis.ChassisPid.WheelRightForward, PID_POSITION, CHASSIS_3508_SPEED_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    PID_init(&Yaw_Angle_pid, PID_POSITION, Yaw_Angle_PID, 1, 0);
    while (1)
    {
        remote_control = *get_remote_control_point(); // update remote contorl data
        ChassisStateMachineUpdate();
        ChassisCommandUpdate();
        ChassisMotorMeasureUpdate();
        ChassisOutputUpdate();
        osDelay(1);
    }
}
/**
 * @brief 根据遥控器拨杆来决定机器人的状态
 */
void ChassisStateMachineUpdate(void)
{
    // 底盘状态机
    if (offline_time > 300) // 离线保护
    {
        chassis.StateMachine = CHASSIS_NO_FORCE;
        return;
    }
    switch (remote_control.rc.s[0]) // 遥控的右拨杆决定是否有力
    {
    case RC_SW_UP:
        chassis.StateMachine = CHASSIS_FORCE;
        break;
    default:
        chassis.StateMachine = CHASSIS_NO_FORCE;
        break;
    }
    if (chassis.StateMachine == CHASSIS_FORCE)
    {
        if (remote_control.rc.s[1] == RC_SW_DOWN) // 遥控的左拨杆决定是否在机体坐标系下控制
        {
            chassis.StateMachine = CHASSIS_CONTROL_IN_BODY;
        }
        else
        {
            chassis.StateMachine = CHASSIS_CONTROL_IN_WORD;
        }
    }
}

// velocity in body
float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vz = 0;
// velocity in word
float Vx = 0;
float Vy = 0;

float theta = 0; // 机体系相对于世界坐标系的角度

/**
 * @brief 根据遥控器的杆量来得到底盘的速度
 */
void ChassisCommandUpdate(void)
{
    static float pi = 3.1415926;
    float coefficient = 0.0f;
    theta = INS_angle[0] + pi / 4;
    if (theta > 2 * pi)
    {
        theta -= 2 * pi;
    }
    else if (theta < -2 * pi)
    {
        theta += 2 * pi;
    }
    // 底盘的速度(坐标系为右手系)
    if (chassis.StateMachine == CHASSIS_CONTROL_IN_BODY)
    {
        chassis_vx = remote_control.rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_VX_RC_SENSITIVITY;
        chassis_vy = -remote_control.rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_VY_RC_SENSITIVITY;
        chassis_vz = -remote_control.rc.ch[CHASSIS_Z_CHANNEL] * CHASSIS_WZ_RC_SENSITIVITY;

        chassis.ChassisCommand.WheelLeftForward = 0.35355 * (chassis_vx - chassis_vy) - MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz;
        chassis.ChassisCommand.WheelLeftBackward = 0.35355 * (chassis_vx + chassis_vy) - MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz;
        chassis.ChassisCommand.WheelRightBackward = -(0.35355 * (chassis_vx - chassis_vy) + MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz);
        chassis.ChassisCommand.WheelRightForward = -(0.35355 * (chassis_vx + chassis_vy) + MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz);
    }
    else if (chassis.StateMachine == CHASSIS_CONTROL_IN_WORD)
    {
        Vx = remote_control.rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_VX_RC_SENSITIVITY;
        Vy = -remote_control.rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_VY_RC_SENSITIVITY;
        chassis_vx = Vx * cos(theta) + Vy * (sin(theta));
        chassis_vy = Vx * (-sin(theta)) + Vy * cos(theta);
        chassis_vz = -remote_control.rc.ch[CHASSIS_Z_CHANNEL] * CHASSIS_WZ_RC_SENSITIVITY;

        // coefficient = 1 / (2 * (cos(theta) + sin(theta)));
        // chassis.ChassisCommand.WheelLeftForward = coefficient * (chassis_vy - chassis_vx) + MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz;
        // chassis.ChassisCommand.WheelLeftBackward = coefficient * (chassis_vy + chassis_vx) + MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz;
        // chassis.ChassisCommand.WheelRightBackward = -coefficient * (chassis_vy - chassis_vx) + MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz;
        // chassis.ChassisCommand.WheelRightForward = -coefficient * (chassis_vy + chassis_vx) + MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz;
        chassis.ChassisCommand.WheelLeftForward = 0.35355 * (chassis_vx - chassis_vy) - MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz;
        chassis.ChassisCommand.WheelLeftBackward = 0.35355 * (chassis_vx + chassis_vy) - MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz;
        chassis.ChassisCommand.WheelRightBackward = -(0.35355 * (chassis_vx - chassis_vy) + MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz);
        chassis.ChassisCommand.WheelRightForward = -(0.35355 * (chassis_vx + chassis_vy) + MOTOR_DISTANCE_TO_CENTER / 4.0F * chassis_vz);
    }
    else
    {
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        chassis_vz = 0.0f;
    }

    return;
}
/**
 * @brief pid控制 & 电机控制
 */
void ChassisOutputUpdate(void)
{
    if (chassis.StateMachine == CHASSIS_NO_FORCE)
    {
        CAN_cmd_chassis(0, 0, 0, 0);
        return;
    }
    else
    {
        // pid控制

        chassis.ChassisOutput.WheelLeftForward = PID_calc(&chassis.ChassisPid.WheelLeftForward, chassis.ChassisMotorMeasure.WheelLeftForward, chassis.ChassisCommand.WheelLeftForward);
        chassis.ChassisOutput.WheelLeftBackward = PID_calc(&chassis.ChassisPid.WheelLeftBackward, chassis.ChassisMotorMeasure.WheelLeftBackward, chassis.ChassisCommand.WheelLeftBackward);
        chassis.ChassisOutput.WheelRightBackward = PID_calc(&chassis.ChassisPid.WheelRightBackward, chassis.ChassisMotorMeasure.WheelRightBackward, chassis.ChassisCommand.WheelRightBackward);
        chassis.ChassisOutput.WheelRightForward = PID_calc(&chassis.ChassisPid.WheelRightForward, chassis.ChassisMotorMeasure.WheelRightForward, chassis.ChassisCommand.WheelRightForward);

        CAN_cmd_chassis(chassis.ChassisOutput.WheelLeftForward, chassis.ChassisOutput.WheelLeftBackward, chassis.ChassisOutput.WheelRightBackward, chassis.ChassisOutput.WheelRightForward);
        // CAN_cmd_chassis(1000,1000,1000,1000);
    }
}
//
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}