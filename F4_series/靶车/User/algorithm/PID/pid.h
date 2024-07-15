/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history    2021-7-23   增加了串级pid的函数和接口
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V2.0.0     Jly-23-2021     Qylann          1. 增加部分函数
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "stdint.h"

enum PID_MODE
{
  PID_POSITION = 0,
  PID_DELTA
};
typedef struct
{
  uint8_t mode;
  // PID 三参数
  float Kp;
  float Ki;
  float Kd;

  float max_out;  // 最大输出
  float max_iout; // 最大积分输出

  float set;
  float fdb;

  float out;
  float Pout;
  float Iout;
  float Dout;
  float Dbuf[3];  // 微分项 0最新 1上一次 2上上次
  float error[3]; // 误差项 0最新 1上一次 2上上次

} pid_type_def;

typedef struct
{
  pid_type_def pid_inside;
  pid_type_def pid_outside;

  float s_set;
  float s_fdb;
  float v_set;
  float v_fdb;
  float out;
} cascade_pid_t;

/**
 * @brief          pid struct data init
 * @param[out]     pid: PID struct data point
 * @param[in]      mode: PID_POSITION: normal pid
 *                 PID_DELTA: delta pid
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid max out
 * @param[in]      max_iout: pid max iout
 * @retval         none
 */
/**
 * @brief          pid struct data init
 * @param[out]     pid: PID结构数据指针
 * @param[in]      mode: PID_POSITION:普通PID
 *                 PID_DELTA: 差分PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 * @retval         none
 */
extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
 * @brief          cascade pid struct data init
 * @param[out]     c_pid: 差分PID结构数据指针
 * @param[in]      oPID: 0: kp, 1: ki, 2:kd
 * @param[in]      iPID: 0: kp, 1: ki, 2:kd
 * @param[in]      oPID_max_out: 外环pid最大输出
 * @param[in]      oPID_max_iout: 外环pid最大积分输出
 * @param[in]      iPID_max_out: 内环pid最大输出
 * @param[in]      iPID_max_iout: 内环pid最大积分输出
 * @retval         none
 */
extern void cascade_PID_init(cascade_pid_t *c_pid, const float oPID[3], const float iPID[3], float oPID_max_out, float oPID_max_iout, float iPID_max_out, float iPID_max_iout);

/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @retval         pid输出
 */
extern float PID_calc(pid_type_def *pid, float ref, float set);

/**
 * @brief          cascade pid calculate
 * @param[out]     c_pid: cascade PID struct data point
 * @param[in]      o_ref: outside feedback data
 * @param[in]      i_ref: inside feedback data
 * @param[in]      s_set: outside set point
 * @retval         cascade pid out
 */
extern float cascade_PID_calc(cascade_pid_t *c_pid, float o_ref, float i_ref, float o_set);

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
extern void PID_clear(pid_type_def *pid);

/**
 * @brief          cascade pid out clear
 * @param[out]     c_pid: cascade PID struct data point
 * @retval         none
 */
/**
 * @brief          差分pid输出清除
 * @param[out]     c_pid: 差分PID结构数据指针
 * @retval         none
 */
extern void cascade_PID_clear(cascade_pid_t *c_pid);

#endif
