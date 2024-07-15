#ifndef __AttitudeThread_H
#define __AttitudeThread_H
#include "stdint.h"

/**
 * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void StartINSTask(void *argument);
const float *get_INS_angle_point(void);
extern float INS_angle[3];

#endif
