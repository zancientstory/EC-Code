#ifndef MOTORDRV_H
#define MOTORDRV_H

#include "cmsis_os.h"
#include "joint_motor.h"
#include "feet_motor.h"

extern void MotorDrvTask(void const *argument);

#endif
