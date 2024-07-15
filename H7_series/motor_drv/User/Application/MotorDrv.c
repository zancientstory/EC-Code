#include "MotorDrv.h"
int A1_motor_mode = 0; // 切换为11可以重置电机id；
double torque = 0.0;
int flag = 0;
void MotorDrvTask(void const *argument)
{

	while (1)
	{

		joint_motor_set_mode(A1_motor_mode);
		Stand_left1_t(0);
		Stand_right1_t(0);
		osDelay(1);
		Stand_left2_t(0);
		Stand_right2_t(0);
		osDelay(1);

		FEET_CONTROL(torque, torque);
	}
}
