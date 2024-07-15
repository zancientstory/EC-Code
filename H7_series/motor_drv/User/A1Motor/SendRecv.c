#include "SendRecv.h"

uint32_t SystemTimer;
int correct_time1 = 0, correct_time2 = 0;
int correct_time;

uint8_t SendData[34], RecvData[78];
uint8_t array4[4], array2[2];
uint32_t test;
uint32_t crcdata[7];
void modify_data(MOTOR_send *motor_s)
{
	motor_s->hex_len = 34;
	motor_s->motor_send_data.head.start[0] = 0xFE;
	motor_s->motor_send_data.head.start[1] = 0xEE;
	motor_s->motor_send_data.head.motorID = motor_s->id;
	motor_s->motor_send_data.head.reserved = 0x0;
	motor_s->motor_send_data.Mdata.mode = motor_s->mode;
	motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
	motor_s->motor_send_data.Mdata.ReadBit = 0x0;
	motor_s->motor_send_data.Mdata.reserved = 0x0;
	motor_s->motor_send_data.Mdata.Modify.L = 0;
	motor_s->motor_send_data.Mdata.T = motor_s->T * 256;
	motor_s->motor_send_data.Mdata.W = motor_s->W * 128;
	motor_s->motor_send_data.Mdata.Pos = (float)((motor_s->Pos / 6.2832) * 16384.0);
	motor_s->motor_send_data.Mdata.K_P = motor_s->K_P * 2048;
	motor_s->motor_send_data.Mdata.K_W = motor_s->K_W * 1024;

	motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
	motor_s->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
	motor_s->motor_send_data.Mdata.Res[0] = motor_s->Res;
	motor_s->motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&(motor_s->motor_send_data)), 7);
}
uint32_t lati, dec;
bool extract_data(uint8_t id, MOTOR_recv *motor_r)
{

	if (id == motor_r->motor_recv_data.head.motorID)
	{
		dec = id;
		lati = SystemTimer;
		motor_r->mode = motor_r->motor_recv_data.Mdata.mode;
		motor_r->Temp = motor_r->motor_recv_data.Mdata.Temp;
		motor_r->MError = motor_r->motor_recv_data.Mdata.MError;
		motor_r->T = (float)(((float)motor_r->motor_recv_data.Mdata.T) / 256.0f);
		motor_r->W = (float)(((float)motor_r->motor_recv_data.Mdata.W) / 128.0f);
		motor_r->LW = motor_r->motor_recv_data.Mdata.LW;

		motor_r->Acc = (int)motor_r->motor_recv_data.Mdata.Acc;
		motor_r->Pos = 6.2832f * ((float)motor_r->motor_recv_data.Mdata.Pos) / 16384.0f;

		motor_r->gyro[0] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[0]) * 0.00107993176f);
		motor_r->gyro[1] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[1]) * 0.00107993176f);
		motor_r->gyro[2] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[2]) * 0.00107993176f);

		motor_r->acc[0] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[0]) * 0.0023911132f);
		motor_r->acc[1] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[1]) * 0.0023911132f);
		motor_r->acc[2] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[2]) * 0.0023911132f);
		return true;
	}

	return false;

}