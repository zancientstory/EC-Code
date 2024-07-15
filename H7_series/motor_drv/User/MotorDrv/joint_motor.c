#include "joint_motor.h"

USART_SENDRECEIVETYPE Usart2Type __attribute__((section(".ARM.__at_0x24000000")));
USART_SENDRECEIVETYPE Usart3Type __attribute__((section(".ARM.__at_0x24000000")));
uint8_t rx_buffer[50] __attribute__((section(".ARM.__at_0x24000000")));

MOTOR_send motor_left1;
MOTOR_send motor_left2;
MOTOR_send motor_right1;
MOTOR_send motor_right2;
MOTOR_recv motor_left1_r;
MOTOR_recv motor_left2_r;
MOTOR_recv motor_right1_r;
MOTOR_recv motor_right2_r;

double x_distance = 0, y_distance_left = 220, y_distance_right = 220, angle = 0;

void joint_motor_set_mode(int MODE)
{

	motor_left1.mode = MODE;
	motor_left1.id = JOINT_LEFT1_ID;
	motor_left1_r.motor_id = JOINT_LEFT1_ID;

	motor_left2.mode = MODE;
	motor_left2.id = JOINT_LEFT2_ID;
	motor_left2_r.motor_id = JOINT_LEFT2_ID;

	motor_right1.mode = MODE;
	motor_right1.id = JOINT_RIGHT1_ID;
	motor_right1_r.motor_id = JOINT_RIGHT1_ID;

	motor_right2.mode = MODE;
	motor_right2.id = JOINT_RIGHT2_ID;
	motor_right2_r.motor_id = JOINT_RIGHT2_ID;
}

void Stand_left1_t(double T)
{
	motor_left1.K_P = 0;
	motor_left1.K_W = 0;
	motor_left1.Pos = 0;
	motor_left1.T = T;
	modify_data(&motor_left1);
	memcpy(Usart2Type.TX_pData, (uint8_t *)&(motor_left1.motor_send_data), SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart2Type.TX_pData), 34);
	HAL_UART_Transmit_DMA(&huart2, Usart2Type.TX_pData, SEND_SIZE);
}

void Stand_left2_t(double T)
{
	motor_left2.K_P = 0;
	motor_left2.K_W = 0;
	motor_left2.Pos = 0;
	motor_left2.T = T;
	modify_data(&motor_left2);
	memcpy(Usart2Type.TX_pData, (uint8_t *)&(motor_left2.motor_send_data), SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart2Type.TX_pData), 34);
	HAL_UART_Transmit_DMA(&huart2, Usart2Type.TX_pData, SEND_SIZE);
}

void Stand_right1_t(double T)
{
	motor_right1.K_P = 0;
	motor_right1.K_W = 0;
	motor_right1.Pos = 0;
	motor_right1.T = T;
	modify_data(&motor_right1);
	memcpy(Usart3Type.TX_pData, (uint8_t *)&(motor_right1.motor_send_data), SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart3Type.TX_pData), 34);
	HAL_UART_Transmit_DMA(&huart3, Usart3Type.TX_pData, SEND_SIZE);
}

void Stand_right2_t(double T)
{
	motor_right2.K_P = 0;
	motor_right2.K_W = 0;
	motor_right2.Pos = 0;
	motor_right2.T = T;
	modify_data(&motor_right2);
	memcpy(Usart3Type.TX_pData, (uint8_t *)&(motor_right2.motor_send_data), SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t *)&(Usart3Type.TX_pData), 34);
	HAL_UART_Transmit_DMA(&huart3, Usart3Type.TX_pData, SEND_SIZE);
}
