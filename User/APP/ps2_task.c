/**
  *********************************************************************
  * @file      ps2_task.c/h
  * @brief     该任务是读取并处理ps2手柄传来的遥控数据，
	*            将遥控数据转化为期望的速度、期望的转角、期望的腿长等
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "ps2_task.h"
#include "cmsis_os.h"
#include "my_config.h"
#include "usart.h"
#include "chassisR_task.h"
#include "chassisL_task.h"
#include "gpio.h"
#include "gimbal_task.h"


extern chassis_t chassis_move;
extern INS_t INS;
extern vmc_leg_t left;
extern vmc_leg_t right;
uint8_t rx_buff[BUFF_SIZE * 2];
uint8_t rx2_buff[BUFF2_SIZE * 2];
uint8_t remoter_buff[BUFF2_SIZE];
remoter_t remoter;


float map_range (float input, float in_min, float in_max, float out_min, float out_max)
{
	return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void pstwo_task (void)
{
	static uint32_t remoter_last_rx_time = 0;
	memset (&remoter, 0, sizeof (remoter_t));
	HAL_UARTEx_ReceiveToIdle_DMA (&huart5, rx_buff, BUFF_SIZE * 2);
	HAL_UARTEx_ReceiveToIdle_DMA (&huart7, rx2_buff, BUFF2_SIZE * 2);
	while (1) {
//		if(remoter.sbus_recever_time - remoter_last_rx_time <= 500 && remoter.sbus_recever_time != 0)
//		{
//			sbus_frame_parse (&remoter, remoter_buff);
//			remoter_last_rx_time = remoter.sbus_recever_time;
//		}

		sbus_frame_parse (&remoter, remoter_buff);
		if (remoter . toggle . SF == RC_SW_UP && remoter . online == 1) {
			chassis_move . start_flag = 1;
		}
		else {
			chassis_move . start_flag = 0;
			chassis_move . x_filter = 0.0f;
			chassis_move . x_set = chassis_move . x_filter;
			chassis_move . leg_set = LEG_INIT;
			chassis_move . turn_set = chassis_move . total_yaw;
			gimbal_motor_tar_pos[0] = chassis_move . joint_motor[4] . para . pos;
			gimbal_motor_tar_pos[1] = chassis_move . joint_motor[5] . para . pos;
		}
//		vofa_demo (left.torque_set[1], left.torque_set[0],
//				   chassis_move.wheel_motor[0].wheel_T, chassis_move.wheel_motor[1].wheel_T);
		switch (remoter . toggle . SA) {
			case RC_SW_UP:
				if (remoter . rc . ch[RC_CH_RY]) {
					chassis_move . leg_set += (float) remoter . rc . ch[RC_CH_RY] * 0.00001f;//调腿长
					chassis_move . leg_change_flag = 1;
				}
				else {
					chassis_move . leg_change_flag = 0;
				}
				if (chassis_move . leg_set > LEG_MAX) {
					chassis_move . leg_set = LEG_MAX;
				}
				else if (chassis_move . leg_set < LEG_MIN) {
					chassis_move . leg_set = LEG_MIN;
				}
				if (remoter . toggle . SB == RC_SW_MID) {
					chassis_move . leg_set = LEG_INIT;
				}
				break;
			case RC_SW_MID: chassis_move . v_set = (float) remoter . rc . ch[RC_CH_RY] * 0.00004f;
//				chassis_move.x_set = 0;
				chassis_move . x_set += chassis_move . v_set;
				chassis_move . turn_set -= (float) remoter . rc . ch[RC_CH_RX] * 0.0001f;
				data_view[9] = chassis_move . v_set;

//				gimbal_motor_tar_pos[0] -=(float)remoter.rc.ch[RC_CH_RX] * 0.0001f;
//				if(gimbal_motor_tar_pos[0] > 1) gimbal_motor_tar_pos[0] = 1;
//				else if(gimbal_motor_tar_pos[0] < -0.6f) gimbal_motor_tar_pos[0] = -0.6f;
//
//				gimbal_motor_tar_pos[0] -=(float)remoter.rc.ch[RC_CH_LX] * 0.0001f;
//				if(gimbal_motor_tar_pos[0] > PI/2) gimbal_motor_tar_pos[0] = PI/2;
//				else if(gimbal_motor_tar_pos[0] < -PI/2) gimbal_motor_tar_pos[0] = -PI/2;
//
//				gimbal_motor_tar_pos[1] = map_range ((float )remoter.rc.ch[RC_CH_LY],-671,671,-0.6,1);
//
//				data_view[10] = gimbal_motor_tar_pos[0];
//				data_view[11] = gimbal_motor_tar_pos[1];
				break;
			case RC_SW_DOWN:

				break;
			default: break;
		}
		switch (remoter . toggle . SC) {
			case RC_SW_DOWN: remoter . rc . ch[RC_CH_LX] += 8;
				gimbal_motor_tar_pos[0] -= (float) remoter . rc . ch[RC_CH_LX] * 0.0001f;
				if (gimbal_motor_tar_pos[0] > PI / 2) { gimbal_motor_tar_pos[0] = PI / 2; }
				else if (gimbal_motor_tar_pos[0] < -PI / 2) { gimbal_motor_tar_pos[0] = -PI / 2; }

				gimbal_motor_tar_pos[1] = map_range ((float) remoter . rc . ch[RC_CH_LY], -671, 671, -0.6, 1);
				switch (remoter . toggle . SD) {
					case RC_SW_UP: break;
					case RC_SW_MID: Power_OUT1_ON;
						break;
					case RC_SW_DOWN: Power_OUT1_OFF;
						break;
					default: break;
				}
				break;
			case RC_SW_MID: gimbal_motor_tar_pos[0] = map_range (rx_commu . motor_yaw, -1, 1, -PI / 2, PI / 2);
				gimbal_motor_tar_pos[1] = map_range (rx_commu . motor_pitch, -1, 1, -0.6, 0.6);
				if(rx_commu.shoot)
					Power_OUT1_ON;
				else
					Power_OUT1_OFF;
				break;


		}
		//671
		
		vofa_send_indefinite_len_data (data_view, DATA_VIEW_SEND_LEN);
		osDelay (PS2_TIME);
	}
}


void sbus_frame_parse (remoter_t *remoter, uint8_t *buf)
{
	if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END)) {
		return;
	}

	if (buf[23] == 0x0C) {
		remoter -> online = 0;
	}
	else {
		remoter -> online = 1;
	}

	remoter -> rc . ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF) - REMOTE_RC_OFFSET;
	remoter -> rc . ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF) - REMOTE_RC_OFFSET;
	remoter -> rc . ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF) - REMOTE_RC_OFFSET;
	remoter -> rc . ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF) - REMOTE_RC_OFFSET;
	remoter -> rc . ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
	remoter -> rc . ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
	remoter -> rc . ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
	remoter -> rc . ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
	remoter -> rc . ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
	remoter -> rc . ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);

	switch (remoter -> rc . ch[4]) {
		case 0x161: remoter -> toggle . SA = RC_SW_UP;
			break;
		case 0x0400: remoter -> toggle . SA = RC_SW_MID;
			break;
		case 0x069F: remoter -> toggle . SA = RC_SW_DOWN;
			break;
	}
	switch (remoter -> rc . ch[5]) {
		case 0x161: remoter -> toggle . SB = RC_SW_UP;
			break;
		case 0x0400: remoter -> toggle . SB = RC_SW_MID;
			break;
		case 0x069F: remoter -> toggle . SB = RC_SW_DOWN;
			break;
	}
	switch (remoter -> rc . ch[6]) {
		case 0x161: remoter -> toggle . SC = RC_SW_UP;
			break;
		case 0x0400: remoter -> toggle . SC = RC_SW_MID;
			break;
		case 0x069F: remoter -> toggle . SC = RC_SW_DOWN;
			break;
	}
	switch (remoter -> rc . ch[7]) {
		case 0x161: remoter -> toggle . SD = RC_SW_UP;
			break;
		case 0x0400: remoter -> toggle . SD = RC_SW_MID;
			break;
		case 0x069F: remoter -> toggle . SD = RC_SW_DOWN;
			break;
	}
	switch (remoter -> rc . ch[8]) {
		case 0x161: remoter -> toggle . SF = RC_SW_UP;
			break;
		case 0x069F: remoter -> toggle . SF = RC_SW_DOWN;
			break;
	}
	switch (remoter -> rc . ch[9]) {
		case 0x161: remoter -> toggle . SG = RC_SW_UP;
			break;
		case 0x069F: remoter -> toggle . SG = RC_SW_DOWN;
			break;
	}

//	remoter -> sbus_recever_time = HAL_GetTick ();
}

void HAL_UARTEx_RxEventCallback (UART_HandleTypeDef *huart, uint16_t Size)
{

	if (huart -> Instance == UART5) {
		if (Size <= BUFF_SIZE) {
			HAL_UARTEx_ReceiveToIdle_DMA (&huart5, rx_buff, BUFF_SIZE * 2); // 接收完毕后重启
			memcpy (remoter_buff, rx_buff, BUFF_SIZE);
			remoter . sbus_recever_time = HAL_GetTick ();

//			memset(rx_buff, 0, BUFF_SIZE);
		}
		else  // 接收数据长度大于BUFF_SIZE，错误处理
		{
			HAL_UARTEx_ReceiveToIdle_DMA (&huart5, rx_buff, BUFF_SIZE * 2); // 接收完毕后重启
			memset (rx_buff, 0, BUFF_SIZE);
		}
	}
	else if (huart -> Instance == UART7) {
		if (Size == BUFF2_SIZE && rx2_buff[0] == 0xAA && rx2_buff[1] == 0xBB && rx2_buff[Size - 2] == 0xCC &&
		    rx2_buff[Size - 1] == 0xDD) {
			HAL_UARTEx_ReceiveToIdle_DMA (&huart7, rx2_buff, BUFF2_SIZE * 2); // 接收完毕后重启
			memcpy (&LQR_K_R, rx2_buff + 2, sizeof (float) * 12);
			memcpy (&LQR_K_L, rx2_buff + 2, sizeof (float) * 12);
			data_view[14]++;
			memset (rx2_buff, 0, BUFF2_SIZE * 2);
			chassis_move . x_filter = 0;
			chassis_move . turn_set = chassis_move . total_yaw;
		}
		else {
			HAL_UARTEx_ReceiveToIdle_DMA (&huart7, rx2_buff, BUFF2_SIZE * 2); // 接收完毕后重启
			memset (rx2_buff, 0, BUFF2_SIZE * 2);
		}

	}
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef *huart)
{
	if (huart -> Instance == UART5) {
		HAL_UARTEx_ReceiveToIdle_DMA (&huart5, rx_buff, BUFF_SIZE * 2); // 接收发生错误后重启
		memset (rx_buff, 0, BUFF_SIZE);                               // 清除接收缓存
	}
}
