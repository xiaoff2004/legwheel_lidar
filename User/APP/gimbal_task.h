#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H


#include "main.h"
#include "stm32h7xx.h"

typedef struct __attribute__((packed))
{
	uint8_t header;//0x5A
	float imu_roll;
	float imu_pitch;
	float imu_yaw;
	float motor_pitch;
	float motor_yaw;
	uint16_t checksum;
}tx_commu_t;


typedef struct __attribute__((packed))
{
	uint8_t header;//0xA5
	float motor_pitch;
	float motor_yaw;
	uint8_t shoot;
	uint16_t checksum;
}rx_commu_t;

extern float gimbal_motor_tar_pos[2];
extern float gimbal_motor_tar_pos_vision[2];
extern void my_gimbal_task(void);
extern rx_commu_t rx_commu;
extern uint8_t rx_buf[sizeof (rx_commu)];

#endif