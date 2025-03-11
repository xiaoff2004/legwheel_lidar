#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"
#include "my_config.h"



typedef struct
{
  Joint_Motor_t joint_motor[6];
  Wheel_Motor_t wheel_motor[2];
	
	float v_set;//�����ٶȣ���λ��m/s
	float x_set;//����λ�ã���λ��m
	float turn_set;//����yaw�ỡ��
	float roll_set;	//����roll�ỡ��
	float leg_set;//�����ȳ�����λ��m
	float last_leg_set;

	float v_filter;//�˲���ĳ����ٶȣ���λ��m/s
	float x_filter;//�˲���ĳ���λ�ã���λ��m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//���ȼн����
		
	float turn_T;//yaw�Ჹ��
	float roll_f0;//roll�Ჹ��
	float leg_tp;//�����油��
	
	uint8_t start_flag;//������־
	uint8_t body_offground_flag;//������ر�־
	uint8_t leg_change_flag;//�ȳ��ı��־

	
	uint8_t jump_flag;//��Ծ��־
	uint8_t jump_flag2;//��Ծ��־
	
	uint8_t prejump_flag;//Ԥ��Ծ��־
	uint8_t recover_flag;//һ������µĵ��������־

	
} chassis_t;

extern float LQR_K_R[12];

extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legr);
extern void ChassisR_task(void);
extern void Pensation_init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn);
extern void mySaturate(float *in,float min,float max);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypeDef *leg);
extern void right_control(void);
#endif




