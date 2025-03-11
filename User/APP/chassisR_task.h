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
	
	float v_set;//期望速度，单位是m/s
	float x_set;//期望位置，单位是m
	float turn_set;//期望yaw轴弧度
	float roll_set;	//期望roll轴弧度
	float leg_set;//期望腿长，单位是m
	float last_leg_set;

	float v_filter;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//两腿夹角误差
		
	float turn_T;//yaw轴补偿
	float roll_f0;//roll轴补偿
	float leg_tp;//防劈叉补偿
	
	uint8_t start_flag;//启动标志
	uint8_t body_offground_flag;//车体离地标志
	uint8_t leg_change_flag;//腿长改变标志

	
	uint8_t jump_flag;//跳跃标志
	uint8_t jump_flag2;//跳跃标志
	
	uint8_t prejump_flag;//预跳跃标志
	uint8_t recover_flag;//一种情况下的倒地自起标志

	
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




