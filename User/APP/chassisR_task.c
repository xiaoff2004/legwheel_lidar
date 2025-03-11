/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
	*						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
	*						 右下角的DM4310发送id为8、接收id为4，
	*						 右边DM轮毂电机发送id为1、接收id为0。
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "my_config.h"

float LQR_K_R[12] = PARA;

float Poly_Coefficient[12][4] = {

		{-242.9620, 299.3549,  -162.1772, -2.3617},
		{0.4096,    0.8223,    -10.6855,  0.0679},
		{-30.0173,  32.5363,   -12.5449,  -3.1905},
		{-24.8376,  27.7830,   -12.3033,  -3.6670},
		{-75.2240,  103.2555,  -57.4417,  16.9364},
		{-2.4809,   3.9657,    -2.5241,   1.2559},
		{269.8904,  -233.2223, 51.4253,   17.4715},
		{20.6657,   -20.2606,  5.6966,    1.1458},
		{-56.8991,  80.9259,   -44.2387,  10.6338},
		{-69.6440,  93.8583,   -49.0809,  11.5144},
		{337.7607,  -367.9950, 145.8889,  4.6029},
		{22.7427,   -25.9250,  11.0196,   -0.2817}};

vmc_leg_t right;

extern INS_t INS;
extern vmc_leg_t left;

chassis_t chassis_move;

PidTypeDef LegR_Pid; // 右腿的腿长pd
PidTypeDef Roll_Pid; // 横滚角补偿pd
PidTypeDef Tp_Pid;     // 防劈叉补偿pd
PidTypeDef Turn_Pid; // 转向pd

void ChassisR_task (void)
{
	while (INS . ins_flag == 0) { // 等待加速度收敛
		osDelay (1);
	}

	ChassisR_init (&chassis_move, &right, &LegR_Pid); // 初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部
	Pensation_init (&Roll_Pid, &Tp_Pid, &Turn_Pid);     // 补偿pid初始化

	while (1) {
				chassisR_feedback_update (&chassis_move, &right, &INS);//更新数据
		//
				chassisR_control_loop (&chassis_move, &right, &INS, LQR_K_R, &LegR_Pid);//控制计算

		if (chassis_move . start_flag == 1) {
			mit_ctrl (&hfdcan1, chassis_move . joint_motor[1] . para . id, 0.0f, 0.0f, 0.0f, 0.0f,
			          right . torque_set[1]);//right.torque_set[1]
			osDelay (CHASSR_TIME);
			mit_ctrl (&hfdcan1, chassis_move . joint_motor[0] . para . id, 0.0f, 0.0f, 0.0f, 0.0f,
			          right . torque_set[0]);//right.torque_set[0]
			dji3508_ctrl (&hfdcan3, chassis_move . wheel_motor[0] . wheel_T,
			              chassis_move . wheel_motor[1] . wheel_T);//右边边轮毂电机
			osDelay (CHASSR_TIME);

//			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, -1.77634048f, 0.1f,10.0f, 0.1f,0.0f);//right.torque_set[1]
//			osDelay(CHASSR_TIME);
//			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 1.71721172f, 0.1f,10.0f, 0.1f,0.0f);//right.torque_set[0]
//			dji3508_ctrl(&hfdcan3, chassis_move.wheel_motor[0].wheel_T, chassis_move.wheel_motor[1].wheel_T);//右边边轮毂电机
//			osDelay(CHASSR_TIME);

//			mit_ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f,
//					 0.0f); // right.torque_set[1]
//			osDelay(CHASSR_TIME);
//			mit_ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f,
//					 0.0f); // right.torque_set[0]
//			// dji3508_ctrl(&hfdcan3,chassis_move.wheel_motor[0].wheel_T, 0.0f);//右边边轮毂电机
//			osDelay(CHASSR_TIME);
		}
		else if (chassis_move . start_flag == 0) {
			mit_ctrl (&hfdcan1, chassis_move . joint_motor[1] . para . id, 0.0f, 0.0f, 0.0f, 0.0f,
			          0.0f); // right.torque_set[1]
			osDelay (CHASSR_TIME);
			mit_ctrl (&hfdcan1, chassis_move . joint_motor[0] . para . id, 0.0f, 0.0f, 0.0f, 0.0f,
			          0.0f);                  // right.torque_set[0]
			dji3508_ctrl (&hfdcan3, 0, 0); // 右边边轮毂电机
			osDelay (CHASSR_TIME);
		}
	}
}

void ChassisR_init (chassis_t *chassis, vmc_leg_t *vmc, PidTypeDef *legr)
{
	const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD};

	joint_motor_init (&chassis -> joint_motor[0], 0, MIT_MODE); // 发送id为0
	joint_motor_init (&chassis -> joint_motor[1], 1, MIT_MODE); // 发送id为1

	VMC_init (vmc); // 给杆长赋值

	PID_init (legr, PID_POSITION, legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT); // 腿长pid

	for (int j = 0; j < 10; j++) {
		enable_motor_mode (&hfdcan1, chassis -> joint_motor[0] . para . id, chassis -> joint_motor[0] . mode);
		osDelay (1);
	}
	for (int j = 0; j < 10; j++) {
		enable_motor_mode (&hfdcan1, chassis -> joint_motor[1] . para . id, chassis -> joint_motor[1] . mode);
		osDelay (1);
	}

}

void Pensation_init (PidTypeDef *roll, PidTypeDef *Tp, PidTypeDef *turn)
{ // 补偿pid初始化：横滚角补偿、防劈叉补偿、偏航角补偿
	const static float roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};

	PID_init (roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	PID_init (Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT, TP_PID_MAX_IOUT);
	PID_init (turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
}

void chassisR_feedback_update (chassis_t *chassis, vmc_leg_t *vmc, INS_t *ins)
{
	vmc -> phi1 = pi / 2.0f + chassis -> joint_motor[0] . para . pos + 0.10472f;
	vmc -> phi4 = pi / 2.0f + chassis -> joint_motor[1] . para . pos - 0.10472f;

	chassis -> myPithR = ins -> Pitch;
	chassis -> myPithGyroR = ins -> Gyro[0];

	chassis -> total_yaw = ins -> YawTotalAngle;
	chassis -> roll = ins -> Roll;
	chassis -> theta_err = 0.0f - (vmc -> theta + left . theta);

	//	if (ins -> Pitch<(3.1415926f / 6.0f) && ins -> Pitch> (-3.1415926f / 6.0f)) {//根据pitch角度判断倒地自起是否完成
	//		chassis -> recover_flag = 0;
	//}
}

uint8_t right_flag = 0;
extern uint8_t left_flag;

void chassisR_control_loop (chassis_t *chassis, vmc_leg_t *vmcr, INS_t *ins, float *LQR_K, PidTypeDef *leg)
{
	static float err[6] = {0.0f};
	VMC_calc_1_right (vmcr, ins,
	                  ((float) CHASSR_TIME) * 3.0f / 1000.0f); // 计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是3*0.001秒

	for (int i = 0; i < 12; i++) {
		//		LQR_K[i] = LQR_K_calc (&Poly_Coefficient[i][0], vmcr -> L0);
		LQR_K[i] = LQR_K_R[i];
	}

	// chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算
	chassis -> turn_T =
			Turn_Pid . Kp * (chassis -> turn_set - chassis -> total_yaw) - Turn_Pid . Kd * ins -> Gyro[2]; // 这样计算更稳一点
	// chassis->roll_f0=PID_Calc(&Roll_Pid, chassis->roll,chassis->roll_set);//roll轴pid计算
	chassis -> roll_f0 = Roll_Pid . Kp * (chassis -> roll_set - chassis -> roll) - Roll_Pid . Kd * ins -> Gyro[1];
	chassis -> leg_tp = PID_Calc (&Tp_Pid, chassis -> theta_err, 0.0f); // 防劈叉pid计算

	err[0] = vmcr -> theta - 0.0f;
	err[1] = vmcr -> d_theta - 0.0f;
	err[2] = chassis -> x_filter - chassis -> x_set;
	//	err[2] = 0.0f;
	err[3] = chassis -> v_filter - chassis -> v_set;
	err[4] = chassis -> myPithR - 0.0f;
	err[5] = chassis -> myPithGyroR - 0.0f;
	chassis -> wheel_motor[0] . wheel_T = (LQR_K[0] * err[0] + LQR_K[1] * err[1] + LQR_K[2] * err[2] +
	                                       LQR_K[3] * err[3] + LQR_K[4] * err[4] + LQR_K[5] * err[5]);
	vmcr -> Tp = (LQR_K[6] * err[0] + LQR_K[7] * err[1] + LQR_K[8] * err[2] + LQR_K[9] * err[3] + LQR_K[10] * err[4] +
	              LQR_K[11] * err[5]);

	//	data_view[2] = vmcr -> theta;
	//	data_view[3] = vmcr -> d_theta;
	//	chassis -> wheel_motor[0] . wheel_T = chassis -> wheel_motor[0] . wheel_T - chassis -> turn_T;    //轮毂电机输出力矩
	mySaturate (&chassis -> wheel_motor[0] . wheel_T, -1.0f * MAX_TORQUE_DJI3508, MAX_TORQUE_DJI3508);

	//	vmcr -> Tp = vmcr -> Tp + chassis -> leg_tp;//髋关节输出力矩

	vmcr -> F0 = MG / arm_cos_f32 (vmcr -> theta) + PID_Calc (leg, vmcr -> L0, chassis -> leg_set); // 前馈+pd

	right_flag = ground_detectionR (vmcr, ins); // 右腿离地检测

	if (chassis -> recover_flag == 0) { // 倒地自起不需要检测是否离地
		if (right_flag == 1 && left_flag == 1 && vmcr -> leg_flag == 0) { // 当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			chassis -> wheel_motor[0] . wheel_T = 0.0f;
			vmcr -> Tp = LQR_K[6] * (vmcr -> theta - 0.0f) + LQR_K[7] * (vmcr -> d_theta - 0.0f);

			chassis -> x_filter = 0.0f;
			chassis -> x_set = chassis -> x_filter;
			chassis -> turn_set = chassis -> total_yaw;
			vmcr -> Tp = vmcr -> Tp + chassis -> leg_tp;
		}
		else {                        // 没有离地
			vmcr -> leg_flag = 0; // 置为0

			vmcr -> F0 = vmcr -> F0 - chassis -> roll_f0; // roll轴补偿取反然后加上去
		}
	}
	else if (chassis -> recover_flag == 1) {
		vmcr -> Tp = 0.0f;
	}

	mySaturate (&vmcr -> F0, -1.0f * MAX_F0, MAX_F0); // 限幅

	VMC_calc_2 (vmcr); // 计算期望的关节输出力矩

	// 额定扭矩
	mySaturate (&vmcr -> torque_set[1], -1.0f * MAX_TORQUE_DM4310, MAX_TORQUE_DM4310);
	mySaturate (&vmcr -> torque_set[0], -1.0f * MAX_TORQUE_DM4310, MAX_TORQUE_DM4310);
}

void mySaturate (float *in, float min, float max)
{
	if (*in < min) {
		*in = min;
	}
	else if (*in > max) {
		*in = max;
	}
}
