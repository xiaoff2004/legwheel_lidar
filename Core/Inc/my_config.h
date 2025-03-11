#ifndef MY_CONFIG_H
#define MY_CONFIG_H

#include "main.h"
#include "vofa.h"


#define PS2_TIME 10//ps2手柄任务周期是10ms
#define CHASSR_TIME  1
#define OBSERVE_TIME 3//任务周期是3ms

#define  LEG_INIT 0.2f
#define  LEG_MAX 0.38f
#define  LEG_MIN 0.14f


#define PARA \
{-26.02, -2.26, -5.63, -6.02, 9.17, 0.94, 17.13, 1.29, 3.73, 3.78, 23.10, 1.21}


//{-22.27, -1.88, -4.47, -5.14, 7.80, 0.86, 7.28, 0.26, 0.04, -0.04, 17.41, 1.41}
//{-15.31, -1.37, -4.19, -3.67, 4.51, 0.28, 17.72, 2.20, 8.56, 7.20, 66.25, 1.71}

//        {-21.00, -1.49, -3.06, -3.73, 4.44, 0.28, 8.65, 0.68, 1.81, 2.16, 37.44, 1.35}
//{-20.54, -2.22, -3.01, -3.65, 5.39, 0.30, 8.63, 1.22, 1.80, 2.15, 30.06, 1.19}
//{-18.22, -1.38, -3.03, -3.38, 4.30, 0.28, 11.29, 1.01, 2.90, 3.15, 44.89, 1.57}
//{-24.84, -6.06, -1.90, -3.24, 6.43, 0.31, 30.18, 9.53, 3.03, 5.13, 26.92, 1.10}
//{-18.22, -1.38, -3.03, -3.38, 4.30, 0.28, 11.29, 1.01, 2.90, 3.15, 44.89, 1.57}
//{-18.34, -1.39, -3.06, -3.41, 3.79, 0.27, 7.03, 0.63, 1.81, 1.96, 32.07, 1.28}
//{-23.91, -2.89, -4.43, -6.09, 2.38, 0.27, 2.32, 0.34, 0.62, 0.82, 17.16, 1.28}


#define REDUCTION_RATIO (268.0f / 17.0f) //减速比
#define MG 8.0f //质量*重力加速度*高度
#define MG_WHEEL (0.594f*9.8f) //轮子质量*重力加速度
#define WHEEL_R (0.2f/2) //轮子半径

#define MAX_F0 100.0f //最大前馈力
#define MAX_TORQUE_DM4310 20.0f //最大扭矩
#define MAX_TORQUE_DJI3508 5.8f //最大扭矩

#define PITCH_MAX 0.26f

#define pi 3.1415926f

//保持腿长
#define LEG_PID_KP  100.0f
#define LEG_PID_KI  0.0f//不积分
#define LEG_PID_KD  15.0f
#define LEG_PID_MAX_OUT  50.0f //90牛
#define LEG_PID_MAX_IOUT 0.0f


//保持机体水平
#define ROLL_PID_KP 100.0f
#define ROLL_PID_KI 0.0f //不用积分项
#define ROLL_PID_KD 5.0f
#define ROLL_PID_MAX_OUT  15.0f
#define ROLL_PID_MAX_IOUT 0.0f
#define ROLL_TARGET 0.0302489772f

//防劈腿
#define TP_PID_KP 10.0f
#define TP_PID_KI 0.0f //不用积分项
#define TP_PID_KD 0.1f
#define TP_PID_MAX_OUT  3.0f
#define TP_PID_MAX_IOUT 0.0f

//转向环yaw
#define TURN_PID_KP 2.0f
#define TURN_PID_KI 0.0f //不用积分项
#define TURN_PID_KD 1.5f
#define TURN_PID_MAX_OUT  4.0f//轮毂电机的额定扭矩
#define TURN_PID_MAX_IOUT 0.0f




#define DATA_VIEW_SEND_LEN 15
#endif //MY_CONFIG_H
