/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     �������ǶԻ����˶��ٶȹ��ƣ��������ƴ�
	* 					 ԭ����Դ��https://zhuanlan.zhihu.com/p/689921165
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "observe_task.h"
#include "kalman_filter.h"
#include "cmsis_os.h"
#include "my_config.h"
#include "math.h"



KalmanFilter_t vaEstimateKF;       // �������˲����ṹ��

float vaEstimateKF_F[4] = {1.0f, 0.003f,
                           0.0f, 1.0f};       // ״̬ת�ƾ��󣬿�������Ϊ0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // �������Э�����ʼֵ

float vaEstimateKF_Q[4] = {0.1f, 0.0f,
                           0.0f, 0.05f};    // QԤ������ʼֵ 0--�ٶ����� 3--���ٶ�����  ����

float vaEstimateKF_R[4] = {20.0f, 0.0f,
                            0.0f,  20.0f};// R�������������ʼֵ 0--�ٶ����� 3--���ٶ�����  �ͷ�

float vaEstimateKF_K[4];

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};    // ���þ���HΪ����

extern INS_t INS;
extern chassis_t chassis_move;

extern vmc_leg_t right;
extern vmc_leg_t left;

float vel_acc[2];

void Observe_task (void)
{
	while (INS . ins_flag == 0) {//�ȴ����ٶ�����
		osDelay (1);
	}
	static float wr, wl = 0.0f;
	static float vrb, vlb = 0.0f;
	static float aver_v = 0.0f;
	static float wheel_v = 0.0f;

	xvEstimateKF_Init (&vaEstimateKF);
	static uint8_t observe_flag = 0;
	float tmp_v = 0.0f;
	static uint32_t OB_DWT = 0;
	static float dt = 0;
	dt = DWT_GetDeltaT (&OB_DWT);
	while (1) {
		wr = -chassis_move . wheel_motor[0] . para . vel *2* pi- INS . Gyro[0] + right . d_alpha;//�ұ�������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
		vrb = wr *  WHEEL_R + right . L0 * right . d_theta * arm_cos_f32 (right . theta) +
		      right . d_L0 * arm_sin_f32 (right . theta);//����bϵ���ٶ�

		wl = -chassis_move . wheel_motor[1] . para . vel * 2*pi + INS . Gyro[0] + left . d_alpha;//���������ת����Դ�ؽ��ٶȣ����ﶨ�����˳ʱ��Ϊ��
		vlb = wl  * WHEEL_R + left . L0 * left . d_theta * arm_cos_f32 (left . theta) +
		      left . d_L0 * arm_sin_f32 (left . theta);//����bϵ���ٶ�

		aver_v = (vrb - vlb) / 2.0f;//ȡƽ��

//		vaEstimateKF.F_data[1] = DWT_GetDeltaT (dwt) ;
		xvEstimateKF_Update (&vaEstimateKF, -INS . MotionAccel_b[1], aver_v);

		wheel_v = (-chassis_move . wheel_motor[0] . para . vel + chassis_move . wheel_motor[1] . para . vel) * WHEEL_R * 2 * pi /
		          2.0f;

		dt = DWT_GetDeltaT (&OB_DWT);
		if (observe_flag == 0) {
			if (fabs (vel_acc[0] - tmp_v)/dt < 0.0001f) {
				observe_flag = 1;
				chassis_move . x_filter = 0.0f;
			}
			else {
				tmp_v = vel_acc[0];
			}
		}
		else if (observe_flag == 1) {

//			if (fabs (wheel_v / vel_acc[0]) > 10.0f) {
//				chassis_move . v_filter = vel_acc[0];//�õ��������˲�����ٶ�
//				//ԭ����ת�Ĺ�����v_filter��x_filterӦ�ö���Ϊ0
//			}
//			else {
//				chassis_move . v_filter = wheel_v;
//			}
//			chassis_move . v_filter = wheel_v;
//			chassis_move . v_filter = vel_acc[0];//�õ��������˲�����ٶ�
//			if (fabs (chassis_move . v_filter) < 0.002f)
//				chassis_move . v_filter = 0.0f;

//			chassis_move . x_filter += chassis_move . v_filter * dt;
		}

//		chassis_move . x_filter += chassis_move . v_filter * dt;

	chassis_move.v_filter = wheel_v;
	chassis_move . x_filter += chassis_move . v_filter * dt;
//	if(chassis_move.v_set < 0.0001f && chassis_move.v_set > -0.0001f)
//if(chassis_move.v_set == 0.0f)
//{
////		if(chassis_move . x_filter ==0.0f)
////			chassis_move . x_filter = -0.305288f;
////		else
//			chassis_move . x_filter += chassis_move . v_filter * dt;
//	}
//
//	else
//		chassis_move . x_filter =0.0f;

//	if(chassis_move.v_filter < 1.0f && chassis_move.v_filter > -1.0f && chassis_move.v_set == 0.0f)
//	{
//		chassis_move . x_filter += chassis_move . v_filter * dt;
//	}
//
//	else
//		chassis_move . x_filter =0.0f;



//	data_view[0]    = dt ;
//	data_view[1]    = chassis_move . v_filter;
//	data_view[2]    = chassis_move . x_filter;
//	data_view[3] =  vel_acc[0];
//	if(fabs(vel_acc[0])>0.001f)
//		data_view[4] +=vel_acc[0] *dt ;
//		data_view[0] = wr;
//		data_view[1] = vrb;
//		data_view[2] = INS . Gyro[0];
//		data_view[3] = right . d_alpha;
//		data_view[4] = -chassis_move . wheel_motor[0] . para . vel;
//		data_view[5] = chassis_move . v_filter;
//		data_view[6] = chassis_move . x_filter;
//		data_view[7] = INS . MotionAccel_b[1];

//		data_view[9] += data_view[8] * ((float) OBSERVE_TIME / 1000.0f);
		//�����ֱ���������ٶȣ������ںϵĻ���������
		//chassis_move.v_filter=(chassis_move.wheel_motor[0].para.vel-chassis_move.wheel_motor[1].para.vel)*(-0.0603f)/2.0f;//0.0603�����Ӱ뾶������������ǽ��ٶȣ��˰뾶��õ����ٶȣ���ѧģ���ж����������˳ʱ��Ϊ��������Ҫ�˸�����
		//chassis_move.x_filter=chassis_move.x_filter+chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		osDelay (OBSERVE_TIME);
	}
}

void xvEstimateKF_Init (KalmanFilter_t *EstimateKF)
{
	Kalman_Filter_Init (EstimateKF, 2, 0, 2);    // ״̬����2ά û�п����� ��������2ά

	memcpy (EstimateKF -> F_data, vaEstimateKF_F, sizeof (vaEstimateKF_F));
	memcpy (EstimateKF -> P_data, vaEstimateKF_P, sizeof (vaEstimateKF_P));
	memcpy (EstimateKF -> Q_data, vaEstimateKF_Q, sizeof (vaEstimateKF_Q));
	memcpy (EstimateKF -> R_data, vaEstimateKF_R, sizeof (vaEstimateKF_R));
	memcpy (EstimateKF -> H_data, vaEstimateKF_H, sizeof (vaEstimateKF_H));

}

void xvEstimateKF_Update (KalmanFilter_t *EstimateKF, float acc, float vel)
{
	//�������˲�������ֵ����
	EstimateKF -> MeasuredVector[0] = vel;//�����ٶ�
	EstimateKF -> MeasuredVector[1] = acc;//�������ٶ�

	//�������˲������º���
	Kalman_Filter_Update (EstimateKF);

	// ��ȡ����ֵ
	for (uint8_t i = 0; i < 2; i++) {
		vel_acc[i] = EstimateKF -> FilteredValue[i];
	}
}


