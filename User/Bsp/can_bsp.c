#include "can_bsp.h"
#include "fdcan.h"
#include "dm4310_drv.h"
#include "string.h"
#include "chassisR_task.h"


void FDCAN1_Config (void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* Configure Rx filter */
	sFilterConfig . IdType = FDCAN_STANDARD_ID;//��չID������
	sFilterConfig . FilterIndex = 0;
	sFilterConfig . FilterType = FDCAN_FILTER_MASK;
	sFilterConfig . FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig . FilterID1 = 0x00000000; //
	sFilterConfig . FilterID2 = 0x00000000; //
	if (HAL_FDCAN_ConfigFilter (&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler ();
	}

/* ȫ�ֹ������� */
/* ���յ���ϢID���׼ID���˲�ƥ�䣬������ */
/* ���յ���ϢID����չID���˲�ƥ�䣬������ */
/* ���˱�׼IDԶ��֡ */
/* ������չIDԶ��֡ */
	if (HAL_FDCAN_ConfigGlobalFilter (&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
	    HAL_OK) {
		Error_Handler ();
	}

	/* ����RX FIFO0���������ж� */
	if (HAL_FDCAN_ActivateNotification (&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler ();
	}

	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);//����FDCAN�ķ����ӳٲ���

    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1,14,14);//���ò���ʱ�� ����2�Ͳ���3��ΪTimeSeg1��ֵ


	/* Start the FDCAN module */
	if (HAL_FDCAN_Start (&hfdcan1) != HAL_OK) {
		Error_Handler ();
	}


}


void FDCAN3_Config (void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* Configure Rx filter */
	sFilterConfig . IdType = FDCAN_STANDARD_ID;//��չID������
	sFilterConfig . FilterIndex = 3;
	sFilterConfig . FilterType = FDCAN_FILTER_MASK;
	sFilterConfig . FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig . FilterID1 = 0x00000000; //
	sFilterConfig . FilterID2 = 0x00000000; //
	if (HAL_FDCAN_ConfigFilter (&hfdcan3, &sFilterConfig) != HAL_OK) {
		Error_Handler ();
	}

/* ȫ�ֹ������� */
/* ���յ���ϢID���׼ID���˲�ƥ�䣬������ */
/* ���յ���ϢID����չID���˲�ƥ�䣬������ */
/* ���˱�׼IDԶ��֡ */
/* ������չIDԶ��֡ */
	if (HAL_FDCAN_ConfigGlobalFilter (&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
	    HAL_OK) {
		Error_Handler ();
	}

	/* ����RX FIFO0���������ж� */
	if (HAL_FDCAN_ActivateNotification (&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler ();
	}


	/* Start the FDCAN module */
	if (HAL_FDCAN_Start (&hfdcan3) != HAL_OK) {
		Error_Handler ();
	}
}


/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan��FDCAN���
* @param:       id��CAN�豸ID
* @param:       data�����͵�����
* @param:       len�����͵����ݳ���     ������������ʹ��len << 16����ֲ��ʹ�ó����⣡��
* @retval:     	void
* @details:    	��������
************************************************************************
**/
uint8_t canx_send_data (FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader . Identifier = id;
	TxHeader . IdType = FDCAN_STANDARD_ID;                                                                // ��׼ID
	TxHeader . TxFrameType = FDCAN_DATA_FRAME;                                                        // ����֡
	TxHeader . DataLength = len;        //len << 16			������������					// �������ݳ���
	TxHeader . ErrorStateIndicator = FDCAN_ESI_ACTIVE;                                        // ���ô���״ָ̬ʾ
	TxHeader . BitRateSwitch = FDCAN_BRS_OFF;                                                            // �������ɱ䲨����
	TxHeader . FDFormat = FDCAN_CLASSIC_CAN;                                                            // ��ͨCAN��ʽ
	TxHeader . TxEventFifoControl = FDCAN_NO_TX_EVENTS;                                        // ���ڷ����¼�FIFO����, ���洢
	TxHeader . MessageMarker = 0x00;            // ���ڸ��Ƶ�TX EVENT FIFO����ϢMaker��ʶ����Ϣ״̬����Χ0��0xFF

	if (HAL_FDCAN_AddMessageToTxFifoQ (hfdcan, &TxHeader, data) != HAL_OK) {
		return 1;
	}//����
	return 0;
}

extern chassis_t chassis_move;

void HAL_FDCAN_RxFifo0Callback (FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	static FDCAN_RxHeaderTypeDef RxHeader1;
	static uint8_t g_Can1RxData[64];
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		/* Retrieve Rx messages from RX FIFO0 */
		memset (g_Can1RxData, 0, sizeof (g_Can1RxData));    //����ǰ���������
		HAL_FDCAN_GetRxMessage (hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
		if (hfdcan -> Instance == FDCAN3) {
			switch (RxHeader1 . Identifier) {
				case 0x201:dji3508_fbdata (&chassis_move . wheel_motor[0], g_Can1RxData, RxHeader1 . DataLength);
					break;
				case 0x202:dji3508_fbdata (&chassis_move . wheel_motor[1], g_Can1RxData, RxHeader1 . DataLength);
					break;
				default: break;
			}
		}
	}

}

void HAL_FDCAN_RxFifo1Callback (FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	static FDCAN_RxHeaderTypeDef RxHeader2;
	static uint8_t g_Can2RxData[64];
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
		if (hfdcan -> Instance == FDCAN1) {
			memset (g_Can2RxData, 0, sizeof (g_Can2RxData));
			HAL_FDCAN_GetRxMessage (hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);

			switch (RxHeader2 . Identifier) {
				case 0x10 :dm4310_fbdata (&chassis_move . joint_motor[0], g_Can2RxData, RxHeader2 . DataLength);
					break;
				case 0x11 :dm4310_fbdata (&chassis_move . joint_motor[1], g_Can2RxData, RxHeader2 . DataLength);
					break;
				case 0x12:dm4310_fbdata (&chassis_move . joint_motor[2], g_Can2RxData, RxHeader2 . DataLength);
					break;
				case 0x13:dm4310_fbdata (&chassis_move . joint_motor[3], g_Can2RxData, RxHeader2 . DataLength);
					break;
				case 0x14:dm4310_fbdata (&chassis_move . joint_motor[4], g_Can2RxData, RxHeader2 . DataLength);
					break;
				case 0x15:dm4310_fbdata (&chassis_move . joint_motor[5], g_Can2RxData, RxHeader2 . DataLength);
					break;
				default: break;
			}
		}
	}
}
