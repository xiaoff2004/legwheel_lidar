#ifndef __PSTWO_TASK_H
#define __PSTWO_TASK_H

#include "main.h"
#include "chassisR_task.h"
#include "ins_task.h"

#define BUFF_SIZE    25
#define BUFF2_SIZE 52
#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define REMOTE_RC_OFFSET 1024
#define REMOTE_TOGGLE_DUAL_VAL 1024
#define REMOTE_TOGGLE_THRE_VAL_A 600
#define REMOTE_TOGGLE_THRE_VAL_B 1400
#define DEAD_AREA    120

extern uint8_t rx_buff[BUFF_SIZE * 2];
extern uint8_t rx2_buff[BUFF2_SIZE * 2];

typedef enum
{
	RC_SW_UP,
	RC_SW_MID,
	RC_SW_DOWN
}toggle_t;



typedef enum
{
	RC_CH_LY ,
	RC_CH_LX,
	RC_CH_RY,
	RC_CH_RX
} rc_ch_t;

typedef struct
{
	uint16_t online;
	uint32_t sbus_recever_time;
	struct
	{
		/*
		 *
		 * ch[0] : ◊Û“°∏À…œœ¬ 3535-1024-1695
		 * ch[1] : ◊Û“°∏À◊Û”“
		 * ch[2] : ”““°∏À…œœ¬
		 * ch[3] : ”““°∏À◊Û”“
		 * */
		int16_t ch[10];
	} rc;
	struct
	{
		uint8_t SA;
		uint8_t SB;
		uint8_t SC;
		uint8_t SD;
		uint8_t SF;
		uint8_t SG;
	} toggle;
} remoter_t;

extern remoter_t remoter;


extern void pstwo_task(void);
void sbus_frame_parse (remoter_t *remoter, uint8_t *buf);

	



#endif



