#ifndef __M2006_CTRL_H__
#define __M2006_CTRL_H__

#include "main.h"
#include "m2006.h"

/* ===== 需要按实际情况修改的三处 ===== */
#define M2006_CTRL_ID        3          /* 电机 ID */
#define M2006_COUNT_PER_REV  8192.0f    /* 每圈计数：实测输出轴转一圈的角度累加量 */
#define CTRL_DIV             5          /* 分频：定时器 2ms × 5 = 10ms 控制周期 */

/* 位置到位死区（count）。太大定位不准，太小会来回抖 */
#define LOC_DEADBAND         30.0f

typedef enum
{
    CTRL_MODE_OPEN  = 0,   /* 开环：直接给电流 */
    CTRL_MODE_SPEED = 1,   /* 速度闭环 */
    CTRL_MODE_LOC   = 2    /* 位置闭环（位置环串速度环） */
} CtrlMode;

/* 外部可读写的目标值 */
extern volatile int16_t  g_open_current;   /* 开环电流 -10000~10000 */
extern volatile int16_t  g_speed_target;   /* 速度目标 rpm */
extern volatile int32_t  g_loc_target;     /* 位置目标 count */

void     Ctrl_Init(M2006_Bus *bus, uint8_t id);
void     Ctrl_Task(void);          /* 定时器里按 CTRL_DIV 调用 */
void     Ctrl_Process(void);       /* 主循环调用：解析串口命令 */
void     Ctrl_SendData(void);      /* 主循环调用：50ms 发一次 VOFA */
CtrlMode Ctrl_GetMode(void);

#endif
