#ifndef __M2006_CTRL_H__
#define __M2006_CTRL_H__

#include "main.h"
#include "m2006.h"

/* ===== 电机配置：按实际情况修改 ===== */
#define M2006_ID_A           1          /* 电机 A 的 ID */
#define M2006_ID_B           3          /* 电机 B 的 ID */

/* 方向系数：反装的那台填 -1，正装填 1。
   如果发现某台电机越跑越远（正反馈飞车），把它的符号反过来即可 */
#define M2006_SIGN_A         ( 1)
#define M2006_SIGN_B         (-1)

/* ===== 机械与周期参数 ===== */
/* 减速比：转子 36 圈 = 输出轴 1 圈 */
#define M2006_REDUCTION      36.0f

/* speed 命令的输出轴 rpm 上限（安全限速），防止误发大值飞车 */
#define M2006_SPEED_LIMIT    80.0f

/* 每圈计数：8192（编码器一圈）× 36（减速比）= 294912，即 OUTPUT 轴转一圈的计数 */
#define M2006_COUNT_PER_REV  294912.0f
#define CTRL_DIV             5          /* 分频：定时器 2ms × 5 = 10ms 控制周期 */

/* 位置到位死区（count）。3000 ≈ 输出轴 3.7°，太小会来回抖，太大定位不准 */
#define LOC_DEADBAND         3000.0f

/* 同步补偿：两台进度不一致时互相拉一把。
   0 = 各自独立跑；0.2~0.4 温和；太大可能相互拉扯震荡 */
#define SYNC_GAIN            0.2f
#define SYNC_MAX             150.0f     /* 补偿量上限 rpm */

typedef enum
{
    CTRL_MODE_OPEN  = 0,   /* 开环：直接给电流 */
    CTRL_MODE_SPEED = 1,   /* 速度闭环 */
    CTRL_MODE_LOC   = 2    /* 位置闭环（位置环串速度环） */
} CtrlMode;

/* 外部可读写的目标值 */
extern volatile int16_t  g_open_current;   /* 开环电流 -10000~10000 */
extern volatile int16_t  g_speed_target;   /* 速度目标 rpm（转子） */
extern volatile int32_t  g_loc_target;     /* 位置目标 count（共同目标） */

/* 差速时各自独立的目标。默认等于 g_loc_target */
extern volatile int32_t  g_loc_target_a;
extern volatile int32_t  g_loc_target_b;

void     Ctrl_SetMode(CtrlMode m);   /* 底盘库切模式用 */

void     Ctrl_Init(M2006_Bus *bus, uint8_t id_a, uint8_t id_b);
void     Ctrl_Task(void);          /* 定时器里按 CTRL_DIV 调用 */
void     Ctrl_Process(void);       /* 主循环调用：解析串口命令 */
void     Ctrl_SendData(void);      /* 主循环调用：50ms 发一次 VOFA */
CtrlMode Ctrl_GetMode(void);

#endif
