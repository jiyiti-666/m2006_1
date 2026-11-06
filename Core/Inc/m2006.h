#ifndef __M2006_H
#define __M2006_H

#include "main.h"

#define M2006_MAX_MOTORS   8
#define M2006_MAX_BUS      2      /* 最多支持几条 CAN 总线 */
#define M2006_CMD_MAX      10000  /* C610 满量程；M3508 改 16384 */
#define M2006_OFFLINE_MS   100

typedef struct {
    uint16_t angle;       /* 单圈机械角度 0~8191 */
    int32_t  total_cnt;   /* 多圈累计计数 */
    float    angle_deg;   /* 累计角度，按 8192/圈 换算 */
    int16_t  speed;       /* rpm */
    int16_t  torque;      /* 实际输出转矩 */
    uint8_t  temp;        /* 部分固件有效 */
    uint32_t tick;
    int16_t  cmd;
    uint8_t  online;
} M2006_Motor;

typedef struct {
    CAN_HandleTypeDef *hcan;                       /* 绑定的 CAN 句柄 */
    M2006_Motor        m[M2006_MAX_MOTORS];
    uint16_t           last_angle[M2006_MAX_MOTORS];
    uint8_t            num;                        /* 本总线电机数量 */
    uint32_t           drop;                       /* 发送失败计数 */
} M2006_Bus;

void         M2006_Init(M2006_Bus *bus, CAN_HandleTypeDef *hcan, uint8_t motor_num);
M2006_Motor *M2006_Get(M2006_Bus *bus, uint8_t id);
void         M2006_SetCurrent(M2006_Bus *bus, uint8_t id, int16_t cur);
void         M2006_Flush(M2006_Bus *bus);
uint32_t     M2006_GetTxDrop(M2006_Bus *bus);
void         M2006_SelfTest(void);

#endif
