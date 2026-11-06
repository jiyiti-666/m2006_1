#include "m2006.h"
#include <stdio.h>

static M2006_Bus *s_bus[M2006_MAX_BUS];
static uint8_t     s_bus_cnt = 0;

/* ---------- 内部：按句柄找对应总线 ---------- */
static M2006_Bus *find_bus(CAN_HandleTypeDef *hcan)
{
    for (uint8_t i = 0; i < s_bus_cnt; i++)
        if (s_bus[i]->hcan == hcan) return s_bus[i];
    return NULL;
}

/* ---------- 内部：组帧发送 ---------- */
static void send_frame(M2006_Bus *bus, uint32_t std_id, const int16_t *c)
{
    uint8_t d[8];
    for (uint8_t i = 0; i < 4; i++) {
        d[i * 2]     = (uint8_t)((uint16_t)c[i] >> 8);
        d[i * 2 + 1] = (uint8_t)((uint16_t)c[i] & 0xFF);
    }

    CAN_TxHeaderTypeDef h = {0};
    h.StdId = std_id;
    h.IDE   = CAN_ID_STD;
    h.RTR   = CAN_RTR_DATA;
    h.DLC   = 8;

    if (HAL_CAN_GetTxMailboxesFreeLevel(bus->hcan) == 0) {
        bus->drop++;
        return;
    }
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(bus->hcan, &h, d, &mailbox);
}

/* ---------- 初始化 ---------- */
void M2006_Init(M2006_Bus *bus, CAN_HandleTypeDef *hcan, uint8_t motor_num)
{
    bus->hcan = hcan;
    bus->num  = (motor_num > M2006_MAX_MOTORS) ? M2006_MAX_MOTORS : motor_num;
    bus->drop = 0;

    for (uint8_t i = 0; i < M2006_MAX_MOTORS; i++) {
        bus->m[i].angle = 0; bus->m[i].total_cnt = 0; bus->m[i].angle_deg = 0.0f;
        bus->m[i].speed = 0; bus->m[i].torque    = 0; bus->m[i].temp      = 0;
        bus->m[i].tick  = 0; bus->m[i].cmd       = 0; bus->m[i].online    = 0;
        bus->last_angle[i] = 0;
    }

    CAN_FilterTypeDef f = {0};
    f.FilterBank = 0;

#if defined(STM32F4xx) || defined(STM32F2xx)
    /* F4/F2 的 CAN2 是 CAN1 的从属，只能用 bank 14 起的滤波器 */
    if (hcan->Instance == CAN2) {
        f.FilterBank           = 14;
        f.SlaveStartFilterBank = 14;
    }
#endif

    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;

    HAL_CAN_ConfigFilter(hcan, &f);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* 注册到总线表，供中断回调路由 */
    for (uint8_t i = 0; i < s_bus_cnt; i++)
        if (s_bus[i] == bus) return;
    if (s_bus_cnt < M2006_MAX_BUS) s_bus[s_bus_cnt++] = bus;
}

/* ---------- 设置电流 ---------- */
void M2006_SetCurrent(M2006_Bus *bus, uint8_t id, int16_t cur)
{
    if (id < 1 || id > bus->num) return;
    if (cur >  M2006_CMD_MAX) cur =  M2006_CMD_MAX;
    if (cur < -M2006_CMD_MAX) cur = -M2006_CMD_MAX;
    bus->m[id - 1].cmd = cur;
}

/* ---------- 周期发送 ---------- */
void M2006_Flush(M2006_Bus *bus)
{
    if (bus->num == 0) return;

    int16_t c[4];
    for (uint8_t i = 0; i < 4; i++)
        c[i] = (i < bus->num) ? bus->m[i].cmd : 0;
    send_frame(bus, 0x200, c);

    if (bus->num > 4) {
        for (uint8_t i = 0; i < 4; i++)
            c[i] = ((uint8_t)(i + 4) < bus->num) ? bus->m[i + 4].cmd : 0;
        send_frame(bus, 0x1FF, c);
    }

    for (uint8_t i = 0; i < bus->num; i++)
        bus->m[i].online = (HAL_GetTick() - bus->m[i].tick) < M2006_OFFLINE_MS;
}

/* ---------- 接收解析（全局唯一回调，内部路由） ---------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    M2006_Bus *bus = find_bus(hcan);
    if (!bus) return;

    CAN_RxHeaderTypeDef r;
    uint8_t d[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &r, d) != HAL_OK) break;
        if (r.StdId < 0x201 || r.StdId > 0x208) continue;

        uint8_t      idx = (uint8_t)(r.StdId - 0x201);
        M2006_Motor *m   = &bus->m[idx];

        m->angle  = (uint16_t)((d[0] << 8) | d[1]);
        m->speed  = (int16_t) ((d[2] << 8) | d[3]);
        m->torque = (int16_t) ((d[4] << 8) | d[5]);
        m->temp   = d[6];

        int32_t diff = (int32_t)m->angle - (int32_t)bus->last_angle[idx];
        if (diff >  4096) diff -= 8192;
        if (diff < -4096) diff += 8192;
        m->total_cnt += diff;
        bus->last_angle[idx] = m->angle;
        m->angle_deg = (float)m->total_cnt * 360.0f / 8192.0f;

        m->tick = HAL_GetTick();
    }
}

/* ---------- 查询 ---------- */
M2006_Motor *M2006_Get(M2006_Bus *bus, uint8_t id)
{
    if (id < 1 || id > M2006_MAX_MOTORS) return NULL;
    return &bus->m[id - 1];
}

uint32_t M2006_GetTxDrop(M2006_Bus *bus) { return bus->drop; }

/* ---------- 自检 ---------- */
void M2006_SelfTest(void)
{
    int16_t t[4] = {1000, -1000, 300, 0};
    uint8_t d[8];
    for (uint8_t i = 0; i < 4; i++) {
        d[i*2]   = (uint8_t)((uint16_t)t[i] >> 8);
        d[i*2+1] = (uint8_t)(t[i] & 0xFF);
    }
    printf("%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           d[0],d[1],d[2],d[3],d[4],d[5],d[6],d[7]);
    /* 必须输出：03 E8 FC 18 01 2C 00 00 */
}
