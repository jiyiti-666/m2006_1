#include "can.h"
#include "can_diag.h"
#include <stdio.h>

static volatile uint32_t rx_total = 0;   /* 收到的总帧数 */
static volatile uint32_t rx_motor = 0;   /* 其中 0x201~0x208 的帧数 */
static volatile uint32_t tx_ok    = 0;
static volatile uint32_t tx_fail  = 0;
static volatile uint32_t last_id  = 0;
static volatile uint8_t  last_raw[8];

void CAN_Diag_Init(void)
{
    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;

    HAL_CAN_ConfigFilter(&hcan, &f);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_Diag_Send(int16_t cur)
{
    uint8_t d[8] = {0};
    d[4] = (uint8_t)((uint16_t)cur >> 8);   /* ID 3 占 DATA[4]/DATA[5] */
    d[5] = (uint8_t)((uint16_t)cur & 0xFF);

    CAN_TxHeaderTypeDef h = {0};
    h.StdId = 0x200;
    h.IDE   = CAN_ID_STD;
    h.RTR   = CAN_RTR_DATA;
    h.DLC   = 8;

    uint32_t box;
    if (HAL_CAN_AddTxMessage(&hcan, &h, d, &box) == HAL_OK) tx_ok++;
    else tx_fail++;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hc)
{
    CAN_RxHeaderTypeDef r;
    uint8_t d[8];

    while (HAL_CAN_GetRxFifoFillLevel(hc, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hc, CAN_RX_FIFO0, &r, d) != HAL_OK) break;

        rx_total++;
        last_id = r.StdId;
        for (int i = 0; i < 8; i++) last_raw[i] = d[i];
        if (r.StdId >= 0x201 && r.StdId <= 0x208) rx_motor++;
    }
}

void CAN_Diag_Print(void)
{
    uint32_t esr = hcan.Instance->ESR;

    printf("[CAN] state=%lu err=0x%08lX\r\n",
           (unsigned long)HAL_CAN_GetState(&hcan),
           (unsigned long)HAL_CAN_GetError(&hcan));
    printf("  TEC=%lu REC=%lu LEC=%lu BOFF=%lu\r\n",
           (unsigned long)((esr >> 16) & 0xFF),
           (unsigned long)((esr >> 24) & 0xFF),
           (unsigned long)((esr >> 4)  & 0x07),
           (unsigned long)((esr >> 2)  & 0x01));
    printf("  tx_ok=%lu tx_fail=%lu  rx_total=%lu rx_motor=%lu\r\n",
           (unsigned long)tx_ok, (unsigned long)tx_fail,
           (unsigned long)rx_total, (unsigned long)rx_motor);

    if (rx_total > 0) {
        printf("  ID=0x%03lX raw=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               (unsigned long)last_id,
               last_raw[0], last_raw[1], last_raw[2], last_raw[3],
               last_raw[4], last_raw[5], last_raw[6], last_raw[7]);
        printf("  angle=%u speed=%d cur=%d\r\n",
               (unsigned)((last_raw[0] << 8) | last_raw[1]),
               (int)(int16_t)((last_raw[2] << 8) | last_raw[3]),
               (int)(int16_t)((last_raw[4] << 8) | last_raw[5]));
    }
}
