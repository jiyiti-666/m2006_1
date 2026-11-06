#ifndef __CAN_DIAG_H
#define __CAN_DIAG_H

#include "main.h"

void CAN_Diag_Init(void);
void CAN_Diag_Send(int16_t cur);
void CAN_Diag_Print(void);

#endif
