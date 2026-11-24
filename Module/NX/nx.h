#ifndef STANDARD_ROBOT_C_NX_H
#define STANDARD_ROBOT_C_NX_H
#include "can/bsp_can.h"
#include "typedef.h"
#include <stdbool.h>

typedef enum
{
    CMD_ID_GIMBAL_CTRL = 0x404,
    CMD_ID_GIMBAL_ANGLE = 0x504,
} nx_cmd_id_t;

typedef struct
{
    CAN_Instance_t* nx_can;
    uint8_t fire;
    uint8_t scan;
    fp32 target_yaw;
    fp32 target_pitch;
    uint32_t update_time;
    uint8_t data_ready;
    uint8_t init;
} nx_ctrl_t;

void NX_Init(CAN_HandleTypeDef *hcan);
void NX_SendAngle(const fp32 q[4]);
bool NX_GetScanRequest(const nx_ctrl_t* nx_ctrl, bool* scan);
nx_ctrl_t *Get_NX_Ctrl_Instance(void);

#endif //STANDARD_ROBOT_C_NX_H