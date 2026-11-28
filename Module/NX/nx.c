#include "nx.h"
#include <string.h>

#define NX_SCAN_TIMEOUT_MS 200U

static nx_ctrl_t nx_instance;

void NX_CtrlCallback(const uint8_t *data, uint32_t id, void* arg);

void NX_Init(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || nx_instance.init)
        return;
    memset(&nx_instance, 0, sizeof(nx_ctrl_t));
    nx_instance.nx_can = BSP_CAN_Init(hcan);
    if (nx_instance.nx_can == NULL ||
        BSP_CAN_RegisterStdCallback(nx_instance.nx_can, CMD_ID_GIMBAL_CTRL, NX_CtrlCallback, &nx_instance) != CAN_OK)
        return;
    nx_instance.init = 1;
}

void NX_CtrlCallback(const uint8_t *data, uint32_t id, void *arg)
{
    if (data == NULL || arg == NULL || id != CMD_ID_GIMBAL_CTRL)
        return;
    nx_ctrl_t *nx_ctrl = (nx_ctrl_t *)arg;
    if (!nx_ctrl->init)
        return;
    nx_ctrl->fire = data[0] & 0x01;
    nx_ctrl->scan = (data[0] >> 1) & 0x01;
    const int16_t yaw = (int16_t)(data[1] | (data[2] << 8));
    const int16_t pitch = (int16_t)(data[3] | (data[4] << 8));
    nx_ctrl->target_yaw = (float)yaw / 100.0f;   // 转换为度
    nx_ctrl->target_pitch = (float)pitch / 100.0f; // 转换为度
    nx_ctrl->update_time = HAL_GetTick();
    nx_ctrl->data_ready = 1;
}

bool NX_GetScanRequest(const nx_ctrl_t* nx_ctrl, bool* scan)
{
    if (nx_ctrl == NULL || scan == NULL)
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool fresh = nx_ctrl->init && nx_ctrl->data_ready &&
                       (uint32_t)(HAL_GetTick() - nx_ctrl->update_time) < NX_SCAN_TIMEOUT_MS;
    *scan = fresh && nx_ctrl->scan;
    __set_PRIMASK(primask);
    return fresh;
}

bool NX_SendFoldStatus(uint8_t status)
{
    if (!nx_instance.init) return false;
    CAN_HandleTypeDef* can = nx_instance.nx_can->handle;
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = CMD_ID_FOLD_STATUS;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 1U;
    uint8_t data[8] = {status};
    uint32_t mailbox;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool sent = HAL_CAN_GetTxMailboxesFreeLevel(can) != 0U &&
                      HAL_CAN_AddTxMessage(can, &header, data, &mailbox) == HAL_OK;
    __set_PRIMASK(primask);
    return sent;
}

void NX_SendAngle(const fp32 q[4])
{
    if (!nx_instance.init) return;
    int16_t q_int[4];
    uint8_t data[8];
    for (int i = 0; i < 4; i++)
    {
        q_int[i] = (int16_t)(q[i] * 10000.0f);
        data[2*i] = q_int[i] & 0xFF;
        data[2*i+1] = (q_int[i] >> 8) & 0xFF;
    }
    BSP_CAN_Transmit(nx_instance.nx_can, CMD_ID_GIMBAL_ANGLE, CAN_ID_STD, data, 8);
}

nx_ctrl_t *Get_NX_Ctrl_Instance(void)
{
    return &nx_instance;
}
