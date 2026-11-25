#ifndef STANDARD_ROBOT_C_NUC_H
#define STANDARD_ROBOT_C_NUC_H
#include <stdbool.h>
#include "usart/bsp_uart.h"
#include "can/bsp_can.h"
#include "typedef.h"

typedef enum
{
    CMD_ID_SELF_DECISION = 0x401,
    CMD_ID_CHASSIS_CTRL = 0x402,
    CMD_ID_BIG_GIMBAL_CTRL = 0x403,
    CMD_ID_REFEREE_FEEDBACK = 0x501,
} nuc_cmd_id_t;

typedef struct
{
    fp32 vx, vy, vw;
    uint8_t chassis_spin, use_cap;
} nuc_chassis_cmd_t;

typedef struct
{
    UART_Instance_t nuc_uart;
    CAN_Instance_t* referee_can;
    QueueHandle_t referee_queue;
    uint32_t referee_drop_count;
    fp32 vx, vy, vw;
    fp32 gimbal_yaw;//这里的是大yaw，雷达只管大yaw电机
    uint32_t gimbal_yaw_sequence;
    uint8_t gimbal_yaw_received;
    uint8_t chassis_spin;
    uint8_t use_cap;
    uint32_t self_decision;
    uint8_t init;
} nuc_ctrl_t;

void NUC_Init(UART_HandleTypeDef *nuc_uart);
CAN_Status_t NUC_Init_CAN(CAN_HandleTypeDef* nuc_can);
CAN_Status_t NUC_InitRefereeCAN(CAN_HandleTypeDef* referee_can);
// 仅有未超时的 0x402 时写入同一帧的底盘指令。
bool NUC_GetChassisCommand(nuc_chassis_cmd_t* command);
bool NUC_GetGimbalCommand(fp32* yaw, uint32_t* sequence);
nuc_ctrl_t *Get_NUC_Ctrl_Instance(void);
void NUC_Referee_Tran(const uint8_t* data, uint16_t len);
void NUC_Referee_Process(void);

#endif //STANDARD_ROBOT_C_NUC_H