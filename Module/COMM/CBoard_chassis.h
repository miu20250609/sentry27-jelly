#ifndef STANDARD_ROBOT_C_CBOARD_CHASSIS_H
#define STANDARD_ROBOT_C_CBOARD_CHASSIS_H
#include "user_lib.h"
#include "can/bsp_can.h"
#include "DBUS/dbus.h"

typedef enum
{
    CMD_ID_REFEREE_TRAN = 0x601,
    CMD_ID_RC_GET = DBUS_CANID, // 由 DBUS 模块接收云台转发的遥控数据
    CMD_ID_IMU_GET = 0x603,
} cboard_chassis_cmd_id_t;

typedef struct
{
    CAN_Instance_t* can_instance;
    fp32 imu_angle[3];
    fp32 imu_initial_yaw;
    volatile uint8_t imu_ready;
    uint8_t init;
} cboard_chassis_t;

void CBoard_Chassis_Init(CAN_HandleTypeDef* can_handle);
void CBoard_Referee_Tranmit(uint8_t game_start, uint8_t camp, uint8_t attitude, uint16_t shoot_heat, uint16_t bullet_allow);
cboard_chassis_t *Get_CBoard_Chassis_Ptr(void);
#endif //STANDARD_ROBOT_C_CBOARD_CHASSIS_H
