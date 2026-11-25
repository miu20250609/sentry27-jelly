#ifndef STANDARD_ROBOT_C_J4310_H
#define STANDARD_ROBOT_C_J4310_H

#include <stdbool.h>
#include "can/bsp_can.h"

#define J4310_CNT_MAX 4U
#define J4310_KP_MAX 500.0f
#define J4310_KD_MAX 5.0f

typedef enum
{
    J4310_MODE_MIT = 0,
    J4310_MODE_POS_VEL,
    J4310_MODE_VEL
} J4310_Mode_t;

typedef enum
{
    J4310_OK = 0,
    J4310_ERROR_INVALID_PARAM,
    J4310_ALREADY_INITIALIZED,
    J4310_ERROR_NO_MEMORY,
    J4310_ERROR_CAN_INIT,
    J4310_ERROR_CAN_REGISTER,
    J4310_ERROR_CAN_TX,
    J4310_ERROR_NOT_INIT,
    J4310_ERROR_INVALID_MODE,
    J4310_ERROR_ID_CONFLICT
} J4310_Status_t;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint16_t motor_id; // 1..15，反馈首字节的高四位用于状态。
    uint16_t master_id; // 同一 CAN 总线各电机使用不同的反馈 ID。
    J4310_Mode_t mode;
    // 必须与调试助手的 P_MAX/V_MAX/T_MAX 一致，单位 rad、rad/s、N·m。
    float p_max;
    float v_max;
    float t_max;
} j4310_config_t;

typedef struct
{
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;
    float position; // rad，保留电机原始坐标和零位。
    float velocity; // rad/s。
    float torque; // N·m。
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    uint8_t status; // 反馈首字节高四位，原样保留状态/故障码。
    uint32_t update_time; // 最近一次有效反馈的 HAL tick，ms。
} j4310_measure_t;

typedef struct
{
    CAN_Instance_t *can_ins;
    uint16_t motor_id;
    uint16_t master_id;
    J4310_Mode_t mode;
    float p_max;
    float v_max;
    float t_max;
    j4310_measure_t ecd;
    uint8_t data_ready;
    uint8_t init;
} j4310_instance;

// 在模块初始化阶段调用；mode 须匹配电机配置，不自动使能或写入参数。
J4310_Status_t j4310_init(const j4310_config_t *config);
j4310_instance *Get_J4310_Ptr(CAN_HandleTypeDef *hcan, uint16_t motor_id);
bool j4310_get_measure(const j4310_instance *ins, j4310_measure_t *measure);

// 返回值只表示 CAN 提交结果，使能状态以反馈 status 为准。
J4310_Status_t j4310_enable(const j4310_instance *ins);
J4310_Status_t j4310_disable(const j4310_instance *ins);
J4310_Status_t j4310_request_measure(const j4310_instance *ins);

// MIT 位置控制（kp > 0）要求 kd > 0。
// p_des/v_des/t_ff 的单位为 rad、rad/s、N·m；MIT 越界或非有限数返回参数错误。
J4310_Status_t j4310_ctrl_mit(const j4310_instance *ins, float p_des, float v_des,
                              float kp, float kd, float t_ff);
// v_limit 是非负的最大绝对速度，单位 rad/s。
J4310_Status_t j4310_ctrl_pos_vel(const j4310_instance *ins, float p_des, float v_limit);
J4310_Status_t j4310_ctrl_vel(const j4310_instance *ins, float v_des);

#endif
