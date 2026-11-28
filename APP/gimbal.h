#ifndef STANDARD_ROBOT_C_GIMBAL_H
#define STANDARD_ROBOT_C_GIMBAL_H
#include "pid.h"
#include "typedef.h"
#include "user_lib.h"
#include "motor/DJI/GM6020/gm6020.h"
#include "motor/DAMIAO/DM4310/j4310.h"
#include "motor/DJI/M2006/m2006.h"
#include "motor/DJI/M3508/m3508.h"
#include "COMM/CBoard_gimbal.h"
#include "NX/nx.h"
#include "DBUS/dbus.h"
#include "IMU/HI12/hi12.h"
#include "IMU/IM900/im900.h"
#include "VOFA/vofa.h"
#include "robot_config.h"

#define SMALL_GIMBAL_ANGLE_DELTA_MAX 0.08f
#define TRIGGER_SPEED_MAX 1000
#define FRICTION_SPEED_MAX 5000
#define FRICTION_READY_SPEED 3000
#define GIMBAL_ROTATE_SPEED_MAX 320
#define YAW_LIMIT 30.0f
#define PITCH_LIMIT 20.0f
#define ECD_YAW_OFFSET 60.0f

#define HI12_TIMEOUT_MS 200U
// 待实车标定：模块姿态转换到瞄准控制坐标，单位度。
#define HI12_YAW_SIGN 1.0f
#define HI12_PITCH_SIGN 1.0f
#define HI12_YAW_OFFSET 0.0f
#define HI12_PITCH_OFFSET 0.0f

typedef enum
{
    GIMBAL_FEEDBACK_UNINITIALIZED = 0,
    GIMBAL_FEEDBACK_HI12,
    GIMBAL_FEEDBACK_ENCODER_HOLD
} Gimbal_Feedback;

typedef enum
{
    GIMBAL_RC_OFFLINE = 0, // 遥控器掉线
    GIMBAL_RC = 3, // 遥控器控制
    GIMBAL_UPC = 2, // 上位机控制
    GIMBAL_SHUTDOWN = 1 // 关机状态
} Gimbal_Controller;

typedef struct
{
    pid_t pid;
    int16_t given_speed;
    fp32 out; 
} friction_ctrl_t;

typedef struct
{
    pid_t pid; 
    int16_t given_speed; 
    fp32 out; 
} trigger_ctrl_t;

// GM6020 电机控制结构体
typedef struct
{
    pid_t pid;
    pid_t speed_pid;
    int16_t given_speed; 
    fp32 out; 
} angle_ctrl_t;

// 角度误差为度，速度输出为 rad/s；状态与电机实例分别保存。
typedef struct
{
    pid_t pid;
    j4310_measure_t measure;
    J4310_Status_t tx_status;
    fp32 out;
    fp32 last_position;
    uint32_t last_service_time;
    uint8_t feedback_valid;
    uint8_t feedback_changed;
    uint8_t encoder_ready;
    uint8_t control_active;
} small_pitch_ctrl_t;

typedef struct
{
    Gimbal_Controller gimbal_controller;
    fp32 target_yaw;
    fp32 target_pitch;
    fp32 encoder_pitch, encoder_yaw;
    Gimbal_Feedback feedback;
    uint8_t feedback_changed;
    uint8_t open_friction;
    uint8_t friction_ready;
    uint8_t fire;

    friction_ctrl_t friction_ctrl[2];
    trigger_ctrl_t trigger_ctrl;
    angle_ctrl_t yaw_ctrl;
    small_pitch_ctrl_t pitch_ctrl;
} gimbal_ctrl_t;

typedef struct
{
    gimbal_ctrl_t ctrl;
    gm6020_instance* yaw_motor;
    j4310_instance* pitch_motor;
    m2006_instance* trigger_motor;
    m3508_instance* friction_motor;

    angle_t angle;

    cboard_gimbal_t* cboard;
    nx_ctrl_t* nx_ctrl;
    rc_instance* rc;
    hi12_t* hi12;
    im900_t* im900;

    vofa_t* vofa;
} gimbal_t;

extern const j4310_config_t gimbal_small_pitch_motor_config;

void Gimbal_Init(gimbal_t* gimbal_ptr);
void Gimbal_Task(const void* argument);

#endif //STANDARD_ROBOT_C_GIMBAL_H
