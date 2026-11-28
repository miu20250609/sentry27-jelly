#include "chassis.h"
#include "can.h"
#include <math.h>
#include "VOFA/vofa.h"
#include "usart.h"
#include <string.h>
#include "robot_config.h"
#include "arm_math.h"
#include "user_lib.h"
#include "pid.h"
#include "dwt/bsp_dwt.h"

// 与电调配置一致；底盘初始化与控制共用此配置。
const j4310_config_t chassis_big_yaw_motor_config = {
    .hcan = &hcan2,
    .motor_id = 0x01U, .master_id = 0x11U,
    .mode = J4310_MODE_VEL,
    .p_max = 12.5f, .v_max = 30.0f, .t_max = 10.0f
};

#if defined(CHASSIS) && CHASSIS_COMM_ONLY
void Chassis_Task(const void* argument)
{
    while (1)
    {
        osDelay(1);
        NUC_Referee_Process();
    }
}
#else

const fp32 filter_speed_alpha[3] = {0.2f, 0.2f, 0.2f};
const pid_config m3508_pid_config = {.mode = PID_POSITION, .kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
// 角度误差用度，输出为 rad/s；增益和安装方向待装车调试。
const pid_config big_yaw_angle_pid_config = {
    .mode = PID_POSITION,
    .kp = 0.05f, .ki = 0.0f, .kd = 0.0f,
    .max_out = 2.0f, .max_iout = 1.0f,
    .out_limit_delta_P = 2.0f, .out_limit_delta_N = 2.0f, .deadzone = 0.0f
};
static const struct {
    fp32 motor_sign;
    fp32 encoder_zero_rad;
    fp32 encoder_sign;
    fp32 imu_sign;
    fp32 feedforward_gain;
    uint32_t feedback_timeout_ms;
    uint32_t service_period_ms;
} big_yaw_control_config = {
    .motor_sign = 1.0f,
    .encoder_zero_rad = 0.0f,
    .encoder_sign = 1.0f,
    .imu_sign = -1.0f,
    .feedforward_gain = 1.0f,
    .feedback_timeout_ms = 200U,
    .service_period_ms = 100U
};
chassis_t chassis;

// ---------- 内部静态函数声明 ----------
#if CHASSIS_VOFA_PORT != VOFA_PORT_OFF
static void chassis_debug(const chassis_t* chassis_ptr);
#endif
static void chassis_filter_init(chassis_t* chassis_ptr);
static void chassis_pid_init(chassis_t* chassis_ptr);
static uint8_t chassis_check_game_start(const chassis_t* chassis_ptr);
static void chassis_switch_controller(chassis_t* chassis_ptr);
static void chassis_calc_move_speed(chassis_t* chassis_ptr, const nuc_chassis_cmd_t* nuc_cmd);
static void chassis_calc_rotate_angle(chassis_t* chassis_ptr, const nuc_chassis_cmd_t* nuc_cmd);
static void chassis_update_big_yaw_feedback(chassis_t* chassis_ptr);
static void chassis_calc_angle(chassis_t* chassis_ptr);
static void chassis_calc_wheelmotor_speed(chassis_t* chassis_ptr, bool nuc_stop);
static void chassis_calc_gimbalmotor_angle(chassis_t* chassis_ptr);
static void chassis_calc_wheelmotor_pidout(chassis_t* chassis_ptr);
static void chassis_calc_gimbalmotor_pidout(chassis_t* chassis_ptr);
static void chassis_set_max_power(chassis_t* chassis_ptr, const nuc_chassis_cmd_t* nuc_cmd);
static void chassis_update_power_param(const chassis_t* chassis_ptr);
static void chassis_limit_power(chassis_t* chassis_ptr);
static void chassis_send_wheelmotor_cmd(const chassis_t* chassis_ptr);
static void chassis_send_gimbalmotor_cmd(chassis_t* chassis_ptr);
static void chassis_send_referee_info(const chassis_t* chassis_ptr);

/**
 * @brief 底盘初始化函数
 *        初始化各类传感器、电机控制、PID等
 */
void Chassis_Init(chassis_t* chassis_ptr)
{
    chassis_ptr->bmi088 = Get_BMI088_Ptr();
    chassis_ptr->m3508 = Get_M3508_Ptr(M3508_TX_1);
    chassis_ptr->cboard_chassis = Get_CBoard_Chassis_Ptr();
    chassis_ptr->ctx = Get_PowerControlParam_Ptr();
    chassis_ptr->big_yaw = Get_J4310_Ptr(chassis_big_yaw_motor_config.hcan, chassis_big_yaw_motor_config.motor_id);
    chassis_ptr->nuc_ctrl = Get_NUC_Ctrl_Instance();
    chassis_ptr->nx_ctrl = Get_NX_Ctrl_Instance();
    chassis_ptr->power_ctrl_config = Get_PowerControllerConfig_Ptr();
    chassis_ptr->power_ctrl_result = Get_PowerAllocationResult_Ptr();
    chassis_ptr->rc = Get_DBUS_Instance();
    chassis_ptr->referee = Get_Referee_Instance();
    chassis_ptr->super_cap = Get_SuperCap_Instance();

    memset(&chassis_ptr->ctrl, 0, sizeof(chassis_ptr->ctrl));
    chassis_filter_init(chassis_ptr);
    chassis_pid_init(chassis_ptr);
    chassis_ptr->ctrl.big_yaw_controller.last_service_time = HAL_GetTick() - big_yaw_control_config.service_period_ms;
}

/**
 * @brief 底盘控制任务，供FreeRTOS调用
 */
void Chassis_Task(const void* argument)
{
    Chassis_Init(&chassis);
    static uint16_t ctrl_loop = 0;
    while(1)
    {
        osDelay(1);
        NUC_Referee_Process();
        nuc_chassis_cmd_t command;
        const nuc_chassis_cmd_t* nuc_cmd = NUC_GetChassisCommand(&command) ? &command : NULL;
        // if (!chassis_check_game_start(&chassis)) continue;
        if (ctrl_loop % 3 == 0) // 控制循环 300Hz左右
        {

            chassis_switch_controller(&chassis);       // 切换控制源（遥控/上位机等）
            chassis_calc_move_speed(&chassis,
                chassis.ctrl.chassis_controller == CHASSIS_UPC && !chassis.ctrl.big_yaw_controller.angle_ready ? NULL : nuc_cmd);
            const bool nuc_stop = chassis.ctrl.chassis_controller == CHASSIS_UPC &&
                (nuc_cmd == NULL || !chassis.ctrl.big_yaw_controller.angle_ready);
            chassis_calc_wheelmotor_speed(&chassis, nuc_stop);
            chassis_calc_wheelmotor_pidout(&chassis);  // 计算电机控制PID输出
            chassis_send_wheelmotor_cmd(&chassis);     // 下发3508控制指令
        }
        else if (ctrl_loop % 3 == 1) // 云台控制循环 300Hz左右，和底盘控制错开，减少CAN总线冲突
        {
            chassis_switch_controller(&chassis);        // 切换控制源（遥控/上位机等）
            chassis_update_big_yaw_feedback(&chassis);
            chassis_calc_angle(&chassis);
            chassis_calc_rotate_angle(&chassis, nuc_cmd);
            chassis_calc_gimbalmotor_angle(&chassis);   // 解算航向云台角度环
            chassis_calc_gimbalmotor_pidout(&chassis);  // 计算云台控制PID输出
            chassis_send_gimbalmotor_cmd(&chassis);     // 下发大 yaw 速度指令
        }
        else
        {
            chassis_set_max_power(&chassis, nuc_cmd);
            //chassis_update_power_param(&chassis);     // 功率控制参数更新 (当前未使用)
            //chassis_limit_power(&chassis);            // 功率分配控制算法 (当前未使用)
        }

        if (ctrl_loop % 100 == 0) // 通信循环 10Hz左右
        {
            chassis_send_referee_info(&chassis); // 发送裁判系统信息到云台
        }

#if CHASSIS_VOFA_PORT != VOFA_PORT_OFF
        if (ctrl_loop % 10 == 0)
            chassis_debug(&chassis);
#endif
        ctrl_loop++;
        if (ctrl_loop > 3000) ctrl_loop = 0;
    }
}

/**
 * @brief 底盘一阶低通滤波初始化
 */
static void chassis_filter_init(chassis_t* chassis_ptr)
{
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[0], CHASSIS_CONTROL_TIME, filter_speed_alpha[0]);
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[1], CHASSIS_CONTROL_TIME, filter_speed_alpha[1]);
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[2], CHASSIS_CONTROL_TIME, filter_speed_alpha[2]);
}

/**
 * @brief 初始化PID参数
 */
static void chassis_pid_init(chassis_t* chassis_ptr)
{
    PID_init(&chassis_ptr->ctrl.big_yaw_controller.pid, big_yaw_angle_pid_config);
    for (int i = 0; i < 4; i++)
        PID_init(&chassis_ptr->ctrl.m3508_controller[i].pid, m3508_pid_config);
}

/**
 * @brief 检查游戏是否开始 底盘由NUC控制时由NUC判断是否开始
 */
static uint8_t chassis_check_game_start(const chassis_t* chassis_ptr)
{
    return (chassis_ptr->referee->game_status.game_progress == 4);
}

/**
 * @brief 切换底盘控制器（通过遥控器S1拨杆）
 */
static void chassis_switch_controller(chassis_t* chassis_ptr)
{
    chassis_ptr->ctrl.chassis_controller = chassis_ptr->rc->rc_data.s1;
}

/**
 * @brief 解算给定移动速度（获取控制源指令并滤波）
 */
static void chassis_calc_move_speed(chassis_t* chassis_ptr, const nuc_chassis_cmd_t* nuc_cmd)
{
    fp32 vx, vy, vw;
    if (chassis_ptr->ctrl.chassis_controller == CHASSIS_RC)//遥控器控制
    {
        chassis_ptr->ctrl.chassis_spin = 0;//注意这个是nuc请求的小陀螺模式标志，不是说不能小陀螺，后面vw即为遥控器控制的小陀螺角速度

        vx = (fp32)chassis_ptr->rc->rc_data.ch3 * CHASSIS_MAX_V / RC_VAL_MAX;
        vy = (fp32)chassis_ptr->rc->rc_data.ch2 * CHASSIS_MAX_V / RC_VAL_MAX;//ch2,3是遥控器左边摇杆的x,y轴
        vw = (fp32)chassis_ptr->rc->rc_data.ch1 * CHASSIS_MAX_V / RC_VAL_MAX;
    }
    else if (chassis_ptr->ctrl.chassis_controller == CHASSIS_UPC && nuc_cmd != NULL)
    {
        chassis_ptr->ctrl.chassis_spin = nuc_cmd->chassis_spin;

        vx = loop_float_constrain(nuc_cmd->vx, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        vy = loop_float_constrain(nuc_cmd->vy, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        vw = loop_float_constrain(nuc_cmd->vw, -CHASSIS_MAX_V, CHASSIS_MAX_V);

        if (chassis_ptr->ctrl.chassis_spin)
            vw = SPIN_VW_SPEED;//码一下，之后是否要以不同的速度小陀螺
    }
    else
    {
        chassis_ptr->ctrl.chassis_spin = 0;

        vx = 0;
        vy = 0;
        vw = 0;
    }

    // 速度滤波
    first_order_filter_cali(&chassis_ptr->ctrl.chassis_speed_filter[0], vx);
    first_order_filter_cali(&chassis_ptr->ctrl.chassis_speed_filter[1], vy);
    first_order_filter_cali(&chassis_ptr->ctrl.chassis_speed_filter[2], vw);
    const fp32 vx_filter = chassis_ptr->ctrl.chassis_speed_filter[0].out;
    const fp32 vy_filter = chassis_ptr->ctrl.chassis_speed_filter[1].out;
    const fp32 vw_filter = chassis_ptr->ctrl.chassis_speed_filter[2].out;

    // 转换为合速度大小与角度（极坐标格式存储进 given_chassis_v）
    fp32 norm_v;
    arm_sqrt_f32(vx_filter * vx_filter + vy_filter * vy_filter, &norm_v);
    chassis_ptr->ctrl.given_chassis_v[0] = norm_v * LINEAR_TO_RPM;//线速度大小
    chassis_ptr->ctrl.given_chassis_v[1] = atan2f(vy_filter, vx_filter);//线速度方向角度
    chassis_ptr->ctrl.given_chassis_w = vw_filter * LINEAR_TO_RPM;// 给定底盘自转角速度
}

/**
 * @brief 解算云台旋转的给定角度
 */
static void chassis_calc_rotate_angle(chassis_t* chassis_ptr, const nuc_chassis_cmd_t* nuc_cmd)
{
    fp32 nuc_yaw = 0.0f;
    uint32_t sequence = 0;
    const bool received = NUC_GetGimbalCommand(&nuc_yaw, &sequence);
    bool scan = false;
    NX_GetScanRequest(chassis_ptr->nx_ctrl, &scan);

    big_yaw_ctrl_t *yaw = &chassis_ptr->ctrl.big_yaw_controller;
    if (!yaw->angle_ready || !yaw->control_active ||
        (chassis_ptr->ctrl.chassis_controller != CHASSIS_RC &&
         chassis_ptr->ctrl.chassis_controller != CHASSIS_UPC))
    {
        chassis_ptr->ctrl.gimbal_shutdown_flag = !yaw->angle_ready ||
            (chassis_ptr->ctrl.chassis_controller != CHASSIS_RC &&
             chassis_ptr->ctrl.chassis_controller != CHASSIS_UPC);
        chassis_ptr->ctrl.gimbal_scan = 0;
        chassis_ptr->ctrl.given_gimbal_l_yaw = yaw->cur_angle;
        if (received)
            chassis_ptr->ctrl.gimbal_yaw_sequence = sequence;
        return;
    }

    if (chassis_ptr->ctrl.chassis_controller == CHASSIS_RC)
    {
        chassis_ptr->ctrl.gimbal_shutdown_flag = 0;
        chassis_ptr->ctrl.gimbal_scan = 0;
        const fp32 yaw_delta = (fp32)chassis_ptr->rc->rc_data.ch0 * GIMBAL_ANGLE_DELTA_MAX / RC_VAL_MAX;
        chassis_ptr->ctrl.given_gimbal_l_yaw += yaw_delta;
    }
    else if (chassis_ptr->ctrl.chassis_controller == CHASSIS_UPC)
    {
        chassis_ptr->ctrl.gimbal_shutdown_flag = 0;
        chassis_ptr->ctrl.gimbal_scan = scan;
        if (scan)
        {
            chassis_ptr->ctrl.given_gimbal_l_yaw += GIMBAL_ANGLE_DELTA_MAX;
            // 扫描期间收到的角度不在退出扫描后重放，上位机负责协调控制权。
            if (received)
                chassis_ptr->ctrl.gimbal_yaw_sequence = sequence;
        }
        else if (nuc_cmd != NULL && received && sequence != chassis_ptr->ctrl.gimbal_yaw_sequence)
        {
            chassis_ptr->ctrl.given_gimbal_l_yaw = nuc_yaw;
            chassis_ptr->ctrl.gimbal_yaw_sequence = sequence;
        }
        return;
    }
    else
    {
        chassis_ptr->ctrl.gimbal_scan = 0;
        chassis_ptr->ctrl.gimbal_shutdown_flag = 1;
    }
    if (received)
        chassis_ptr->ctrl.gimbal_yaw_sequence = sequence;
}

static void chassis_update_big_yaw_feedback(chassis_t* chassis_ptr)
{
    big_yaw_ctrl_t *ctrl = &chassis_ptr->ctrl.big_yaw_controller;
    const bool received = j4310_get_measure(chassis_ptr->big_yaw, &ctrl->measure);
    const uint32_t now = HAL_GetTick();
    ctrl->feedback_valid = received &&
        (uint32_t)(now - ctrl->measure.update_time) < big_yaw_control_config.feedback_timeout_ms &&
        ctrl->measure.status <= 1U && isfinite(ctrl->measure.position) &&
        isfinite(ctrl->measure.velocity) && isfinite(ctrl->measure.torque);
    ctrl->angle_ready = 0U;
    if (!ctrl->feedback_valid)
        return;

    if (!ctrl->encoder_ready)
    {
        ctrl->encoder_yaw_deg = theta_format(big_yaw_control_config.encoder_sign *
            (ctrl->measure.position - big_yaw_control_config.encoder_zero_rad) * 57.295779513f);
        ctrl->encoder_ready = 1U;
    }
    else
    {
        // 处理映射端点回绕；实际固件跨圈行为仍需装车验证。
        fp32 delta = ctrl->measure.position - ctrl->last_position;
        if (delta > chassis_big_yaw_motor_config.p_max)
            delta -= 2.0f * chassis_big_yaw_motor_config.p_max;
        else if (delta < -chassis_big_yaw_motor_config.p_max)
            delta += 2.0f * chassis_big_yaw_motor_config.p_max;
        ctrl->encoder_yaw_deg = theta_format(ctrl->encoder_yaw_deg +
            big_yaw_control_config.encoder_sign * delta * 57.295779513f);
    }
    ctrl->last_position = ctrl->measure.position;
}

static void chassis_calc_angle(chassis_t* chassis_ptr)
{
    big_yaw_ctrl_t *ctrl = &chassis_ptr->ctrl.big_yaw_controller;
    ctrl->angle_ready = 0U;
    if (!ctrl->feedback_valid)
        return;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint8_t imu_ready = chassis_ptr->cboard_chassis->imu_ready;
    fp32 imu_yaw = chassis_ptr->cboard_chassis->imu_angle[0];
    const fp32 initial_yaw = chassis_ptr->cboard_chassis->imu_initial_yaw;
    __set_PRIMASK(primask);
    if (!imu_ready || !isfinite(imu_yaw) || !isfinite(initial_yaw))
        return;

    if (!ctrl->imu_reference_ready)
    {
        ctrl->imu_reference_deg = initial_yaw;
        imu_yaw = initial_yaw;
        ctrl->imu_reference_ready = 1U;
    }
    const fp32 gimbal_angle[3] = {theta_format(imu_yaw - ctrl->imu_reference_deg), 0.0f, 0.0f};
    Angle_Update(&chassis_ptr->gimbal_angle, gimbal_angle);
    ctrl->cur_angle = big_yaw_control_config.imu_sign * chassis_ptr->gimbal_angle.yaw_total_angle;
    const fp32 chassis_angle[3] = {
        theta_format(ctrl->encoder_yaw_deg + gimbal_angle[0]), 0.0f, 0.0f
    };
    Angle_Update(&chassis_ptr->chassis_angle, chassis_angle);
    ctrl->angle_ready = 1U;
}

/**
 * @brief 全向轮底盘运动学解算函数
 * @param vx x向速度 m/s
 * @param vy y向速度 m/s
 * @param vw 自转角速度 rad/s
 * @param spin_yaw 当前Yaw向扭矩偏移
 * @param motor_speeds 输出到4个电机的目标转速
 */
void Omni_Kinematics_Resolve(const fp32 vx, const fp32 vy, const fp32 vw, const fp32 spin_yaw, int16_t* motor_speeds)
{
    //雷达发送的vx vy是世界系下的，spin_yaw是底盘在世界系下的yaw角度，vx vy需要转换为底盘系下的速度再解算
    const fp32 sin_yaw = sinf(radian_format(spin_yaw));
    const fp32 cos_yaw = cosf(radian_format(spin_yaw));
    const fp32 vx_set = vx * cos_yaw - vy * sin_yaw;
    const fp32 vy_set = vx * sin_yaw + vy * cos_yaw;

    motor_speeds[0] = (int16_t)((vx_set + vy_set) / ROOT_2 + vw);
    motor_speeds[1] = (int16_t)((-vx_set + vy_set) / ROOT_2 + vw);
    motor_speeds[2] = (int16_t)((-vx_set - vy_set) / ROOT_2 + vw);
    motor_speeds[3] = (int16_t)((vx_set - vy_set) / ROOT_2 + vw);
}

/**
 * @brief 解算3508电机的目标速度（给四个轮毂下发指令速度）
 */
static void chassis_calc_wheelmotor_speed(chassis_t* chassis_ptr, bool nuc_stop)
{
    const fp32 chassis_v = chassis_ptr->ctrl.given_chassis_v[0];
    const fp32 theta = chassis_ptr->ctrl.given_chassis_v[1];
    // NUC 超时减速时关闭云台跟随项，确保轮速目标最终归零。
    const fp32 chassis_w = chassis_ptr->ctrl.given_chassis_w + (nuc_stop ? 0.0f : chassis_ptr->gimbal_angle.yaw_rad);
    const fp32 vx = chassis_v * cosf(theta);
    const fp32 vy = chassis_v * sinf(theta);
    int16_t motor_speeds[4];
#ifdef OMNI
    Omni_Kinematics_Resolve(vx, vy, chassis_w, chassis_ptr->chassis_angle.yaw_rad, motor_speeds);
#endif

    chassis_ptr->ctrl.m3508_controller[0].given_speed = motor_speeds[0];
    chassis_ptr->ctrl.m3508_controller[1].given_speed = motor_speeds[1];
    chassis_ptr->ctrl.m3508_controller[2].given_speed = motor_speeds[2];
    chassis_ptr->ctrl.m3508_controller[3].given_speed = motor_speeds[3];
}

/**
 * @brief 解算大 yaw 目标及速度前馈
 */
static void chassis_calc_gimbalmotor_angle(chassis_t* chassis_ptr)
{
    BMI088_Read();
    chassis_ptr->ctrl.big_yaw_controller.given_angle = chassis_ptr->ctrl.given_gimbal_l_yaw;
    chassis_ptr->ctrl.big_yaw_controller.ff_speed = big_yaw_control_config.feedforward_gain * chassis_ptr->bmi088->Gyro[2];
}

/**
 * @brief PID控制律计算，包括3508速度环的PID计算和大 yaw 航向角度环的PID计算
 */
static void chassis_calc_wheelmotor_pidout(chassis_t* chassis_ptr)
{
    for (int i = 0; i < 4; i++)
    {
        PID_calc(&chassis_ptr->ctrl.m3508_controller[i].pid, chassis_ptr->m3508->ecd[i].speed, chassis_ptr->ctrl.m3508_controller[i].given_speed);
        chassis_ptr->ctrl.m3508_controller[i].out = chassis_ptr->ctrl.m3508_controller[i].pid.out[0];
    }
}
static void chassis_calc_gimbalmotor_pidout(chassis_t* chassis_ptr)
{
    big_yaw_ctrl_t *ctrl = &chassis_ptr->ctrl.big_yaw_controller;
    ctrl->out = 0.0f;
    if (chassis_ptr->ctrl.gimbal_shutdown_flag || !ctrl->angle_ready ||
        !ctrl->control_active || ctrl->measure.status != 1U)
    {
        PID_clear(&ctrl->pid);
        return;
    }
    PID_calc(&ctrl->pid, ctrl->cur_angle, ctrl->given_angle);
    ctrl->out = ctrl->pid.out[0] + ctrl->ff_speed;
    if (isfinite(ctrl->out))
        ctrl->out = fmaxf(-big_yaw_angle_pid_config.max_out, fminf(big_yaw_angle_pid_config.max_out, ctrl->out));
}

/**
 * @brief 最大功率限制参数设置与超级电容控制模式控制策略
 */
static void chassis_set_max_power(chassis_t* chassis_ptr, const nuc_chassis_cmd_t* nuc_cmd)
{
    chassis_ptr->ctrl.use_cap = nuc_cmd != NULL && nuc_cmd->use_cap;
    switch (chassis_ptr->ctrl.use_cap) // 该策略仅作联盟赛哨兵使用
    {
        case 0: // 常规情况，不使用超电
            chassis_ptr->ctrl.maxpower = sentinelMaxPower[0];
            super_cap_use(0);
            break;
        case 1: // ♿️冲刺♿️，使用超电抢占中心点
            chassis_ptr->ctrl.maxpower = sentinelMaxPower[0] + chassis_ptr->super_cap->power_data.supercap_power;
            super_cap_use(1);
            break;
        default:
            chassis_ptr->ctrl.maxpower = sentinelMaxPower[0];
            super_cap_use(0);
            break;
    }
    if (chassis_ptr->super_cap->power_data.remain_v < REMAINPOWER_MIN) // 超电剩余能量过低时不使用超电
    {
        chassis_ptr->ctrl.maxpower = sentinelMaxPower[0];
        super_cap_use(0);
    }
    setMaxPower(chassis_ptr->ctrl.maxpower);
    limitMaxPower(chassis_ptr->referee->power_heat_data.buffer_energy);
}

/**
 * @brief 将功率预测参数更新
 */
static void chassis_update_power_param(const chassis_t* chassis_ptr)
{
    const float measuredpower = chassis_ptr->super_cap->power_data.total_power;

    float torque_feedback[4];
    float rpm_feedback[4];

    for (int i = 0; i < 4; i++) {
        torque_feedback[i] = (float)chassis_ptr->m3508->ecd[i].current * M3508_CURRENT_LIMIT / M3508_ECD_MAX * TORQUE_TO_CURRENT_FACTOR;
        rpm_feedback[i] = chassis_ptr->m3508->ecd[i].speed;
    }

    float effective_power = 0.0f;
    for (int i = 0; i < 4; i++) {
        const float angular_velocity = rpm_feedback[i] * (PI / 30.0f);
        effective_power += torque_feedback[i] * angular_velocity;
    }

    PowerControl_CollectMotorData(torque_feedback, rpm_feedback, measuredpower, 4);
    PowerControl_Update(effective_power);

    updatePowerControlConfig(chassis_ptr->ctx->k2, chassis_ptr->ctx->k3);
}

/**
 * @brief 利用功率控制器计算各电机的限制输出
 */
static void chassis_limit_power(chassis_t* chassis_ptr)
{
    static MotorPowerObj motor_power[4];

    for (int i = 0; i < 4; i++)
    {
        motor_power[i].curAv = (fp32)chassis_ptr->m3508->ecd[i].speed * ECD_TO_AV;
        motor_power[i].setAv = (fp32)chassis_ptr->ctrl.m3508_controller[i].given_speed * ECD_TO_AV;
        motor_power[i].pidOutput = chassis_ptr->ctrl.m3508_controller[i].pid.out[0];
        motor_power[i].pidMaxOutput = chassis_ptr->ctrl.m3508_controller[i].pid.max_out;
    }

    MotorPowerObj *motors[4] = {&motor_power[0], &motor_power[1], &motor_power[2], &motor_power[3]};
    allocatePowerWithLimit(motors);

    for (int i = 0; i < 4; i++)
    {
        chassis_ptr->ctrl.m3508_controller[i].out = (int16_t)chassis_ptr->power_ctrl_result->newTorqueCurrent[i];
    }
}

/**
 * @brief 将控制计算结果下发到CAN总线上
 */
static void chassis_send_wheelmotor_cmd(const chassis_t* chassis_ptr)
{
    int16_t m3508_iq[4];
    for (int i = 0; i < 4; i++)
        m3508_iq[i] = (int16_t)chassis_ptr->ctrl.m3508_controller[i].out;
    m3508_ctrl(chassis_ptr->m3508, m3508_iq);
}
static void chassis_send_gimbalmotor_cmd(chassis_t* chassis_ptr)
{
    big_yaw_ctrl_t *ctrl = &chassis_ptr->ctrl.big_yaw_controller;
    if (chassis_ptr->big_yaw == NULL)
    {
        ctrl->control_active = 0U;
        ctrl->out = 0.0f;
        ctrl->tx_status = J4310_ERROR_NOT_INIT;
        PID_clear(&ctrl->pid);
        return;
    }
    const uint32_t now = HAL_GetTick();
    const bool service_due = (uint32_t)(now - ctrl->last_service_time) >= big_yaw_control_config.service_period_ms;
    const bool allowed = !chassis_ptr->ctrl.gimbal_shutdown_flag && ctrl->angle_ready &&
        ctrl->feedback_valid &&
        (uint32_t)(now - ctrl->measure.update_time) < big_yaw_control_config.feedback_timeout_ms &&
        isfinite(ctrl->out);

    if (!allowed)
    {
        if (ctrl->control_active || service_due)
        {
            ctrl->last_service_time = now;
            ctrl->tx_status = j4310_ctrl_vel(chassis_ptr->big_yaw, 0.0f);
            if (ctrl->tx_status == J4310_OK)
                ctrl->tx_status = j4310_disable(chassis_ptr->big_yaw);
            if (ctrl->tx_status == J4310_OK)
                ctrl->tx_status = j4310_request_measure(chassis_ptr->big_yaw);
        }
        ctrl->control_active = 0U;
        ctrl->out = 0.0f;
        PID_clear(&ctrl->pid);
        return;
    }

    if (ctrl->measure.status == 0U)
    {
        ctrl->control_active = 0U;
        ctrl->out = 0.0f;
        chassis_ptr->ctrl.given_gimbal_l_yaw = ctrl->cur_angle;
        PID_clear(&ctrl->pid);
        if (service_due)
        {
            ctrl->last_service_time = now;
            ctrl->tx_status = j4310_ctrl_vel(chassis_ptr->big_yaw, 0.0f);
            if (ctrl->tx_status == J4310_OK)
                ctrl->tx_status = j4310_enable(chassis_ptr->big_yaw);
            if (ctrl->tx_status == J4310_OK)
                ctrl->tx_status = j4310_ctrl_vel(chassis_ptr->big_yaw, 0.0f);
        }
        return;
    }

    // 首次进入或恢复控制先发零速，下一周期才采用新的角度指令。
    const fp32 speed = ctrl->control_active ? big_yaw_control_config.motor_sign * ctrl->out : 0.0f;
    ctrl->tx_status = j4310_ctrl_vel(chassis_ptr->big_yaw, speed);
    ctrl->control_active = ctrl->tx_status == J4310_OK;
    if (!ctrl->control_active)
        PID_clear(&ctrl->pid);
}

static void chassis_send_referee_info(const chassis_t* chassis_ptr)
{
    const uint8_t game_start = chassis_ptr->referee->game_status.game_progress == 4 ? 1 : 0;
    const uint8_t camp = (chassis_ptr->referee->robot_performance.robot_id > 100) ? 1 : 0;
    const uint8_t attitude = (chassis_ptr->referee->sentry_info.sentry_info_2 >> 12) & 0x03;
    const uint16_t shoot_heat = chassis_ptr->referee->power_heat_data.shooter_17mm_1_barrel_heat;
    const uint16_t bullet_allow = chassis_ptr->referee->projectile_allowance.projectile_allowance_17mm;
    CBoard_Referee_Tranmit(game_start, camp, attitude, shoot_heat, bullet_allow);
}

#if CHASSIS_VOFA_PORT != VOFA_PORT_OFF
static void chassis_debug(const chassis_t* chassis_ptr)
{
    // 按此数组顺序输出，可直接替换、增加或减少本板变量。
    const fp32 debug[] = {
        (fp32)chassis_ptr->ctrl.m3508_controller[0].given_speed,
        (fp32)chassis_ptr->ctrl.m3508_controller[1].given_speed,
        (fp32)chassis_ptr->ctrl.m3508_controller[2].given_speed,
        (fp32)chassis_ptr->ctrl.m3508_controller[3].given_speed,
        (fp32)chassis_ptr->m3508->ecd[0].speed,
        (fp32)chassis_ptr->m3508->ecd[1].speed,
        (fp32)chassis_ptr->m3508->ecd[2].speed,
        (fp32)chassis_ptr->m3508->ecd[3].speed,
    };
    if (!Get_Vofa_Ptr()->init)
        vofa_init(&huart6, sizeof(debug) / sizeof(debug[0]), 0);
    vofa_print(debug);
}
#endif

#endif
