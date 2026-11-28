#include "gimbal.h"
#include "usart.h"
#include "can.h"
#include <math.h>

const pid_config yaw_pid_config = {.mode = PID_POSITION, .kp = 13.0f, .ki = 0.0f, .kd = 0.0f, .max_out = GIMBAL_ROTATE_SPEED_MAX, .max_iout = 0.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
// 与电调量程一致；初始化和云台控制共用此配置。
const j4310_config_t gimbal_small_pitch_motor_config = {
    .hcan = &hcan1,
    .motor_id = 0x06U, .master_id = 0x12U,
    .mode = J4310_MODE_VEL,
    .p_max = 12.5f, .v_max = 30.0f, .t_max = 10.0f
};

// 输入为度，输出为 rad/s；增益、零位和方向待装车标定。
const pid_config pitch_pid_config = {
    .mode = PID_POSITION, .kp = 0.05f, .ki = 0.0f, .kd = 0.0f,
    .max_out = 1.0f, .max_iout = 0.0f,
    .out_limit_delta_P = 1.0f, .out_limit_delta_N = 1.0f, .deadzone = 0.0f
};
static const struct {
    fp32 motor_sign;
    fp32 encoder_sign;
    fp32 encoder_zero_rad;
    uint32_t feedback_timeout_ms;
    uint32_t service_period_ms;
} small_pitch_control_config = {
    .motor_sign = 1.0f,
    .encoder_sign = 1.0f,
    .encoder_zero_rad = 0.0f,
    .feedback_timeout_ms = 200U,
    .service_period_ms = 100U
};

const pid_config gimbal_speed_pid_config = {.mode = PID_POSITION, .kp = 100.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 25000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config friction_pid_config = {.mode = PID_POSITION, .kp = 3.0f, .ki = 0.0f, .kd = 1.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config trigger_pid_config = {.mode = PID_POSITION, .kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};

gimbal_t gimbal;

// ---------- 内部静态函数声明 ----------
static void gimbal_pid_init(gimbal_t* gimbal_ptr);
static uint8_t gimbal_check_game_start(const gimbal_t* gimbal_ptr) __attribute__((unused));
static void gimbal_switch_controller(gimbal_t* gimbal_ptr);
static void gimbal_calc_target_angle(gimbal_t* gimbal_ptr);
static void gimbal_calc_current_angle(gimbal_t* gimbal_ptr);
static void gimbal_fire_confirm(gimbal_t* gimbal_ptr);
static void gimbal_calc_rotate_pidout(gimbal_t* gimbal_ptr);
static void gimbal_calc_friction_pidout(gimbal_t* gimbal_ptr);
static void gimbal_calc_trigger_pidout(gimbal_t* gimbal_ptr);
static void gimbal_send_friction_cmd(const gimbal_t* gimbal_ptr);
static void gimbal_send_trigger_cmd(const gimbal_t* gimbal_ptr);
static void gimbal_send_rotate_cmd(gimbal_t* gimbal_ptr);
static void gimbal_update_pitch_feedback(gimbal_t* gimbal_ptr);
static void gimbal_calc_pitch_pidout(gimbal_t* gimbal_ptr);
static void gimbal_send_pitch_cmd(gimbal_t* gimbal_ptr);
static void gimbal_send_rc(const gimbal_t* gimbal_ptr);
static void gimbal_send_angle(const gimbal_t* gimbal_ptr);
#if GIMBAL_VOFA_PORT != VOFA_PORT_OFF
static void gimbal_debug(const gimbal_t* gimbal_ptr);
#endif

void Gimbal_Init(gimbal_t* gimbal_ptr)
{
    memset(gimbal_ptr, 0, sizeof(gimbal_t));

    gimbal_ptr->cboard = Get_CBoard_Gimbal_Ptr();
    gimbal_ptr->nx_ctrl = Get_NX_Ctrl_Instance();
    gimbal_ptr->rc = Get_DBUS_Instance();
    gimbal_ptr->hi12 = Get_HI12_Instance();
    gimbal_ptr->im900 = Get_IM900_Instance();

    gimbal_ptr->yaw_motor = Get_GM6020_Ptr(GM6020_TX_2);
    gimbal_ptr->pitch_motor = Get_J4310_Ptr(gimbal_small_pitch_motor_config.hcan,
        gimbal_small_pitch_motor_config.motor_id);
    gimbal_ptr->ctrl.pitch_ctrl.last_service_time = HAL_GetTick() - small_pitch_control_config.service_period_ms;
    gimbal_ptr->trigger_motor = Get_M2006_Ptr(M2006_TX_2);
    gimbal_ptr->friction_motor = Get_M3508_Ptr(M3508_TX_1);

#if GIMBAL_VOFA_PORT != VOFA_PORT_OFF
    gimbal_ptr->vofa = Get_Vofa_Ptr();
#endif

    gimbal_pid_init(gimbal_ptr);
}

void Gimbal_Task(const void* argument)
{
    Gimbal_Init(&gimbal);
    static uint16_t ctrl_loop = 0;
    while(1)
    {
        osDelay(1);
        // if (!gimbal_check_game_start(&gimbal)) continue;
        gimbal_switch_controller(&gimbal);
        if (ctrl_loop % 3 == 0) // 控制循环 300Hz左右
        {
            gimbal_calc_current_angle(&gimbal);
            gimbal_calc_target_angle(&gimbal);
            gimbal_calc_rotate_pidout(&gimbal);
            gimbal_send_rotate_cmd(&gimbal);
        }
        else if (ctrl_loop % 3 == 1)
        {
            gimbal_calc_friction_pidout(&gimbal);
            gimbal_send_friction_cmd(&gimbal);
        }
        else
        {
            gimbal_fire_confirm(&gimbal);
            gimbal_calc_trigger_pidout(&gimbal);
            gimbal_send_trigger_cmd(&gimbal);
        }

        if (ctrl_loop % 5 == 0) // 通信循环 200Hz左右
            gimbal_send_rc(&gimbal);
        else if (ctrl_loop % 5 == 1)
            gimbal_send_angle(&gimbal);
#if GIMBAL_VOFA_PORT != VOFA_PORT_OFF
        else if (ctrl_loop % 5 == 2)
            gimbal_debug(&gimbal);
#endif

        ctrl_loop++;
        if (ctrl_loop > 3000) ctrl_loop = 0;
    }
}

static void gimbal_pid_init(gimbal_t* gimbal_ptr)
{
    PID_init(&gimbal_ptr->ctrl.yaw_ctrl.pid, yaw_pid_config);
    PID_init(&gimbal_ptr->ctrl.yaw_ctrl.speed_pid, gimbal_speed_pid_config);
    PID_init(&gimbal_ptr->ctrl.pitch_ctrl.pid, pitch_pid_config);
    PID_init(&gimbal_ptr->ctrl.friction_ctrl[0].pid, friction_pid_config);
    PID_init(&gimbal_ptr->ctrl.friction_ctrl[1].pid, friction_pid_config);
    PID_init(&gimbal_ptr->ctrl.trigger_ctrl.pid, trigger_pid_config);
}

static uint8_t gimbal_check_game_start(const gimbal_t* gimbal_ptr)
{
    return gimbal_ptr->cboard->game_start;
}

static void gimbal_switch_controller(gimbal_t* gimbal_ptr)
{
    gimbal_ptr->ctrl.gimbal_controller = gimbal_ptr->rc->rc_data.s1;
}

static void gimbal_calc_target_angle(gimbal_t* gimbal_ptr)
{
    const bool accept_target = gimbal_ptr->ctrl.feedback == GIMBAL_FEEDBACK_HI12 &&
                               !gimbal_ptr->ctrl.feedback_changed;
    const small_pitch_ctrl_t* pitch = &gimbal_ptr->ctrl.pitch_ctrl;
    const bool accept_pitch = accept_target && pitch->feedback_valid &&
                              pitch->control_active && !pitch->feedback_changed;
    if (!pitch->control_active || pitch->feedback_changed)
        gimbal_ptr->ctrl.target_pitch = gimbal_ptr->angle.pitch_deg;
    if (gimbal_ptr->ctrl.gimbal_controller == GIMBAL_RC)
    {
        if (accept_target)
        {
            gimbal_ptr->ctrl.target_yaw -= (fp32)gimbal_ptr->rc->rc_data.ch0 * SMALL_GIMBAL_ANGLE_DELTA_MAX / RC_VAL_MAX;
            if (accept_pitch)
                gimbal_ptr->ctrl.target_pitch += (fp32)gimbal_ptr->rc->rc_data.ch1 * SMALL_GIMBAL_ANGLE_DELTA_MAX / RC_VAL_MAX;
        }
        gimbal_ptr->ctrl.fire = gimbal_ptr->rc->rc_data.roll > (RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET) / 2 ? 1 : 0;
        gimbal_ptr->ctrl.open_friction = (gimbal_ptr->rc->rc_data.s2 == 2) ? 1 : 0;
    }
    else if (gimbal_ptr->ctrl.gimbal_controller == GIMBAL_UPC)
    {
        if (accept_target)
        {
            gimbal_ptr->ctrl.target_yaw = gimbal_ptr->nx_ctrl->target_yaw;
            if (accept_pitch)
                gimbal_ptr->ctrl.target_pitch = gimbal_ptr->nx_ctrl->target_pitch;
        }
        gimbal_ptr->ctrl.fire = gimbal_ptr->nx_ctrl->fire;
        gimbal_ptr->ctrl.open_friction = (gimbal_ptr->rc->rc_data.s2 == 2) ? 1 : 0; //测试完改为等于fire
    }
    else
    {
        gimbal_ptr->ctrl.fire = 0;
        gimbal_ptr->ctrl.open_friction = 0;
        gimbal_ptr->ctrl.target_pitch = gimbal_ptr->angle.pitch_deg;
    }
    if (gimbal_ptr->ctrl.feedback == GIMBAL_FEEDBACK_HI12)
    {
        // 用机械相对角约束本次可达的姿态增量，大 yaw 转动时限位随之改变。
        fp32 yaw_delta = theta_format(gimbal_ptr->ctrl.target_yaw - gimbal_ptr->angle.yaw_deg);
        fp32 pitch_delta = gimbal_ptr->ctrl.target_pitch - gimbal_ptr->angle.pitch_deg;
        float_constrain(&yaw_delta, -YAW_LIMIT - gimbal_ptr->ctrl.encoder_yaw,
                                   YAW_LIMIT - gimbal_ptr->ctrl.encoder_yaw);
        if (accept_pitch)
            float_constrain(&pitch_delta, -PITCH_LIMIT - gimbal_ptr->ctrl.encoder_pitch,
                                         PITCH_LIMIT - gimbal_ptr->ctrl.encoder_pitch);
        else
            pitch_delta = 0.0f;
        gimbal_ptr->ctrl.target_yaw = theta_format(gimbal_ptr->angle.yaw_deg + yaw_delta);
        gimbal_ptr->ctrl.target_pitch = gimbal_ptr->angle.pitch_deg + pitch_delta;
    }
    else
    {
        float_constrain(&gimbal_ptr->ctrl.target_yaw, -YAW_LIMIT, YAW_LIMIT);
        // 编码器保持目标在切换时锁存，失去 HI12 时不按姿态限位重写目标。
    }
}

static void gimbal_calc_current_angle(gimbal_t* gimbal_ptr)
{
    gimbal_ctrl_t* ctrl = &gimbal_ptr->ctrl;
    gimbal_update_pitch_feedback(gimbal_ptr);
    ctrl->encoder_yaw = theta_format((fp32)gimbal_ptr->yaw_motor->ecd[1].ecd * 360.0f / 8192.0f - ECD_YAW_OFFSET);

    SensorDataPacket sample;
    uint32_t update_time;
    const bool fresh = HI12_GetData(&sample, &update_time) &&
                       (uint32_t)(HAL_GetTick() - update_time) < HI12_TIMEOUT_MS;
    const Gimbal_Feedback feedback = fresh ? GIMBAL_FEEDBACK_HI12 : GIMBAL_FEEDBACK_ENCODER_HOLD;
    if (fresh)
    {
        gimbal_ptr->angle.yaw_deg = theta_format(HI12_YAW_SIGN * sample.yaw - HI12_YAW_OFFSET);
        gimbal_ptr->angle.pitch_deg = HI12_PITCH_SIGN * sample.pitch - HI12_PITCH_OFFSET;
    }
    else
    {
        gimbal_ptr->angle.yaw_deg = ctrl->encoder_yaw;
        gimbal_ptr->angle.pitch_deg = ctrl->encoder_pitch;
    }

    ctrl->feedback_changed = feedback != ctrl->feedback;
    if (ctrl->feedback_changed)
    {
        ctrl->feedback = feedback;
        ctrl->target_yaw = gimbal_ptr->angle.yaw_deg;
        ctrl->target_pitch = gimbal_ptr->angle.pitch_deg;
        PID_clear(&ctrl->yaw_ctrl.pid);
        PID_clear(&ctrl->yaw_ctrl.speed_pid);
        PID_clear(&ctrl->pitch_ctrl.pid);
    }
    if (ctrl->pitch_ctrl.feedback_changed)
    {
        ctrl->target_pitch = gimbal_ptr->angle.pitch_deg;
        PID_clear(&ctrl->pitch_ctrl.pid);
    }
}


static void gimbal_update_pitch_feedback(gimbal_t* gimbal_ptr)
{
    small_pitch_ctrl_t* pitch = &gimbal_ptr->ctrl.pitch_ctrl;
    const bool was_valid = pitch->feedback_valid;
    pitch->feedback_valid = j4310_get_measure(gimbal_ptr->pitch_motor, &pitch->measure) &&
        (uint32_t)(HAL_GetTick() - pitch->measure.update_time) < small_pitch_control_config.feedback_timeout_ms &&
        pitch->measure.status <= 1U && isfinite(pitch->measure.position) &&
        isfinite(pitch->measure.velocity) && isfinite(pitch->measure.torque);
    pitch->feedback_changed = was_valid != pitch->feedback_valid;
    if (!pitch->feedback_valid)
    {
        pitch->encoder_ready = 0U;
        pitch->out = 0.0f;
        PID_clear(&pitch->pid);
        return;
    }
    if (!pitch->encoder_ready)
    {
        gimbal_ptr->ctrl.encoder_pitch = theta_format(small_pitch_control_config.encoder_sign *
            (pitch->measure.position - small_pitch_control_config.encoder_zero_rad) * 57.295779513f);
        pitch->encoder_ready = 1U;
    }
    else
    {
        fp32 delta = pitch->measure.position - pitch->last_position;
        const fp32 p_max = gimbal_small_pitch_motor_config.p_max;
        if (delta > p_max)
            delta -= 2.0f * p_max;
        else if (delta < -p_max)
            delta += 2.0f * p_max;
        gimbal_ptr->ctrl.encoder_pitch = theta_format(gimbal_ptr->ctrl.encoder_pitch +
            small_pitch_control_config.encoder_sign * delta * 57.295779513f);
    }
    pitch->last_position = pitch->measure.position;
}

static void gimbal_calc_pitch_pidout(gimbal_t* gimbal_ptr)
{
    small_pitch_ctrl_t* pitch = &gimbal_ptr->ctrl.pitch_ctrl;
    pitch->out = 0.0f;
    if (!pitch->feedback_valid || !pitch->encoder_ready || !pitch->control_active ||
        pitch->measure.status != 1U ||
        (gimbal_ptr->ctrl.gimbal_controller != GIMBAL_RC &&
         gimbal_ptr->ctrl.gimbal_controller != GIMBAL_UPC))
    {
        PID_clear(&pitch->pid);
        return;
    }
    PID_calc(&pitch->pid, gimbal_ptr->angle.pitch_deg, gimbal_ptr->ctrl.target_pitch);
    pitch->out = pitch->pid.out[0];
    if (isfinite(pitch->out))
        pitch->out = fmaxf(-pitch_pid_config.max_out, fminf(pitch_pid_config.max_out, pitch->out));
    if ((gimbal_ptr->ctrl.encoder_pitch >= PITCH_LIMIT && pitch->out > 0.0f) ||
        (gimbal_ptr->ctrl.encoder_pitch <= -PITCH_LIMIT && pitch->out < 0.0f))
    {
        pitch->out = 0.0f;
        PID_clear(&pitch->pid);
    }
}

static void gimbal_send_pitch_cmd(gimbal_t* gimbal_ptr)
{
    small_pitch_ctrl_t* pitch = &gimbal_ptr->ctrl.pitch_ctrl;
    if (gimbal_ptr->pitch_motor == NULL)
    {
        pitch->control_active = 0U;
        pitch->out = 0.0f;
        pitch->tx_status = J4310_ERROR_NOT_INIT;
        PID_clear(&pitch->pid);
        return;
    }
    const uint32_t now = HAL_GetTick();
    const bool service_due = (uint32_t)(now - pitch->last_service_time) >= small_pitch_control_config.service_period_ms;
    const bool allowed = pitch->feedback_valid && pitch->encoder_ready &&
        (uint32_t)(now - pitch->measure.update_time) < small_pitch_control_config.feedback_timeout_ms &&
        (gimbal_ptr->ctrl.gimbal_controller == GIMBAL_RC || gimbal_ptr->ctrl.gimbal_controller == GIMBAL_UPC) &&
        isfinite(pitch->out);
    if (!allowed)
    {
        if (pitch->control_active || service_due)
        {
            pitch->last_service_time = now;
            pitch->tx_status = j4310_ctrl_vel(gimbal_ptr->pitch_motor, 0.0f);
            if (pitch->tx_status == J4310_OK)
                pitch->tx_status = j4310_disable(gimbal_ptr->pitch_motor);
            if (pitch->tx_status == J4310_OK)
                pitch->tx_status = j4310_request_measure(gimbal_ptr->pitch_motor);
        }
        pitch->control_active = 0U;
        pitch->out = 0.0f;
        PID_clear(&pitch->pid);
        return;
    }
    if (pitch->measure.status == 0U)
    {
        pitch->control_active = 0U;
        pitch->out = 0.0f;
        gimbal_ptr->ctrl.target_pitch = gimbal_ptr->angle.pitch_deg;
        PID_clear(&pitch->pid);
        if (service_due)
        {
            pitch->last_service_time = now;
            pitch->tx_status = j4310_ctrl_vel(gimbal_ptr->pitch_motor, 0.0f);
            if (pitch->tx_status == J4310_OK)
                pitch->tx_status = j4310_enable(gimbal_ptr->pitch_motor);
            if (pitch->tx_status == J4310_OK)
                pitch->tx_status = j4310_ctrl_vel(gimbal_ptr->pitch_motor, 0.0f);
        }
        return;
    }
    // 首次进入或恢复先零速并对齐目标，下一周期再接收新目标。
    const fp32 speed = pitch->control_active ? small_pitch_control_config.motor_sign * pitch->out : 0.0f;
    pitch->tx_status = j4310_ctrl_vel(gimbal_ptr->pitch_motor, speed);
    pitch->control_active = pitch->tx_status == J4310_OK;
    if (!pitch->control_active)
        PID_clear(&pitch->pid);
}

static void gimbal_fire_confirm(gimbal_t* gimbal_ptr)
{
    gimbal_ptr->ctrl.friction_ready = 1;
    if (!(gimbal_ptr->ctrl.open_friction
        && gimbal_ptr->friction_motor->ecd[0].speed > FRICTION_READY_SPEED
        && gimbal_ptr->friction_motor->ecd[1].speed < -FRICTION_READY_SPEED ))
        gimbal_ptr->ctrl.friction_ready = 0;
    // if (!(gimbal_ptr->cboard_gimbal.bullet_allow > 0 && gimbal_ptr->cboard_gimbal.shoot_heat < 140))
    //     gimbal_ptr->ctrl.friction_ready = 0;
}

static void gimbal_calc_rotate_pidout(gimbal_t* gimbal_ptr)
{
    angle_ctrl_t* yaw = &gimbal_ptr->ctrl.yaw_ctrl;
    const fp32 encoder = gimbal_ptr->ctrl.encoder_yaw;
    const fp32 error = theta_format(gimbal_ptr->ctrl.target_yaw - gimbal_ptr->angle.yaw_deg);
    PID_calc(&yaw->pid, 0.0f, error);
    yaw->given_speed = (int16_t)yaw->pid.out[0];
    if ((encoder >= YAW_LIMIT && yaw->given_speed > 0) ||
        (encoder <= -YAW_LIMIT && yaw->given_speed < 0))
    {
        yaw->given_speed = 0;
        PID_clear(&yaw->pid);
    }
    PID_calc(&yaw->speed_pid, gimbal_ptr->yaw_motor->ecd[1].speed, yaw->given_speed);
    yaw->out = yaw->speed_pid.out[0];
    if ((encoder >= YAW_LIMIT && yaw->out > 0.0f) ||
        (encoder <= -YAW_LIMIT && yaw->out < 0.0f))
    {
        yaw->out = 0.0f;
        PID_clear(&yaw->speed_pid);
    }
    gimbal_calc_pitch_pidout(gimbal_ptr);
}

static void gimbal_calc_friction_pidout(gimbal_t* gimbal_ptr)
{
    gimbal_ptr->ctrl.friction_ctrl[0].given_speed = (gimbal_ptr->ctrl.open_friction) ? FRICTION_SPEED_MAX : 0;
    gimbal_ptr->ctrl.friction_ctrl[1].given_speed = (gimbal_ptr->ctrl.open_friction) ? -FRICTION_SPEED_MAX : 0;

    PID_calc(&gimbal_ptr->ctrl.friction_ctrl[0].pid, gimbal_ptr->friction_motor->ecd[0].speed, gimbal_ptr->ctrl.friction_ctrl[0].given_speed);
    PID_calc(&gimbal_ptr->ctrl.friction_ctrl[1].pid, gimbal_ptr->friction_motor->ecd[1].speed, gimbal_ptr->ctrl.friction_ctrl[1].given_speed);

    gimbal_ptr->ctrl.friction_ctrl[0].out = gimbal_ptr->ctrl.friction_ctrl[0].pid.out[0];
    gimbal_ptr->ctrl.friction_ctrl[1].out = gimbal_ptr->ctrl.friction_ctrl[1].pid.out[0];
}
static void gimbal_calc_trigger_pidout(gimbal_t* gimbal_ptr)
{
    gimbal_ptr->ctrl.trigger_ctrl.given_speed = (gimbal_ptr->ctrl.fire && gimbal_ptr->ctrl.friction_ready) ? TRIGGER_SPEED_MAX : 0;
    PID_calc(&gimbal_ptr->ctrl.trigger_ctrl.pid, gimbal_ptr->trigger_motor->ecd[0].speed, gimbal_ptr->ctrl.trigger_ctrl.given_speed);
    gimbal_ptr->ctrl.trigger_ctrl.out = gimbal_ptr->ctrl.trigger_ctrl.pid.out[0];
}

static void gimbal_send_friction_cmd(const gimbal_t* gimbal_ptr)
{
    int16_t m3508_iq[4];
    m3508_iq[0] = (int16_t)gimbal_ptr->ctrl.friction_ctrl[0].out;
    m3508_iq[1] = (int16_t)gimbal_ptr->ctrl.friction_ctrl[1].out;
    m3508_iq[2] = 0; m3508_iq[3] = 0;

    m3508_ctrl(gimbal_ptr->friction_motor, m3508_iq);
}
static void gimbal_send_rotate_cmd(gimbal_t* gimbal_ptr)
{
    // 小 yaw 保留原 0x2FF 第二槽；旧小 pitch 槽不再输出电压。
    int16_t gm6020_v[4] = {0, (int16_t)gimbal_ptr->ctrl.yaw_ctrl.out, 0, 0};
    gm6020_ctrl_voltage(gimbal_ptr->yaw_motor, gm6020_v);
    gimbal_send_pitch_cmd(gimbal_ptr);
}

static void gimbal_send_trigger_cmd(const gimbal_t* gimbal_ptr)
{
    int16_t m2006_iq[4];

    m2006_iq[0] = (int16_t)gimbal_ptr->ctrl.trigger_ctrl.out;
    m2006_iq[1] = 0; m2006_iq[2] = 0; m2006_iq[3] = 0;

    m2006_ctrl(gimbal_ptr->trigger_motor, m2006_iq);
}

static void gimbal_send_rc(const gimbal_t* gimbal_ptr)
{
    RemoteData_UART2CAN();//遥控器的接收机只接在云台c板上，需要把遥控器数据转发给底盘c板
}
static void gimbal_send_angle(const gimbal_t* gimbal_ptr)
{
#if GIMBAL_VOFA_PORT != VOFA_PORT_UART6
    im900_data_t data;
    if (IM900_GetData(&data) && (data.data_flags & IM900_DATA_EULER) != 0U)
        CBoard_IMU_Transmit(data.yaw);
#endif

    const fp32 quaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f};//四元数，用于把云台姿态报给nx，目前没有启用
    NX_SendAngle(quaternion);
}
#if GIMBAL_VOFA_PORT != VOFA_PORT_OFF
static void gimbal_debug(const gimbal_t* gimbal_ptr)
{
    // 按此数组顺序输出，可直接替换、增加或减少本板变量。
    const fp32 debug[] = {
        gimbal_ptr->ctrl.target_pitch,
        gimbal_ptr->angle.pitch_deg,
        gimbal_ptr->ctrl.target_yaw,
        gimbal_ptr->angle.yaw_deg,
        gimbal_ptr->ctrl.pitch_ctrl.measure.velocity,
        (fp32)gimbal_ptr->yaw_motor->ecd[1].speed,
        (fp32)gimbal_ptr->friction_motor->ecd[0].speed,
        (fp32)gimbal_ptr->trigger_motor->ecd[0].speed,
    };
    if (!gimbal_ptr->vofa->init)
    {
#if GIMBAL_VOFA_PORT == VOFA_PORT_UART1
        vofa_init(&huart1, sizeof(debug) / sizeof(debug[0]), 0);
#else
        vofa_init(&huart6, sizeof(debug) / sizeof(debug[0]), 0);
#endif
    }
    vofa_print(debug);
}
#endif
