#include "gimbal.h"
#include "usart.h"

const pid_config yaw_pid_config = {.mode = PID_POSITION, .kp = 12.0f, .ki = 0.0f, .kd = 0.0f, .max_out = GIMBAL_ROTATE_SPEED_MAX, .max_iout = 0.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config pitch_pid_config = {.mode = PID_POSITION, .kp = 13.0f, .ki = 0.0f, .kd = 0.0f, .max_out = GIMBAL_ROTATE_SPEED_MAX, .max_iout = 0.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
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
static void gimbal_send_rotate_cmd(const gimbal_t* gimbal_ptr);
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

    gimbal_ptr->angle_motor = Get_GM6020_Ptr(GM6020_TX_2);
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
    PID_init(&gimbal_ptr->ctrl.angle_ctrl[0].pid, yaw_pid_config);
    PID_init(&gimbal_ptr->ctrl.angle_ctrl[1].pid, pitch_pid_config);
    PID_init(&gimbal_ptr->ctrl.angle_ctrl[0].speed_pid, gimbal_speed_pid_config);
    PID_init(&gimbal_ptr->ctrl.angle_ctrl[1].speed_pid, gimbal_speed_pid_config);
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
    if (gimbal_ptr->ctrl.gimbal_controller == GIMBAL_RC)
    {
        if (accept_target)
        {
            gimbal_ptr->ctrl.target_yaw -= (fp32)gimbal_ptr->rc->rc_data.ch0 * SMALL_GIMBAL_ANGLE_DELTA_MAX / RC_VAL_MAX;
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
            gimbal_ptr->ctrl.target_pitch = gimbal_ptr->nx_ctrl->target_pitch;
        }
        gimbal_ptr->ctrl.fire = gimbal_ptr->nx_ctrl->fire;
        gimbal_ptr->ctrl.open_friction = (gimbal_ptr->rc->rc_data.s2 == 2) ? 1 : 0; //测试完改为等于fire
    }
    else
    {
        gimbal_ptr->ctrl.fire = 0;
        gimbal_ptr->ctrl.open_friction = 0;
    }
    if (gimbal_ptr->ctrl.feedback == GIMBAL_FEEDBACK_HI12)
    {
        // 用机械相对角约束本次可达的姿态增量，大 yaw 转动时限位随之改变。
        fp32 yaw_delta = theta_format(gimbal_ptr->ctrl.target_yaw - gimbal_ptr->angle.yaw_deg);
        fp32 pitch_delta = gimbal_ptr->ctrl.target_pitch - gimbal_ptr->angle.pitch_deg;
        float_constrain(&yaw_delta, -YAW_LIMIT - gimbal_ptr->ctrl.encoder_yaw,
                                   YAW_LIMIT - gimbal_ptr->ctrl.encoder_yaw);
        float_constrain(&pitch_delta, -PITCH_LIMIT - gimbal_ptr->ctrl.encoder_pitch,
                                     PITCH_LIMIT - gimbal_ptr->ctrl.encoder_pitch);
        gimbal_ptr->ctrl.target_yaw = theta_format(gimbal_ptr->angle.yaw_deg + yaw_delta);
        gimbal_ptr->ctrl.target_pitch = gimbal_ptr->angle.pitch_deg + pitch_delta;
    }
    else
    {
        float_constrain(&gimbal_ptr->ctrl.target_yaw, -YAW_LIMIT, YAW_LIMIT);
        float_constrain(&gimbal_ptr->ctrl.target_pitch, -PITCH_LIMIT, PITCH_LIMIT);
    }
}

static void gimbal_calc_current_angle(gimbal_t* gimbal_ptr)
{
    gimbal_ctrl_t* ctrl = &gimbal_ptr->ctrl;
    ctrl->encoder_pitch = theta_format((fp32)gimbal_ptr->angle_motor->ecd[0].ecd * 360.0f / 8192.0f - ECD_PITCH_OFFSET);
    ctrl->encoder_yaw = theta_format((fp32)gimbal_ptr->angle_motor->ecd[1].ecd * 360.0f / 8192.0f - ECD_YAW_OFFSET);

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
        for (uint8_t axis = 0; axis < 2; axis++)
        {
            PID_clear(&ctrl->angle_ctrl[axis].pid);
            PID_clear(&ctrl->angle_ctrl[axis].speed_pid);
        }
    }
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
    const fp32 feedback[2] = {gimbal_ptr->angle.pitch_deg, gimbal_ptr->angle.yaw_deg};
    const fp32 target[2] = {gimbal_ptr->ctrl.target_pitch, gimbal_ptr->ctrl.target_yaw};
    const fp32 encoder[2] = {gimbal_ptr->ctrl.encoder_pitch, gimbal_ptr->ctrl.encoder_yaw};
    const fp32 limit[2] = {PITCH_LIMIT, YAW_LIMIT};
    for (uint8_t axis = 0; axis < 2; axis++)
    {
        angle_ctrl_t* motor = &gimbal_ptr->ctrl.angle_ctrl[axis];
        const fp32 error = axis == 1 ? theta_format(target[axis] - feedback[axis]) : target[axis] - feedback[axis];
        // 归一化后的角度差供外环使用，速度内环仍采用电机转速。
        PID_calc(&motor->pid, 0.0f, error);
        motor->given_speed = (int16_t)motor->pid.out[0];
        if ((encoder[axis] >= limit[axis] && motor->given_speed > 0) ||
            (encoder[axis] <= -limit[axis] && motor->given_speed < 0))
        {
            motor->given_speed = 0;
            PID_clear(&motor->pid);
        }
        PID_calc(&motor->speed_pid, gimbal_ptr->angle_motor->ecd[axis].speed, motor->given_speed);
        motor->out = motor->speed_pid.out[0];
        if ((encoder[axis] >= limit[axis] && motor->out > 0.0f) ||
            (encoder[axis] <= -limit[axis] && motor->out < 0.0f))
        {
            motor->out = 0.0f;
            PID_clear(&motor->speed_pid);
        }
    }
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
static void gimbal_send_rotate_cmd(const gimbal_t* gimbal_ptr)
{
    int16_t gm6020_v[4];

    gm6020_v[0] = (int16_t)gimbal_ptr->ctrl.angle_ctrl[0].out;
    gm6020_v[1] = (int16_t)gimbal_ptr->ctrl.angle_ctrl[1].out;
    gm6020_v[2] = 0; gm6020_v[3] = 0;

    gm6020_ctrl_voltage(gimbal_ptr->angle_motor, gm6020_v);
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
        (fp32)gimbal_ptr->angle_motor->ecd[0].speed,
        (fp32)gimbal_ptr->angle_motor->ecd[1].speed,
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
