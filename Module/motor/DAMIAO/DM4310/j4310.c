#include "j4310.h"
#include <math.h>
#include <string.h>

static j4310_instance motors[J4310_CNT_MAX];

static uint16_t j4310_encode_signed(float value, float limit, uint8_t bits)
{
    return (uint16_t)((value / limit + 1.0f) * 0.5f * (float)((1U << bits) - 1U));
}

static float j4310_decode_signed(uint16_t value, float limit, uint8_t bits)
{
    return ((float)value / (float)((1U << bits) - 1U) * 2.0f - 1.0f) * limit;
}

static void j4310_write_float(uint8_t *data, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8);
    data[2] = (uint8_t)(raw >> 16);
    data[3] = (uint8_t)(raw >> 24);
}

static void j4310_callback(const uint8_t *data, uint32_t id, void *arg)
{
    j4310_instance *ins = (j4310_instance *)arg;
    if (data == NULL || ins == NULL || !ins->init || id != ins->master_id ||
        (data[0] & 0x0FU) != ins->motor_id)
        return;

    j4310_measure_t measure = {0};
    measure.position_raw = (uint16_t)((uint16_t)data[1] << 8) | data[2];
    measure.velocity_raw = (uint16_t)((uint16_t)data[3] << 4) | (data[4] >> 4);
    measure.torque_raw = (uint16_t)((uint16_t)(data[4] & 0x0FU) << 8) | data[5];
    measure.position = j4310_decode_signed(measure.position_raw, ins->p_max, 16);
    measure.velocity = j4310_decode_signed(measure.velocity_raw, ins->v_max, 12);
    measure.torque = j4310_decode_signed(measure.torque_raw, ins->t_max, 12);
    measure.mos_temperature = data[6];
    measure.rotor_temperature = data[7];
    measure.status = data[0] >> 4;
    measure.update_time = HAL_GetTick();
    ins->ecd = measure;
    ins->data_ready = 1U;
}

J4310_Status_t j4310_init(const j4310_config_t *config)
{
    if (config == NULL || config->hcan == NULL || config->motor_id == 0U ||
        config->motor_id > 15U || config->master_id > 0x7FFU ||
        (unsigned int)config->mode > J4310_MODE_VEL ||
        !isfinite(config->p_max) || config->p_max <= 0.0f ||
        !isfinite(config->v_max) || config->v_max <= 0.0f ||
        !isfinite(config->t_max) || config->t_max <= 0.0f)
        return J4310_ERROR_INVALID_PARAM;

    j4310_instance *ins = NULL;
    for (uint8_t i = 0; i < J4310_CNT_MAX; i++)
    {
        if (motors[i].init && motors[i].can_ins->handle->Instance == config->hcan->Instance &&
            motors[i].motor_id == config->motor_id)
            return J4310_ALREADY_INITIALIZED;
        if (!motors[i].init && ins == NULL)
            ins = &motors[i];
    }
    if (ins == NULL)
        return J4310_ERROR_NO_MEMORY;

    CAN_Instance_t *can_ins = BSP_CAN_Init(config->hcan);
    if (can_ins == NULL)
        return J4310_ERROR_CAN_INIT;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    // BSP 注册相同 ID 会覆盖回调，这里先拒绝冲突。
    for (uint8_t i = 0; i < MAX_CAN_FILTERS; i++)
    {
        if (can_ins->rx_callbacks[i] != NULL && can_ins->callback_ide[i] == CAN_ID_STD &&
            (can_ins->callback_ids[i] & can_ins->callback_idmasks[i]) ==
            (config->master_id & can_ins->callback_idmasks[i]))
        {
            __set_PRIMASK(primask);
            return J4310_ERROR_ID_CONFLICT;
        }
    }

    memset(ins, 0, sizeof(*ins));
    ins->can_ins = can_ins;
    ins->motor_id = config->motor_id;
    ins->master_id = config->master_id;
    ins->mode = config->mode;
    ins->p_max = config->p_max;
    ins->v_max = config->v_max;
    ins->t_max = config->t_max;
    if (BSP_CAN_RegisterStdDataCallback(can_ins, ins->master_id, 8U, j4310_callback, ins) != CAN_OK)
    {
        memset(ins, 0, sizeof(*ins));
        __set_PRIMASK(primask);
        return J4310_ERROR_CAN_REGISTER;
    }
    ins->init = 1U;
    __set_PRIMASK(primask);
    return J4310_OK;
}

j4310_instance *Get_J4310_Ptr(CAN_HandleTypeDef *hcan, uint16_t motor_id)
{
    if (hcan == NULL)
        return NULL;
    for (uint8_t i = 0; i < J4310_CNT_MAX; i++)
        if (motors[i].init && motors[i].can_ins->handle->Instance == hcan->Instance &&
            motors[i].motor_id == motor_id)
            return &motors[i];
    return NULL;
}

bool j4310_get_measure(const j4310_instance *ins, j4310_measure_t *measure)
{
    if (ins == NULL || measure == NULL || !ins->init)
        return false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool ready = ins->data_ready != 0U;
    if (ready)
        *measure = ins->ecd;
    __set_PRIMASK(primask);
    return ready;
}

static J4310_Status_t j4310_transmit(const j4310_instance *ins, uint16_t offset,
                                     const uint8_t *data, uint8_t size)
{
    if (BSP_CAN_Transmit(ins->can_ins, ins->motor_id + offset, CAN_ID_STD, data, size) != CAN_OK)
        return J4310_ERROR_CAN_TX;
    return J4310_OK;
}

static J4310_Status_t j4310_send_command(const j4310_instance *ins, uint8_t command)
{
    if (ins == NULL || !ins->init || ins->can_ins == NULL)
        return J4310_ERROR_NOT_INIT;
    uint16_t offset;
    switch (ins->mode)
    {
        case J4310_MODE_MIT: offset = 0U; break;
        case J4310_MODE_POS_VEL: offset = 0x100U; break;
        case J4310_MODE_VEL: offset = 0x200U; break;
        default: return J4310_ERROR_INVALID_MODE;
    }
    const uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
    return j4310_transmit(ins, offset, data, 8U);
}

J4310_Status_t j4310_enable(const j4310_instance *ins)
{
    return j4310_send_command(ins, 0xFCU);
}

J4310_Status_t j4310_disable(const j4310_instance *ins)
{
    return j4310_send_command(ins, 0xFDU);
}

J4310_Status_t j4310_request_measure(const j4310_instance *ins)
{
    if (ins == NULL || !ins->init || ins->can_ins == NULL)
        return J4310_ERROR_NOT_INIT;
    const uint8_t data[4] = {(uint8_t)ins->motor_id, (uint8_t)(ins->motor_id >> 8), 0xCC, 0x00};
    if (BSP_CAN_Transmit(ins->can_ins, 0x7FFU, CAN_ID_STD, data, 4U) != CAN_OK)
        return J4310_ERROR_CAN_TX;
    return J4310_OK;
}

J4310_Status_t j4310_ctrl_mit(const j4310_instance *ins, float p_des, float v_des,
                              float kp, float kd, float t_ff)
{
    if (ins == NULL || !ins->init || ins->can_ins == NULL)
        return J4310_ERROR_NOT_INIT;
    if (ins->mode != J4310_MODE_MIT)
        return J4310_ERROR_INVALID_MODE;
    if (!isfinite(p_des) || !isfinite(v_des) || !isfinite(kp) || !isfinite(kd) || !isfinite(t_ff) ||
        p_des < -ins->p_max || p_des > ins->p_max ||
        v_des < -ins->v_max || v_des > ins->v_max ||
        t_ff < -ins->t_max || t_ff > ins->t_max ||
        kp < 0.0f || kp > J4310_KP_MAX || kd < 0.0f || kd > J4310_KD_MAX ||
        (kp > 0.0f && kd == 0.0f))
        return J4310_ERROR_INVALID_PARAM;

    const uint16_t p = j4310_encode_signed(p_des, ins->p_max, 16);
    const uint16_t v = j4310_encode_signed(v_des, ins->v_max, 12);
    const uint16_t k_p = (uint16_t)(kp / J4310_KP_MAX * 4095.0f);
    const uint16_t k_d = (uint16_t)(kd / J4310_KD_MAX * 4095.0f);
    const uint16_t t = j4310_encode_signed(t_ff, ins->t_max, 12);
    uint8_t data[8];
    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)((v << 4) | (k_p >> 8));
    data[4] = (uint8_t)k_p;
    data[5] = (uint8_t)(k_d >> 4);
    data[6] = (uint8_t)((k_d << 4) | (t >> 8));
    data[7] = (uint8_t)t;
    return j4310_transmit(ins, 0U, data, 8U);
}

J4310_Status_t j4310_ctrl_pos_vel(const j4310_instance *ins, float p_des, float v_limit)
{
    if (ins == NULL || !ins->init || ins->can_ins == NULL)
        return J4310_ERROR_NOT_INIT;
    if (ins->mode != J4310_MODE_POS_VEL)
        return J4310_ERROR_INVALID_MODE;
    if (!isfinite(p_des) || !isfinite(v_limit) || v_limit < 0.0f)
        return J4310_ERROR_INVALID_PARAM;

    uint8_t data[8];
    j4310_write_float(&data[0], p_des);
    j4310_write_float(&data[4], v_limit);
    return j4310_transmit(ins, 0x100U, data, 8U);
}

J4310_Status_t j4310_ctrl_vel(const j4310_instance *ins, float v_des)
{
    if (ins == NULL || !ins->init || ins->can_ins == NULL)
        return J4310_ERROR_NOT_INIT;
    if (ins->mode != J4310_MODE_VEL)
        return J4310_ERROR_INVALID_MODE;
    if (!isfinite(v_des))
        return J4310_ERROR_INVALID_PARAM;

    uint8_t data[4];
    j4310_write_float(data, v_des);
    return j4310_transmit(ins, 0x200U, data, 4U);
}
