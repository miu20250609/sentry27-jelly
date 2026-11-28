#include "nuc.h"
#include "crc.h"
#include "usart.h"
#include "user_lib.h"
#include <math.h>
#include <string.h>

#define NUC_SOF 0xA5U
#define NUC_HEADER_SIZE 5U
#define NUC_PAYLOAD_OFFSET 7U
#define NUC_FRAME_OVERHEAD (NUC_PAYLOAD_OFFSET + 2U)
#define NUC_CHASSIS_TIMEOUT_MS 200U
#define NUC_REFEREE_QUEUE_DEPTH 16U
#define NUC_REFEREE_FRAGMENT_SIZE 6U

typedef struct
{
    uint16_t length;
    uint8_t data[RX_BUFFER_SIZE];
} nuc_referee_frame_t;

static nuc_referee_frame_t referee_tx_frame;
static uint16_t referee_tx_offset;
static uint8_t referee_tx_id;
static uint32_t referee_tx_mailbox;
static bool referee_tx_pending;

static void NucRxCallback(uint8_t* data, uint16_t len);
static void NUC_CanRxCallback(const uint8_t* data, uint32_t id, void* arg);
static void NUC_FoldCanRxCallback(const uint8_t* data, uint32_t id, void* arg);
static void NUC_UpdateFold(uint8_t flags, uint32_t update_time);
static void NUC_Chassis_Handler(const uint8_t* data, uint16_t len);
static void NUC_Gimbal_Handler(const uint8_t* data, uint16_t len);
static void NUC_UpdateChassis(fp32 vx, fp32 vy, uint8_t flags);
static void NUC_UpdateGimbal(fp32 yaw);

static nuc_ctrl_t nuc_instance;
static uint8_t nuc_rx_buffer[RX_BUFFER_SIZE];
static uint16_t nuc_rx_size;
static bool nuc_uart_ready;
static CAN_Instance_t* nuc_fold_can;
static bool nuc_chassis_received;
static uint32_t nuc_chassis_rx_tick;

static void NUC_InitUART(UART_HandleTypeDef* nuc_uart, UART_RxCallback_t rx_callback)
{
    if (nuc_uart != NULL && !nuc_uart_ready)
        nuc_uart_ready = BSP_UART_Init(&nuc_instance.nuc_uart, nuc_uart, 115200, rx_callback, NULL,
                                      TX_BUFFER_SIZE, RX_BUFFER_SIZE) == UART_OK;
}

void NUC_Init(UART_HandleTypeDef* nuc_uart)
{
    if (nuc_instance.init || nuc_uart_ready || nuc_fold_can != NULL)
        return;
    NUC_InitUART(nuc_uart, NucRxCallback);
    nuc_instance.init = nuc_uart_ready;
}

CAN_Status_t NUC_InitRefereeCAN(CAN_HandleTypeDef* referee_can)
{
    if (nuc_instance.referee_queue != NULL)
        return CAN_OK;
    nuc_instance.referee_can = BSP_CAN_Init(referee_can);
    if (nuc_instance.referee_can == NULL)
        return CAN_ERROR;
    nuc_instance.referee_queue = xQueueCreate(NUC_REFEREE_QUEUE_DEPTH, sizeof(nuc_referee_frame_t));
    return nuc_instance.referee_queue != NULL ? CAN_OK : CAN_ERROR;
}

CAN_Status_t NUC_Init_CAN(CAN_HandleTypeDef* nuc_can)
{
    if (nuc_instance.init || nuc_fold_can != NULL)
        return CAN_ERROR;

    if (NUC_InitRefereeCAN(nuc_can) != CAN_OK)
        return CAN_ERROR;
    CAN_Instance_t* can = BSP_CAN_Init(nuc_can);
    if (can == NULL)
        return CAN_ERROR;

    CAN_Status_t status = BSP_CAN_RegisterStdDataCallback(can, CMD_ID_SELF_DECISION, 4, NUC_CanRxCallback, NULL);
    if (status == CAN_OK)
        status = BSP_CAN_RegisterStdDataCallback(can, CMD_ID_CHASSIS_CTRL, 5, NUC_CanRxCallback, NULL);
    if (status == CAN_OK)
        status = BSP_CAN_RegisterStdDataCallback(can, CMD_ID_BIG_GIMBAL_CTRL, 2, NUC_CanRxCallback, NULL);
    if (status == CAN_OK)
        nuc_instance.init = 1;
    return status;
}

CAN_Status_t NUC_InitFoldCAN(CAN_HandleTypeDef* nuc_can)
{
    if (nuc_can == NULL)
        return CAN_ERROR;
    if (nuc_fold_can != NULL)
        return nuc_fold_can->handle == nuc_can ? CAN_OK : CAN_ERROR;
    if (nuc_instance.init || nuc_uart_ready)
        return CAN_ERROR;
    CAN_Instance_t* can = BSP_CAN_Init(nuc_can);
    if (can == NULL)
        return CAN_ERROR;
    const CAN_Status_t status = BSP_CAN_RegisterStdDataCallback(can, CMD_ID_CHASSIS_CTRL, 5,
                                                               NUC_FoldCanRxCallback, NULL);
    if (status == CAN_OK)
        nuc_fold_can = can;
    return status;
}

static void NUC_FoldCanRxCallback(const uint8_t* data, uint32_t id, void* arg)
{
    (void)arg;
    if (data != NULL && id == CMD_ID_CHASSIS_CTRL && nuc_fold_can != NULL)
        NUC_UpdateFold(data[4], HAL_GetTick());
}

static int16_t NUC_ReadInt16LE(const uint8_t* data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void NUC_CanRxCallback(const uint8_t* data, uint32_t id, void* arg)
{
    (void)arg;
    if (!nuc_instance.init)
        return;

    switch (id)
    {
        case CMD_ID_SELF_DECISION:
            nuc_instance.self_decision = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                                         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
            break;
        case CMD_ID_CHASSIS_CTRL:
            NUC_UpdateChassis(NUC_ReadInt16LE(&data[0]) / 1000.0f,
                              NUC_ReadInt16LE(&data[2]) / 1000.0f, data[4]);
            break;
        case CMD_ID_BIG_GIMBAL_CTRL:
            NUC_UpdateGimbal(NUC_ReadInt16LE(data) / 100.0f);
            break;
        default:
            break;
    }
}

static void NUC_DispatchFrame(const uint8_t* frame, const uint16_t payload_size)
{
    const uint16_t cmd_id = (frame[6] << 8) | frame[5];
    const uint8_t *payload = &frame[NUC_PAYLOAD_OFFSET];
    switch (cmd_id)
    {
        case CMD_ID_SELF_DECISION:
            if (payload_size >= sizeof(nuc_instance.self_decision))
                memcpy(&nuc_instance.self_decision, payload, sizeof(nuc_instance.self_decision));
            break;
        case CMD_ID_CHASSIS_CTRL:
            NUC_Chassis_Handler(payload, payload_size);
            break;
        case CMD_ID_BIG_GIMBAL_CTRL:
            NUC_Gimbal_Handler(payload, payload_size);
            break;
        default:
            break;
    }
}

static void NUC_ParseRxBuffer(void)
{
    uint16_t consumed = 0;
    while (consumed < nuc_rx_size)
    {
        uint8_t *frame = &nuc_rx_buffer[consumed];
        const uint16_t available = nuc_rx_size - consumed;
        if (frame[0] != NUC_SOF)
        {
            consumed++;
            continue;
        }
        if (available < NUC_HEADER_SIZE)
            break;

        const uint16_t payload_size = frame[1] | (frame[2] << 8);
        if (!Verify_CRC8_Check_Sum(frame, NUC_HEADER_SIZE) ||
            payload_size > sizeof(nuc_rx_buffer) - NUC_FRAME_OVERHEAD)
        {
            consumed++;
            continue;
        }

        const uint16_t frame_size = payload_size + NUC_FRAME_OVERHEAD;
        if (available < frame_size)
            break;
        if (!Verify_CRC16_Check_Sum(frame, frame_size))
        {
            consumed++;
            continue;
        }

        NUC_DispatchFrame(frame, payload_size);
        consumed += frame_size;
    }

    if (consumed != 0)
    {
        nuc_rx_size -= consumed;
        memmove(nuc_rx_buffer, &nuc_rx_buffer[consumed], nuc_rx_size);
    }
}

static void NucRxCallback(uint8_t* data, const uint16_t len)
{
    if (!nuc_instance.init)
        return;

    // DMA 空闲回调可能带来半帧或多帧，保留还没收齐的候选帧。
    for (uint16_t offset = 0; offset < len;)
    {
        uint16_t copy_size = sizeof(nuc_rx_buffer) - nuc_rx_size;
        if (copy_size > len - offset)
            copy_size = len - offset;
        memcpy(&nuc_rx_buffer[nuc_rx_size], &data[offset], copy_size);
        nuc_rx_size += copy_size;
        offset += copy_size;
        NUC_ParseRxBuffer();
    }
}

static void NUC_Chassis_Handler(const uint8_t* data, const uint16_t len)
{
    if (len < 9)
        return;

    fp32 vx, vy;
    unpack_4bytes_to_floats(&data[0], &vx);
    unpack_4bytes_to_floats(&data[4], &vy);
    if (!isfinite(vx) || !isfinite(vy))
        return;
    NUC_UpdateChassis(vx, vy, data[8]);
}

static void NUC_UpdateChassis(fp32 vx, fp32 vy, uint8_t flags)
{
    nuc_instance.vx = vx;
    nuc_instance.vy = vy;
    nuc_instance.chassis_spin = flags & 0x01;
    nuc_instance.use_cap = (flags >> 2) & 0x01;
    nuc_chassis_rx_tick = HAL_GetTick();
    nuc_chassis_received = true;
    NUC_UpdateFold(flags, nuc_chassis_rx_tick);
}

static void NUC_UpdateFold(uint8_t flags, uint32_t update_time)
{
    nuc_instance.fold_command.fold = (flags >> 1) & 0x01U;
    nuc_instance.fold_command.update_time = update_time;
    nuc_instance.fold_received = 1U;
}

bool NUC_GetFoldCommand(nuc_fold_cmd_t* command)
{
    if (command == NULL)
        return false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool fresh = nuc_instance.fold_received &&
        (uint32_t)(HAL_GetTick() - nuc_instance.fold_command.update_time) < NUC_CHASSIS_TIMEOUT_MS;
    if (fresh)
        *command = nuc_instance.fold_command;
    __set_PRIMASK(primask);
    return fresh;
}

bool NUC_GetChassisCommand(nuc_chassis_cmd_t* command)
{
    // 防止 CAN/UART 接收中断在读出同一帧的字段时更新它们。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (nuc_chassis_received && HAL_GetTick() - nuc_chassis_rx_tick >= NUC_CHASSIS_TIMEOUT_MS)
        nuc_chassis_received = false;

    const bool fresh = nuc_chassis_received;
    if (fresh)
    {
        *command = (nuc_chassis_cmd_t) {
            .vx = nuc_instance.vx, .vy = nuc_instance.vy, .vw = nuc_instance.vw,
            .chassis_spin = nuc_instance.chassis_spin,
            .use_cap = nuc_instance.use_cap,
        };
    }
    __set_PRIMASK(primask);
    return fresh;
}

static void NUC_Gimbal_Handler(const uint8_t* data, const uint16_t len)
{
    if (len < 4)
        return;

    fp32 yaw;
    unpack_4bytes_to_floats(&data[0], &yaw);
    NUC_UpdateGimbal(yaw);
}

static void NUC_UpdateGimbal(fp32 yaw)
{
    if (!isfinite(yaw))
        return;
    nuc_instance.gimbal_yaw = yaw;
    nuc_instance.gimbal_yaw_sequence++;
    nuc_instance.gimbal_yaw_received = 1;
}

bool NUC_GetGimbalCommand(fp32* yaw, uint32_t* sequence)
{
    if (yaw == NULL || sequence == NULL)
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool received = nuc_instance.gimbal_yaw_received;
    if (received)
    {
        *yaw = nuc_instance.gimbal_yaw;
        *sequence = nuc_instance.gimbal_yaw_sequence;
    }
    __set_PRIMASK(primask);
    return received;
}

void NUC_Referee_Tran(const uint8_t* data, const uint16_t len)
{
    if (nuc_instance.referee_queue == NULL || data == NULL ||
        len < NUC_FRAME_OVERHEAD || len > RX_BUFFER_SIZE)
        return;

    nuc_referee_frame_t frame = {.length = len};
    memcpy(frame.data, data, len);
    BaseType_t status;
    if (__get_IPSR() != 0U)
    {
        BaseType_t task_woken = pdFALSE;
        status = xQueueSendFromISR(nuc_instance.referee_queue, &frame, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
    else
        status = xQueueSend(nuc_instance.referee_queue, &frame, 0);
    if (status != pdPASS)
        nuc_instance.referee_drop_count++;
}

void NUC_Referee_Process(void)
{
    if (nuc_instance.referee_queue == NULL || nuc_instance.referee_can == NULL)
        return;
    CAN_HandleTypeDef* can = nuc_instance.referee_can->handle;
    // 同一 ID 的分包逐包完成后再提交，避免多个邮箱导致片序改变。
    if (referee_tx_pending)
    {
        if (HAL_CAN_IsTxMessagePending(can, referee_tx_mailbox))
            return;
        referee_tx_pending = false;
    }
    if (referee_tx_offset >= referee_tx_frame.length)
    {
        if (xQueueReceive(nuc_instance.referee_queue, &referee_tx_frame, 0) != pdPASS)
            return;
        referee_tx_offset = 0;
        referee_tx_id++;
    }
    uint16_t size = referee_tx_frame.length - referee_tx_offset;
    if (size > NUC_REFEREE_FRAGMENT_SIZE)
        size = NUC_REFEREE_FRAGMENT_SIZE;
    uint8_t data[8] = {referee_tx_id, referee_tx_offset / NUC_REFEREE_FRAGMENT_SIZE};
    memcpy(data + 2, referee_tx_frame.data + referee_tx_offset, size);
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = CMD_ID_REFEREE_FEEDBACK;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = size + 2U;

    // 只在短临界区尝试提交，不调用会等待邮箱的 BSP_CAN_Transmit。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_CAN_GetTxMailboxesFreeLevel(can) != 0U &&
        HAL_CAN_AddTxMessage(can, &header, data, &referee_tx_mailbox) == HAL_OK)
    {
        referee_tx_offset += size;
        referee_tx_pending = true;
    }
    __set_PRIMASK(primask);
}

nuc_ctrl_t *Get_NUC_Ctrl_Instance(void)
{
    return &nuc_instance;
}
