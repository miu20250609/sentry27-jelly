#include "im900.h"
#include <string.h>

#define IM900_BAUDRATE 115200U
#define IM900_PAYLOAD_MAX 73U
#define IM900_FRAME_OVERHEAD 5U
#define IM900_REPORT_HEADER 7U

static im900_t im900_instance;
static uint8_t rx_buffer[IM900_PAYLOAD_MAX + IM900_FRAME_OVERHEAD];
static uint16_t rx_size;

static int16_t IM900_ReadInt16(const uint8_t* data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void IM900_DecodeReport(const uint8_t* payload, uint8_t length)
{
    // IM948 SDK 1.05 的订阅位 0..11，按位序排列，其他位尚未定义。
    static const uint8_t field_sizes[12] = {6, 6, 6, 6, 8, 8, 6, 6, 5, 6, 2, 1};
    if (length < IM900_REPORT_HEADER || payload[0] != 0x11U)
        return;

    const uint16_t flags = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
    if ((flags & 0xF000U) != 0U)
        return;
    uint16_t expected_size = IM900_REPORT_HEADER;
    for (uint8_t bit = 0; bit < 12; bit++)
        if ((flags & (1U << bit)) != 0U)
            expected_size += field_sizes[bit];
    if (length != expected_size)
        return;

    im900_data_t data = {0};
    data.system_time = (uint32_t)payload[3] | ((uint32_t)payload[4] << 8) |
                       ((uint32_t)payload[5] << 16) | ((uint32_t)payload[6] << 24);
    data.data_flags = flags & (IM900_DATA_GYRO | IM900_DATA_QUAT | IM900_DATA_EULER);
    if (data.data_flags == 0U)
        return;

    uint16_t offset = IM900_REPORT_HEADER;
    for (uint8_t bit = 0; bit < 12; bit++)
    {
        if ((flags & (1U << bit)) == 0U)
            continue;
        const uint8_t* field = &payload[offset];
        switch (1U << bit)
        {
            case IM900_DATA_GYRO:
                for (uint8_t axis = 0; axis < 3; axis++)
                    data.gyr_b[axis] = IM900_ReadInt16(&field[axis * 2]) * (2000.0f / 32768.0f);
                break;
            case IM900_DATA_QUAT:
                for (uint8_t axis = 0; axis < 4; axis++)
                    data.quat[axis] = IM900_ReadInt16(&field[axis * 2]) / 32768.0f;
                break;
            case IM900_DATA_EULER:
                data.roll = IM900_ReadInt16(&field[0]) * (180.0f / 32768.0f);
                data.pitch = IM900_ReadInt16(&field[2]) * (180.0f / 32768.0f);
                data.yaw = IM900_ReadInt16(&field[4]) * (180.0f / 32768.0f);
                break;
            default:
                break;
        }
        offset += field_sizes[bit];
    }
    data.update_time = HAL_GetTick();
    im900_instance.imu_data = data;
    im900_instance.data_ready = 1;
}

static void IM900_ParseBuffer(void)
{
    uint16_t consumed = 0;
    while (consumed < rx_size)
    {
        const uint8_t* frame = &rx_buffer[consumed];
        const uint16_t available = rx_size - consumed;
        if (frame[0] != 0x49U)
        {
            consumed++;
            continue;
        }
        if (available < 3U)
            break;
        const uint8_t payload_size = frame[2];
        if (frame[1] == 0xFFU || payload_size == 0U || payload_size > IM900_PAYLOAD_MAX)
        {
            consumed++;
            continue;
        }
        const uint16_t frame_size = payload_size + IM900_FRAME_OVERHEAD;
        if (available < frame_size)
            break;
        uint8_t checksum = 0;
        for (uint16_t i = 1; i < payload_size + 3U; i++)
            checksum += frame[i];
        if (frame[frame_size - 1U] != 0x4DU || frame[frame_size - 2U] != checksum)
        {
            consumed++;
            continue;
        }
        IM900_DecodeReport(&frame[3], payload_size);
        consumed += frame_size;
    }
    if (consumed != 0U)
    {
        rx_size -= consumed;
        memmove(rx_buffer, &rx_buffer[consumed], rx_size);
    }
}

static void IM900_RxCallback(uint8_t* data, uint16_t length)
{
    if (data == NULL || !im900_instance.init)
        return;
    // 将 DMA 每次交付的数据作为字节流处理，保留跨回调的半帧。
    for (uint16_t offset = 0; offset < length;)
    {
        uint16_t copy_size = sizeof(rx_buffer) - rx_size;
        if (copy_size > length - offset)
            copy_size = length - offset;
        memcpy(&rx_buffer[rx_size], &data[offset], copy_size);
        rx_size += copy_size;
        offset += copy_size;
        IM900_ParseBuffer();
    }
}

UART_Status_t IM900_Init(UART_HandleTypeDef* uart_handle)
{
    if (uart_handle == NULL)
        return UART_ERROR_INVALID_PARAM;
    if (im900_instance.init)
        return UART_ERROR_ALREADY_INIT;

    const UART_Status_t status = BSP_UART_Init(&im900_instance.imu_uart, uart_handle,
        IM900_BAUDRATE, IM900_RxCallback, NULL, 0, RX_BUFFER_SIZE);
    if (status == UART_OK)
        im900_instance.init = 1;
    return status;
}

im900_t* Get_IM900_Instance(void)
{
    return &im900_instance;
}

bool IM900_GetData(im900_data_t* data)
{
    if (data == NULL)
        return false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool ready = im900_instance.data_ready != 0U;
    if (ready)
        *data = im900_instance.imu_data;
    __set_PRIMASK(primask);
    return ready;
}
