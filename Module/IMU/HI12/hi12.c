#include "hi12.h"
#include <math.h>
#include <string.h>
#include "crc.h"

#define HI12_HEADER_SIZE 6U
#define HI12_PAYLOAD_SIZE 76U
#define HI12_FRAME_SIZE (HI12_HEADER_SIZE + HI12_PAYLOAD_SIZE)

static hi12_t hi12_instance;
static uint8_t rx_buffer[HI12_FRAME_SIZE];
static uint16_t rx_size;

static void HI12_ParseBuffer(void)
{
    uint16_t consumed = 0;
    while (consumed < rx_size)
    {
        uint8_t* frame = &rx_buffer[consumed];
        const uint16_t available = rx_size - consumed;
        if (frame[0] != 0x5AU)
        {
            consumed++;
            continue;
        }
        if (available < 2U)
            break;
        if (frame[1] != 0xA5U)
        {
            consumed++;
            continue;
        }
        if (available < HI12_HEADER_SIZE)
            break;
        const uint16_t payload_size = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
        if (payload_size != HI12_PAYLOAD_SIZE)
        {
            consumed++;
            continue;
        }
        if (available < HI12_FRAME_SIZE)
            break;

        uint16_t crc = 0;
        crc16_update(&crc, frame, 4);
        crc16_update(&crc, frame + HI12_HEADER_SIZE, payload_size);
        const uint16_t expected = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
        if (crc != expected || frame[6] != 0x91U)
        {
            consumed++;
            continue;
        }

        SensorDataPacket sample;
        memcpy(&sample, frame, sizeof(sample));
        if (isfinite(sample.yaw) && isfinite(sample.pitch) && isfinite(sample.roll) &&
            sample.yaw >= -180.0f && sample.yaw <= 180.0f &&
            sample.pitch >= -90.0f && sample.pitch <= 90.0f)
        {
            hi12_instance.imu_data = sample;
            hi12_instance.update_time = HAL_GetTick();
            hi12_instance.data_ready = 1;
        }
        consumed += HI12_FRAME_SIZE;
    }
    if (consumed != 0U)
    {
        rx_size -= consumed;
        memmove(rx_buffer, &rx_buffer[consumed], rx_size);
    }
}

static void HI12_RxCallback(uint8_t* data, uint16_t length)
{
    if (!hi12_instance.init || data == NULL)
        return;
    for (uint16_t offset = 0; offset < length;)
    {
        uint16_t copy_size = sizeof(rx_buffer) - rx_size;
        if (copy_size > length - offset)
            copy_size = length - offset;
        memcpy(&rx_buffer[rx_size], &data[offset], copy_size);
        rx_size += copy_size;
        offset += copy_size;
        HI12_ParseBuffer();
    }
}

void HI12_Init(UART_HandleTypeDef* uart_handle)
{
    if (uart_handle == NULL || hi12_instance.init)
        return;
    if (BSP_UART_Init(&hi12_instance.imu_uart, uart_handle, 115200, HI12_RxCallback,
                      NULL, 0, RX_BUFFER_SIZE) == UART_OK)
        hi12_instance.init = 1;
}

hi12_t* Get_HI12_Instance(void)
{
    return &hi12_instance;
}

bool HI12_GetData(SensorDataPacket* data, uint32_t* update_time)
{
    if (data == NULL || update_time == NULL)
        return false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool ready = hi12_instance.data_ready != 0U;
    if (ready)
    {
        *data = hi12_instance.imu_data;
        *update_time = hi12_instance.update_time;
    }
    __set_PRIMASK(primask);
    return ready;
}
