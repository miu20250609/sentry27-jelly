#ifndef STANDARD_ROBOT_C_IM900_H
#define STANDARD_ROBOT_C_IM900_H

#include <stdbool.h>
#include "typedef.h"
#include "usart/bsp_uart.h"

#define IM900_DATA_GYRO 0x0004U
#define IM900_DATA_QUAT 0x0020U
#define IM900_DATA_EULER 0x0040U

typedef struct
{
    fp32 roll, pitch, yaw; // 模块 X/Y/Z 欧拉角，单位 °，保持模块配置的坐标系。
    fp32 gyr_b[3]; // XYZ，单位 °/s。
    fp32 quat[4]; // WXYZ。
    uint32_t system_time; // 模块时间戳，ms。
    uint32_t update_time; // C 板收到此帧的 HAL tick，ms。
    uint16_t data_flags; // 本帧实际包含的数据，使用 IM900_DATA_* 判断有效字段。
} im900_data_t;

typedef struct
{
    UART_Instance_t imu_uart;
    im900_data_t imu_data;
    uint8_t init;
    uint8_t data_ready;
} im900_t;

UART_Status_t IM900_Init(UART_HandleTypeDef* uart_handle);
im900_t* Get_IM900_Instance(void);
bool IM900_GetData(im900_data_t* data);

#endif
