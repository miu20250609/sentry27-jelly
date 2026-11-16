/* 板型由 Keil 配置或 CMake 的 SENTRY_BUILD_CHASSIS 选择。 */
#ifndef STANDARD_ROBOT_C_ROBOT_CONFIG_H
#define STANDARD_ROBOT_C_ROBOT_CONFIG_H

// 底盘 NUC 控制来源：1 = CAN2，0 = USART6。裁判回传始终使用 USART6。
#ifndef NUC_USE_CAN
#define NUC_USE_CAN 1
#endif

#ifndef MADE_BY_CMAKE

//#define USE_CBOARD_IMU
#ifndef CHASSIS
#define CHASSIS
#endif
//#define GIMBAL
#ifndef OMNI
#define OMNI
#endif

#endif

#endif //STANDARD_ROBOT_C_ROBOT_CONFIG_H
