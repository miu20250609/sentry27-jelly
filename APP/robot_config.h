/* 板型由 Keil 配置或 CMake 的 SENTRY_BUILD_CHASSIS 选择。 */
#ifndef STANDARD_ROBOT_C_ROBOT_CONFIG_H
#define STANDARD_ROBOT_C_ROBOT_CONFIG_H

// 底盘 NUC 控制来源：1 = CAN2，0 = USART6。
#ifndef NUC_USE_CAN
#define NUC_USE_CAN 1
#endif

// 底盘仅接 USART1 和 CAN2 联调时置 1，恢复整车控制前改回 0。
#ifndef CHASSIS_COMM_ONLY
#define CHASSIS_COMM_ONLY 0
#endif
#if CHASSIS_COMM_ONLY != 0 && CHASSIS_COMM_ONLY != 1
#error "CHASSIS_COMM_ONLY must be 0 or 1"
#endif

// 大 yaw：电机配置须与调试助手一致；角度外环仍用 IM900。
#define BIG_YAW_CAN_HANDLE (&hcan2)
#define BIG_YAW_CAN_ID 0x01U
#define BIG_YAW_MASTER_ID 0x11U
#define BIG_YAW_P_MAX 12.5f
#define BIG_YAW_V_MAX 30.0f
#define BIG_YAW_T_MAX 10.0f
#define BIG_YAW_FEEDBACK_TIMEOUT_MS 200U
#define BIG_YAW_SERVICE_PERIOD_MS 100U

// 待装车标定：电机正向、编码器安装零位(rad)、编码器正向、IMU 控制正向。
#define BIG_YAW_MOTOR_SIGN 1.0f
#define BIG_YAW_ENCODER_ZERO_RAD 0.0f
#define BIG_YAW_ENCODER_SIGN 1.0f
#define BIG_YAW_IMU_SIGN (-1.0f)

// 输入误差为度，PID 输出及速度前馈为 rad/s。以下增益为待实机调整的初值。
#ifndef BIG_YAW_ANGLE_KP
#define BIG_YAW_ANGLE_KP 0.05f
#endif
#define BIG_YAW_ANGLE_KI 0.0f
#define BIG_YAW_ANGLE_KD 0.0f
#define BIG_YAW_SPEED_LIMIT_RAD_S 2.0f
#define BIG_YAW_INTEGRAL_LIMIT_RAD_S 1.0f
#define BIG_YAW_FF_GAIN 1.0f

// 选择 VOFA 接口，观测变量在各板 debug() 的发送数组中修改。
#define VOFA_PORT_OFF 0
#define VOFA_PORT_UART1 1
#define VOFA_PORT_UART6 6

#ifndef GIMBAL_VOFA_PORT
//#define GIMBAL_VOFA_PORT VOFA_PORT_UART1
//#define GIMBAL_VOFA_PORT VOFA_PORT_UART6
#define GIMBAL_VOFA_PORT VOFA_PORT_OFF
#endif

#ifndef CHASSIS_VOFA_PORT
#define CHASSIS_VOFA_PORT VOFA_PORT_UART6
//#define CHASSIS_VOFA_PORT VOFA_PORT_OFF
#endif

#if GIMBAL_VOFA_PORT != VOFA_PORT_OFF && GIMBAL_VOFA_PORT != VOFA_PORT_UART1 && GIMBAL_VOFA_PORT != VOFA_PORT_UART6
#error "GIMBAL_VOFA_PORT must be OFF, UART1 or UART6"
#endif
#if CHASSIS_VOFA_PORT != VOFA_PORT_OFF && CHASSIS_VOFA_PORT != VOFA_PORT_UART6
#error "CHASSIS_VOFA_PORT supports OFF or UART6"
#endif
#if CHASSIS_VOFA_PORT != VOFA_PORT_OFF && !NUC_USE_CAN
#error "Chassis UART6 VOFA requires NUC control over CAN"
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
