#include "Referee/referee.h"
#include "crc.h"
#include "usart.h"
#include "NUC/nuc.h"
#include <string.h>

static referee_t referee_instance;

void Referee_RxCallback(uint8_t* data, uint16_t len);
void Referee_Init(UART_HandleTypeDef *referee_uart)
{
    referee_t *referee_ptr = &referee_instance;
    memset(referee_ptr, 0, sizeof(referee_t));
    BSP_UART_Init(&referee_ptr->uart_instance, referee_uart, 115200, Referee_RxCallback, NULL, 0, RX_BUFFER_SIZE);
    referee_instance.init = 1;
}

void Referee_RxCallback(uint8_t* data, const uint16_t len)
{
    for (uint16_t i = 0; i + FRAME_HEADER_LEN + CMD_ID_LEN + 2U <= len; i++)
    {
        if (data[i] == SOF_VALUE)
        {
            uint8_t *rx_data = &data[i];
            if (!Verify_CRC8_Check_Sum(rx_data, FRAME_HEADER_LEN))
                continue;

            const uint16_t data_length = rx_data[1] | (rx_data[2] << 8);
            const uint32_t frame_length = (uint32_t)data_length + FRAME_HEADER_LEN + CMD_ID_LEN + 2U;
            if (frame_length > (uint32_t)(len - i) || !Verify_CRC16_Check_Sum(rx_data, frame_length))
                continue;

            const uint16_t cmd_id = (rx_data[6] << 8) | rx_data[5];

            switch (cmd_id)
            {
            case CMD_ID_GAME_STATUS:
                if (data_length >= sizeof(referee_instance.game_status))
                    memcpy(&referee_instance.game_status, &rx_data[7], sizeof(game_status_t));
                break;
            case CMD_ID_GAME_RESULT:
                if (data_length >= sizeof(referee_instance.game_result))
                    memcpy(&referee_instance.game_result, &rx_data[7], sizeof(game_result_t));
                break;
            case CMD_ID_GAME_ROBOT_HP:
                if (data_length >= sizeof(referee_instance.game_robot_hp))
                    memcpy(&referee_instance.game_robot_hp, &rx_data[7], sizeof(game_robot_HP_t));
                break;
            case CMD_ID_EVENT_DATA:
                if (data_length >= sizeof(referee_instance.event_data))
                    memcpy(&referee_instance.event_data, &rx_data[7], sizeof(event_data_t));
                break;
            case CMD_ID_REFEREE_WARNING:
                if (data_length >= sizeof(referee_instance.referee_warning))
                    memcpy(&referee_instance.referee_warning, &rx_data[7], sizeof(referee_warning_t));
                break;
            case CMD_ID_DART_INFO:
                if (data_length >= sizeof(referee_instance.dart_info))
                    memcpy(&referee_instance.dart_info, &rx_data[7], sizeof(dart_info_t));
                break;
            case CMD_ID_ROBOT_PERFORMANCE:
                if (data_length >= sizeof(referee_instance.robot_performance))
                    memcpy(&referee_instance.robot_performance, &rx_data[7], sizeof(robot_performance_t));
                break;
            case CMD_ID_POWER_HEAT_DATA:
                if (data_length >= sizeof(referee_instance.power_heat_data))
                    memcpy(&referee_instance.power_heat_data, &rx_data[7], sizeof(power_heat_data_t));
                break;
            case CMD_ID_ROBOT_POS:
                if (data_length >= sizeof(referee_instance.robot_pos))
                    memcpy(&referee_instance.robot_pos, &rx_data[7], sizeof(robot_pos_t));
                break;
            case CMD_ID_BUFF_DATA:
                if (data_length >= sizeof(referee_instance.buff_data))
                    memcpy(&referee_instance.buff_data, &rx_data[7], sizeof(buff_data_t));
                break;
            case CMD_ID_HURT_DATA:
                if (data_length >= sizeof(referee_instance.hurt_data))
                    memcpy(&referee_instance.hurt_data, &rx_data[7], sizeof(hurt_data_t));
                break;
            case CMD_ID_SHOOT_DATA:
                if (data_length >= sizeof(referee_instance.shoot_data))
                    memcpy(&referee_instance.shoot_data, &rx_data[7], sizeof(shoot_data_t));
                break;
            case CMD_ID_PROJECTILE_ALLOWANCE:
                if (data_length >= sizeof(referee_instance.projectile_allowance))
                    memcpy(&referee_instance.projectile_allowance, &rx_data[7], sizeof(projectile_allowance_t));
                break;
            case CMD_ID_RFID_STATUS:
                if (data_length >= sizeof(referee_instance.rfid_status))
                    memcpy(&referee_instance.rfid_status, &rx_data[7], sizeof(rfid_status_t));
                break;
            case CMD_ID_DART_CLIENT_CMD:
                if (data_length >= sizeof(referee_instance.dart_client_cmd))
                    memcpy(&referee_instance.dart_client_cmd, &rx_data[7], sizeof(dart_client_cmd_t));
                break;
            case CMD_ID_ROBOT_POS_EXT:
                if (data_length >= sizeof(referee_instance.robot_pos_ext))
                    memcpy(&referee_instance.robot_pos_ext, &rx_data[7], sizeof(robot_pos_ext_t));
                break;
            case CMD_ID_MARK_PROGRESS:
                if (data_length >= sizeof(referee_instance.mark_progress))
                    memcpy(&referee_instance.mark_progress, &rx_data[7], sizeof(mark_progress_t));
                break;
            case CMD_ID_SENTRY_INFO:
                if (data_length >= sizeof(referee_instance.sentry_info))
                    memcpy(&referee_instance.sentry_info, &rx_data[7], sizeof(sentry_info_t));
                break;
                // case CMD_ID_RADAR_INFO :
                //     memcpy(&referee_ins.radar_info, &rx_data[7], sizeof(radar_info_t));
                // break;
            default :
                break;
            }
            NUC_Referee_Tran(rx_data, (uint16_t)frame_length);
            i += (uint16_t)(frame_length - 1U);
        }
    }
}

void RefereeCmd_PackPacket(const uint8_t *data_in, const uint16_t data_len, const uint16_t cmd_id)
{
    const uint16_t total_len = FRAME_HEADER_LEN + CMD_ID_LEN + data_len + 2;
    uint8_t data_out[total_len];
    uint32_t offset = 0;
    data_out[offset++] = SOF_VALUE;
    data_out[offset++] = data_len & 0xFF;
    data_out[offset++] = (data_len >> 8) & 0xFF;

    data_out[offset++] = 0;
    data_out[offset++] = 0;
    data_out[offset++] = cmd_id & 0xFF;
    data_out[offset++] = (cmd_id >> 8) & 0xFF;
    if (data_in != NULL && data_len > 0) {
        memcpy(&data_out[offset], data_in, data_len);
        offset += data_len;
    }
    Append_CRC8_Check_Sum(data_out, FRAME_HEADER_LEN);
    Append_CRC16_Check_Sum(data_out, total_len);

    BSP_UART_Transmit_To_Mail(&referee_instance.uart_instance, data_out, sizeof(data_out), 100);
}

void Referee_SendSentryCmd(const uint8_t progress_revive, // 读条复活
    const uint8_t exchange_revive, // 兑换复活
    const uint16_t exchange_projectile, // 兑换发弹量
    const uint8_t remote_projectile_req, // 兑换发弹量的请求次数
    const uint8_t remote_hp_req, // 兑换血量的请求次数
    const uint8_t posture, // 修改当前姿态指令
    const uint8_t trigger_energy // 是否确认使能量机关进入正在激活状态
    )
{
    if (!referee_instance.init)
        return;
    sentry_cmd_t sentry_cmd = {0};

    if (progress_revive) sentry_cmd.sentry_cmd |= (1 << 0);
    if (exchange_revive) sentry_cmd.sentry_cmd |= (1 << 1);
    sentry_cmd.sentry_cmd |= ((uint32_t)(exchange_projectile & 0x7FF) << 2);
    sentry_cmd.sentry_cmd |= ((uint32_t)(remote_projectile_req & 0xF) << 13);
    sentry_cmd.sentry_cmd |= ((uint32_t)(remote_hp_req & 0xF) << 17);
    sentry_cmd.sentry_cmd |= ((uint32_t)(posture & 0x3) << 21);
    if (trigger_energy) sentry_cmd.sentry_cmd |= (1 << 23);

    RefereeCmd_PackPacket((uint8_t*)&sentry_cmd, sizeof(sentry_cmd), CMD_ID_SENTRY_SELFDECISION);
}

referee_t *Get_Referee_Instance(void)
{
    return &referee_instance;
}
