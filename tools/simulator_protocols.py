"""Frame builders for the desktop simulator. CAN uses little-endian payloads."""
import math
import struct

from referee_simulator import MESSAGES, encode, payloads


REF_NAMES = ("比赛状态", "机器人血量", "场地事件", "机器人状态", "功率与热量",
             "增益", "受击事件", "弹量", "RFID", "哨兵信息")
REF_COMMANDS = {f"0x{cmd:04X}  {name}": cmd for (cmd, *_), name in zip(MESSAGES, REF_NAMES)}
MODES = {"正常": "normal", "拆帧": "split", "连续写入": "burst",
         "错误 CRC": "bad-crc", "错误载荷长度": "bad-length"}


def decision(value):
    number = int(value, 0)
    if not 0 <= number <= 0xffffffff:
        raise ValueError("自决策值范围为 0 到 4294967295")
    return struct.pack("<I", number)


def chassis(vx, vy, fold, spin, cap):
    speeds = [float(vx), float(vy)]
    if not all(math.isfinite(v) and -32.768 <= v <= 32.767 for v in speeds):
        raise ValueError("速度须在 −32.768 至 32.767 m/s 之间")
    return struct.pack("<hhB", *(round(v * 1000) for v in speeds),
                       int(spin) | int(fold) << 1 | int(cap) << 2)


def referee_frames(commands, elapsed, robot_id, sequence, mode):
    samples = payloads(elapsed, robot_id)
    return [(cmd, encode(cmd, samples[cmd], sequence + i, mode))
            for i, cmd in enumerate(commands)]


def fold_status(data):
    if len(data) != 1:
        raise ValueError("0x505 应为 DLC 1")
    names = ("禁瞄联锁", "折叠请求有效", "小轴避让到位", "大 pitch 折叠到位", "大 pitch 直立到位")
    return " · ".join(name for i, name in enumerate(names) if data[0] & (1 << i)) or "所有标志为 0"
