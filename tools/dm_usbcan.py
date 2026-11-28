"""DM Tool classic USB2CAN serial framing (dmBots u2can SDK)."""
import struct


def packet(can_id, data):
    if not 0 <= can_id <= 0x7ff or len(data) > 8:
        raise ValueError("需要标准 CAN 数据帧，DLC 最大为 8")
    buf = bytearray(30)
    buf[:5] = bytes.fromhex("55 AA 1E 03 01")
    buf[8] = 10
    struct.pack_into("<I", buf, 13, can_id)
    buf[18] = len(data)
    buf[21:21 + len(data)] = data
    return bytes(buf)


class Receiver:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        while len(self.buffer) >= 16:
            start = self.buffer.find(b"\xaa")
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < 16:
                break
            if self.buffer[15] != 0x55:
                del self.buffer[0]
                continue
            frame = self.buffer[:16]
            del self.buffer[:16]
            can_id = int.from_bytes(frame[3:7], "little")
            dlc = frame[2]
            if frame[1] == 0x11 and can_id <= 0x7ff and dlc <= 8:
                yield can_id, bytes(frame[7:7 + dlc])
