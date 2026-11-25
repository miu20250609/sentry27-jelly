#!/usr/bin/env python3
"""RM2027 referee UART simulator. CAN reception is handled separately by the DM application."""
import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path


def crc8(data):
    value = 0xFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8C if value & 1 else 0)
    return value


def crc16(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def pack_frame(command, payload, sequence):
    header = struct.pack('<BHB', 0xA5, len(payload), sequence & 0xFF)
    body = header + bytes([crc8(header)]) + struct.pack('<H', command) + payload
    return body + struct.pack('<H', crc16(body))


# Rates follow periodic referee messages; 0x0206 synthesizes one hit event/second.
MESSAGES = (
    (0x0001, 'game_status', 1, 11),
    (0x0003, 'robot_hp', 3, 24),
    (0x0101, 'field_event', 1, 8),
    (0x0201, 'robot_status', 10, 17),
    (0x0202, 'power_heat', 10, 14),
    (0x0204, 'buff', 3, 7),
    (0x0206, 'hurt_event', 1, 7),
    (0x0208, 'ammunition', 10, 8),
    (0x0209, 'rfid', 3, 6),
    (0x020D, 'sentry_info', 1, 14),
)


def payloads(elapsed, robot_id):
    tick = int(elapsed)
    hp = 400 - tick % 100
    heat = int((math.sin(elapsed) + 1) * 40)
    posture = 1 + (tick // 3) % 3
    # Synthetic values exercise fields, not official robot limits or game logic.
    return {
        0x0001: struct.pack('<BHQ', 0x44, max(0, 420 - tick), 0),
        0x0003: struct.pack('<4Hh7H', 500, 0, 300, 300, -20, hp,
                            0, 1000, 0, 1000, tick % 100, 0),
        0x0101: struct.pack('<II', 1, (1 << 16) | (1 << 18)),
        0x0201: struct.pack('<BB5HfB', robot_id, 1, hp, 400, 40, 200, 80, 30.0, 7),
        0x0202: struct.pack('<HHfHHH', 0, 0, 0.0, 60, heat, 0),
        0x0204: struct.pack('<BHBBH', 10, 20, 30, 0, 40),
        0x0206: struct.pack('<BHhh', tick % 4, tick & 0xFFFF, -12, 34),
        0x0208: struct.pack('<4H', 200 - tick % 100, 0, 100, 0),
        # Alternate the three RMUL control-area RFID bits every three seconds.
        0x0209: struct.pack('<IH', 1 << (11 + (tick // 3) % 3), 0),
        # Encode ordinary posture in bits12..13; duration bytes are synthetic.
        0x020D: struct.pack('<IHQ', 0, posture << 12, 0x000A141E0028323C),
    }


def encode(command, payload, sequence, mode):
    if mode == 'bad-length':
        payload += b'\x00'  # Valid CRC/frame boundary, wrong command-specific length.
    frame = pack_frame(command, payload, sequence)
    if mode == 'bad-crc':
        frame = frame[:-1] + bytes([frame[-1] ^ 1])
    return frame


def self_test():
    assert crc8(b'123456789') == 0x0B
    assert crc16(b'123456789') == 0x6F91
    for robot_id in (7, 107):
        for elapsed in (0, 1, 3, 6, 100, 421):
            samples = payloads(elapsed, robot_id)
            for command, _, _, length in MESSAGES:
                payload = samples[command]
                assert len(payload) == length
                frame = encode(command, payload, 257, 'normal')
                assert frame[0] == 0xA5 and frame[3] == 1
                assert struct.unpack_from('<H', frame, 1)[0] == length
                assert crc8(frame[:4]) == frame[4]
                assert struct.unpack_from('<H', frame, 5)[0] == command
                assert crc16(frame[:-2]) == struct.unpack_from('<H', frame, len(frame)-2)[0]
                bad = encode(command, payload, 0, 'bad-crc')
                assert crc16(bad[:-2]) != struct.unpack_from('<H', bad, len(bad)-2)[0]
                wrong = encode(command, payload, 0, 'bad-length')
                assert len(wrong) == len(frame) + 1
                assert crc16(wrong[:-2]) == struct.unpack_from('<H', wrong, len(wrong)-2)[0]
    print('PASS: official lengths, little endian, signed fields, CRC8/CRC16, sequence wrap, fault modes')


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=str.strip, help='explicitly confirmed UART DM Tool port; no default')
    parser.add_argument('--list-ports', action='store_true', help='list current ports without opening them')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--robot-id', type=int, choices=(7, 107), default=7)
    parser.add_argument('--duration', type=float, default=30, help='seconds; 0 runs until Ctrl+C')
    parser.add_argument('--once', action='store_true', help='send one sample of all 10 messages')
    parser.add_argument('--mode', choices=('normal', 'split', 'burst', 'bad-crc', 'bad-length'), default='normal')
    parser.add_argument('--split-at', type=int, default=5, help='split offset in split mode')
    parser.add_argument('--gap-ms', type=float, default=5, help='gap between split writes')
    parser.add_argument('--verbose', action='store_true', help='print each frame in hex')
    parser.add_argument('--log', type=Path, help='JSONL log of generated original UART frames')
    parser.add_argument('--dry-run', action='store_true', help='print one cycle without opening a port')
    parser.add_argument('--self-test', action='store_true', help='offline checks, no port opened')
    args = parser.parse_args()
    if not (args.dry_run or args.self_test or args.list_ports) and not args.port:
        parser.error('sending requires --port; run --list-ports and confirm the UART adapter first')
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if not math.isfinite(args.gap_ms) or args.gap_ms < 0:
        parser.error('--gap-ms must be finite and nonnegative')
    if args.baud <= 0 or args.split_at < 1:
        parser.error('--baud and --split-at must be positive')
    if args.mode == 'split' and args.split_at >= min(row[3]+9 for row in MESSAGES):
        parser.error('--split-at must be less than 15 so every frame is actually split')
    return args


def run(args, serial_port=None, log=None):
    start = time.monotonic()
    next_due = [start] * len(MESSAGES)
    report_due = start
    sequence = 0
    frame_count = byte_count = 0
    single_cycle = args.once or args.dry_run
    while True:
        now = time.monotonic()
        elapsed = now - start
        if not single_cycle and args.duration and elapsed >= args.duration:
            break
        samples = payloads(elapsed, args.robot_id)
        batch = []
        for index, (command, name, hz, _) in enumerate(MESSAGES):
            if now < next_due[index]:
                continue
            frame = encode(command, samples[command], sequence, args.mode)
            batch.append((command, name, frame))
            sequence = (sequence + 1) & 0xFF
            # Skip missed deadlines rather than flooding a catch-up burst.
            period = 1.0 / hz
            next_due[index] += (int((now-next_due[index])/period) + 1) * period
        def write(data):
            if serial_port is not None and serial_port.write(data) != len(data):
                raise OSError('Incomplete UART write')
        if batch and args.mode == 'burst':
            write(b''.join(frame for _, _, frame in batch))
        for command, name, frame in batch:
            if args.mode == 'split':
                write(frame[:args.split_at])
                if serial_port is not None:
                    serial_port.flush()
                    time.sleep(args.gap_ms / 1000.0)
                write(frame[args.split_at:])
            elif args.mode != 'burst':
                write(frame)
            frame_count += 1
            byte_count += len(frame)
            if args.verbose or args.dry_run:
                print(f'{elapsed:8.3f}s 0x{command:04X} {name:14s} seq={frame[3]:3d} {frame.hex(" ").upper()}')
            if log is not None:
                log.write(json.dumps({'elapsed_s': round(elapsed, 6), 'command': f'0x{command:04X}',
                                      'sequence': frame[3], 'mode': args.mode, 'frame_hex': frame.hex(' ')}) + '\n')
        if now >= report_due and not args.dry_run:
            print(f'{elapsed:6.1f}s UART writes: {frame_count} frames, {byte_count} bytes (not C-board acknowledgements)')
            report_due = now + 1
            if log is not None:
                log.flush()
        if single_cycle:
            break
        time.sleep(max(0.001, min(0.01, min(next_due) - time.monotonic())))
    if serial_port is not None:
        serial_port.flush()
    print(f'Done: {frame_count} generated frames, {byte_count} bytes. Verify CAN ID 0x501 separately.')


def main():
    args = arguments()
    if args.self_test:
        self_test()
        return 0
    from contextlib import ExitStack
    try:
        if args.list_ports or not args.dry_run:
            try:
                from serial.tools import list_ports
            except ImportError:
                print('Install dependency: python -m pip install pyserial', file=sys.stderr)
                return 2
            ports = list(list_ports.comports())
            if args.list_ports:
                if not ports:
                    print('No serial ports found.')
                for item in sorted(ports, key=lambda item: item.device):
                    print(f'{item.device}: {item.description} | {item.hwid}')
                print('Identify the physically connected UART and CAN adapters; names do not determine their role.')
                return 0
            selected = next((item for item in ports if item.device.casefold() == args.port.casefold()), None)
            if selected is None:
                raise ValueError(f'{args.port} is not currently present; run --list-ports and confirm the wiring again')
            args.port = selected.device
            print(f'Selected UART adapter: {selected.device} | {selected.description} | {selected.hwid}')
        with ExitStack() as stack:
            log = stack.enter_context(args.log.open('w', encoding='utf-8')) if args.log else None
            if args.dry_run:
                run(args, log=log)
            else:
                try:
                    import serial
                except ImportError:
                    print('Install dependency: python -m pip install pyserial', file=sys.stderr)
                    return 2
                port = stack.enter_context(serial.Serial(args.port, args.baud, bytesize=8,
                    parity='N', stopbits=1, timeout=0, write_timeout=1))
                print(f'UART: {args.port} {args.baud} 8N1; mode={args.mode}; Only the selected UART port is opened; CAN reception stays in the DM application.')
                run(args, port, log)
    except KeyboardInterrupt:
        print('\nStopped; serial port closed.')
    except (OSError, ValueError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
