#!/usr/bin/env python3
"""JK-BD BMS reader — Pi 5 hardware UART (GPIO 12/13). Polls every 2 s, prints SOC
and pack V, and publishes them on ROS so telemetry_udp_bridge forwards the real
battery % (SOC) to the app instead of a motor-voltage estimate.

Wiring: BMS TX -> Pi GPIO13 (RXD), BMS RX -> Pi GPIO12 (TXD), GND common, 3.3 V.
GPIO 12/13 on the Pi 5 is `dtoverlay=uart4-pi5` -> /dev/ttyAMA4 (see
pi5_setup_commands.txt). Override with the BMS_PORT env var if it differs."""

import os
import serial
import struct
import time
import signal
import sys

import rclpy
from std_msgs.msg import UInt8, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Serial device for the BMS. Default = Pi 5 GPIO 12/13 UART (dtoverlay=uart4-pi5
# -> /dev/ttyAMA4). Override via env (e.g. BMS_PORT=/dev/ttyAMA5) if it differs.
PORT = os.environ.get('BMS_PORT', '/dev/ttyAMA4')

BAUD = 115200
INTERVAL = 2.0
HEADER = b'\x4e\x57'

READ_ALL = bytes([
    0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00,
    0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x68, 0x00, 0x00, 0x01, 0x29,
])

FIXED = {
    0x80: (2, 'mos_temp_c'),   0x81: (2, 'probe1_c'),
    0x82: (2, 'probe2_c'),     0x83: (2, 'pack_v'),
    0x84: (2, 'current_raw'),  0x85: (1, 'soc_pct'),
    0x86: (1, 'n_probes'),     0x87: (2, 'cycles'),
    0x89: (4, 'cycle_cap_ah'), 0x8A: (2, 'n_cells'),
    0x8B: (2, 'warn_bits'),    0x8C: (2, 'status_bits'),
}

WARN_BITS = {
    0: 'low_cap', 1: 'mos_overtemp', 2: 'charge_overvolt', 3: 'discharge_undervolt',
    4: 'probe_diff', 5: 'charge_overcurrent', 6: 'discharge_overcurrent',
    7: 'cell_delta', 8: 'overtemp', 9: 'cell_overvolt', 10: 'cell_undervolt',
    11: 'protection_309a', 12: 'protection_309b',
}


def checksum(data):
    return sum(data) & 0xFFFF


def find_frames(buf, verify=True):
    frames = []
    while True:
        start = buf.find(HEADER)
        if start < 0:
            if len(buf) > 1:
                del buf[:len(buf) - 1]
            break
        if start:
            del buf[:start]
        if len(buf) < 4:
            break
        length = struct.unpack('>H', buf[2:4])[0]
        if not (20 < length < 600):
            del buf[:2]
            continue
        total = 2 + length
        if len(buf) < total:
            break
        frame = bytes(buf[:total])
        del buf[:total]
        if verify and checksum(frame[:-4]) != struct.unpack('>H', frame[-2:])[0]:
            continue
        frames.append(frame)
    return frames


def read_frame(ser, timeout=1.5):
    buf = bytearray()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        chunk = ser.read(512)
        if chunk:
            buf += chunk
            got = find_frames(buf)
            if got:
                return got[0]
        else:
            time.sleep(0.01)
    return None


def _temp(raw):
    return float(-(raw - 100)) if raw > 100 else float(raw)


def parse(frame):
    if not frame or len(frame) < 20:
        return {}
    p = frame[11:-9]
    out, cells, i = {}, {}, 0
    while i < len(p):
        rid = p[i]
        i += 1
        try:
            if rid == 0x79:
                n = p[i]
                i += 1
                for k in range(0, n, 3):
                    cells[p[i + k]] = struct.unpack('>H', p[i + k + 1:i + k + 3])[0] / 1000.0
                i += n
            elif rid in FIXED:
                width, key = FIXED[rid]
                out[key] = int.from_bytes(p[i:i + width], 'big')
                i += width
            else:
                break
        except (IndexError, struct.error):
            break

    for k in ('mos_temp_c', 'probe1_c', 'probe2_c'):
        if k in out:
            out[k] = _temp(out[k])
    if 'pack_v' in out:
        out['pack_v'] /= 100.0
    if 'cycle_cap_ah' in out:
        out['cycle_cap_ah'] /= 1000.0
    if 'current_raw' in out:
        raw = out.pop('current_raw')
        out['current_a'] = ((raw & 0x7FFF) / 100.0) * (1 if raw & 0x8000 else -1)
    if 'warn_bits' in out:
        out['warnings'] = [n for b, n in WARN_BITS.items() if out['warn_bits'] & (1 << b)]
    if cells:
        vs = list(cells.values())
        out['cells_v'] = cells
        out['cell_min_v'] = min(vs)
        out['cell_max_v'] = max(vs)
        out['cell_delta_v'] = round(max(vs) - min(vs), 3)
    if 'pack_v' in out and 'current_a' in out:
        out['power_w'] = round(out['pack_v'] * out['current_a'], 1)
    return out


running = True


def stop(signum, frm):
    global running
    running = False


def open_port(port):
    ser = serial.Serial(port, BAUD, timeout=0.2)
    # RTS/DTR are modem-control lines an FTDI has but a raw Pi UART does not, so
    # setting them can raise on /dev/ttyAMA* — best-effort, ignore failures.
    try:
        ser.setRTS(False)
        ser.setDTR(False)
    except Exception:
        pass
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def main(args=None):
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    port = PORT
    try:
        ser = open_port(port)
    except serial.SerialException as e:
        print(f'Cannot open {port}: {e}', file=sys.stderr)
        print('Enable the UART (dtoverlay=uart4-pi5 -> /dev/ttyAMA4), free it from any '
              'serial-getty, add the user to dialout, or set BMS_PORT to the right '
              'device.', file=sys.stderr)
        sys.exit(1)

    print(f'Polling {port} every {INTERVAL:g}s. Ctrl-C to stop.\n')
    fails = 0

    # ROS: publish SOC (%) and pack voltage so telemetry_udp_bridge forwards the
    # real battery percentage to the app. Latched so a late subscriber gets the
    # last value immediately.
    rclpy.init(args=args)
    node = rclpy.create_node('battery_bms')
    latched = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_soc = node.create_publisher(UInt8, '/battery_soc', latched)
    pub_v   = node.create_publisher(Float32, '/battery_pack_v', latched)

    try:
        with ser:
            while running and rclpy.ok():
                t0 = time.monotonic()
                try:
                    ser.reset_input_buffer()
                    ser.write(READ_ALL)
                    ser.flush()
                    data = parse(read_frame(ser))
                except serial.SerialException as e:
                    print(f'serial error: {e}', file=sys.stderr)
                    data = {}

                ts = time.strftime('%H:%M:%S')
                if 'soc_pct' in data and 'pack_v' in data:
                    fails = 0
                    print(f'{ts}  SOC {data["soc_pct"]:3d} %   {data["pack_v"]:6.2f} V')
                    pub_soc.publish(UInt8(data=int(max(0, min(100, data["soc_pct"])))))
                    pub_v.publish(Float32(data=float(data["pack_v"])))
                else:
                    fails += 1
                    print(f'{ts}  no data ({fails})')
                    if fails == 5:
                        print('  -> BMS TX to Pi GPIO13 (RXD), BMS RX to Pi GPIO12 (TXD), '
                              'GND common, 3.3 V logic.', file=sys.stderr)

                sleep = INTERVAL - (time.monotonic() - t0)
                if sleep > 0:
                    time.sleep(sleep)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print('\nStopped.')


if __name__ == '__main__':
    main()