#!/usr/bin/env python3
"""Rover pack voltage/SOC reader — ADS1115 ADC over I2C. Polls every 2 s and publishes
SOC and pack V on ROS so telemetry_udp_bridge forwards the real battery % (SOC) to
the app instead of a motor-voltage estimate.

Runs silently: readings are published, not printed, because this node is launched
from bringup_sequence.launch.py and its stdout would otherwise flood
robot_startup_logs/02_bringup_launch.log. Only genuine faults go to stderr.

Wiring: pack+ -> 133k -> ADS1115 A0 -> 10k -> pack- (GND common with Pi). This divider
scales the pack voltage down by a factor of ~14.3, so ADS1115 A0 sees pack_v / 14.3.
ADS1115 ADDR tied to GND -> I2C address 0x48. VDD tied to Pi 5V rail (needed since
divider output exceeds 3.3V). See pi5_setup_commands.txt for I2C bus setup.
Override with ADC_ADDR / ADC_CHANNEL / ADC_AVG_N env vars if they differ.

SOC is a linear estimate between the pack's expected empty (39.0 V) and full
(53.3 V) resting voltages — there is no coulomb counting or cell-level data
without the BMS, so this is a voltage-based approximation only."""

import os
import time
import signal
import sys

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

import rclpy
from std_msgs.msg import UInt8, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ADS1115 config. Default = ADDR tied to GND -> 0x48, signal on A0.
ADC_ADDR = int(os.environ.get('ADC_ADDR', '0x48'), 16)
ADC_CHANNEL = int(os.environ.get('ADC_CHANNEL', '0'))  # 0..3 -> A0..A3
ADC_AVG_N = int(os.environ.get('ADC_AVG_N', '10'))       # moving-average sample count

INTERVAL = 1.0

# Pack-voltage EMA (exponential moving average), persistent across polls - smooths
# ADC/reference noise (scenario: SOC flickering 97/98% at idle) and softens
# transient IR-drop sag under load without fully hiding real depletion trends.
# Larger tau = smoother but slower to react to genuine voltage change.
PACK_V_EMA_TAU_S = float(os.environ.get('PACK_V_EMA_TAU_S', '20.0'))

# Published-SOC debounce: only change the published integer % once the filtered
# value has agreed with the new SOC for this many consecutive polls in a row.
# Kills single-poll flicker across a rounding boundary (e.g. 97% <-> 98%).
SOC_DEBOUNCE_POLLS = int(os.environ.get('SOC_DEBOUNCE_POLLS', '5'))

# Divider: 133k (top) + 10k (bottom), nominal ratio 14.3. Empirically calibrated
# against a multimeter (pack 52.9V vs topic-implied ADC reading) gives 14.69 -
# this folds in both resistor tolerance and the ADS1115's own reference error.
# Re-measure and update if you change the divider resistors or swap the ADC.
DIVIDER_RATIO = 14.69

# SOC mapped linearly between these resting-voltage endpoints (13S pack).
SOC_V_EMPTY = 39.0   # -> 0 %
SOC_V_FULL = 53.3    # -> 100 %

# P0-P3 are just 0-3 internally; using plain ints here avoids depending on
# ADS.P0/P1/P2/P3 attribute names, which have moved between library versions.
_ADC_CHANNELS = {0: 0, 1: 1, 2: 2, 3: 3}


def voltage_to_soc(pack_v):
    if SOC_V_FULL == SOC_V_EMPTY:
        return 0
    frac = (pack_v - SOC_V_EMPTY) / (SOC_V_FULL - SOC_V_EMPTY)
    return int(round(max(0.0, min(1.0, frac)) * 100))


running = True


def stop(signum, frm):
    global running
    running = False


def open_adc():
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c, address=ADC_ADDR)
    if ADC_CHANNEL not in _ADC_CHANNELS:
        raise ValueError(f'ADC_CHANNEL must be 0-3, got {ADC_CHANNEL}')
    chan = AnalogIn(ads, _ADC_CHANNELS[ADC_CHANNEL])
    return chan


def read_pack_v(chan, n):
    """Average n raw ADC voltage samples, then undo the divider."""
    total = 0.0
    got = 0
    for _ in range(n):
        try:
            total += chan.voltage
            got += 1
        except OSError:
            # transient I2C hiccup on one sample - skip it, don't kill the poll
            continue
    if got == 0:
        raise OSError('no successful ADC samples')
    return (total / got) * DIVIDER_RATIO


def main(args=None):
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        chan = open_adc()
    except (ValueError, OSError, RuntimeError) as e:
        print(f'Cannot open ADS1115 (addr={hex(ADC_ADDR)}): {e}', file=sys.stderr)
        print('Check I2C is enabled, wiring to SDA/SCL, ADDR pin strapping, and '
              'ADC_ADDR/ADC_CHANNEL env vars.', file=sys.stderr)
        sys.exit(1)

    fails = 0
    filtered_v = None          # persistent EMA state across polls
    published_soc = None       # last SOC actually published
    pending_soc = None         # candidate SOC waiting to be confirmed
    pending_count = 0          # consecutive polls pending_soc has held
    # EMA smoothing factor from time constant: alpha = INTERVAL / (tau + INTERVAL)
    ema_alpha = INTERVAL / (PACK_V_EMA_TAU_S + INTERVAL)

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
        while running and rclpy.ok():
            t0 = time.monotonic()
            try:
                pack_v = read_pack_v(chan, ADC_AVG_N)
                data_ok = True
            except OSError as e:
                print(f'ADC read error: {e}', file=sys.stderr)
                data_ok = False

            if data_ok:
                fails = 0
                filtered_v = pack_v if filtered_v is None else (
                    ema_alpha * pack_v + (1 - ema_alpha) * filtered_v)
                candidate_soc = voltage_to_soc(filtered_v)

                if published_soc is None:
                    # First good reading: publish immediately, no need to debounce.
                    published_soc = candidate_soc
                    pending_soc = candidate_soc
                    pending_count = 0
                elif candidate_soc == published_soc:
                    # Matches what's already published - nothing pending.
                    pending_soc = candidate_soc
                    pending_count = 0
                else:
                    if candidate_soc == pending_soc:
                        pending_count += 1
                    else:
                        pending_soc = candidate_soc
                        pending_count = 1
                    if pending_count >= SOC_DEBOUNCE_POLLS:
                        published_soc = pending_soc
                        pending_count = 0

                pub_soc.publish(UInt8(data=published_soc))
                pub_v.publish(Float32(data=float(filtered_v)))
            else:
                fails += 1
                # Warn once, not every poll: a silent ADC must still be visible
                # without spamming the launch log.
                if fails == 5:
                    print('battery_bms: no data after 5 polls -> check ADS1115 wiring '
                          '(A0/GND/VDD/SDA/SCL), ADDR strapping, and I2C bus.',
                          file=sys.stderr)

            sleep = INTERVAL - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
