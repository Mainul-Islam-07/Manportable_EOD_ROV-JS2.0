#!/usr/bin/env python3
"""
fake_memory_battery.py
======================
Stand-in for the STM32 encoder memory-battery sender, for bench testing without
the hardware.

It emits the same ASCII UDP datagram that battery_monitor.py expects on port
9000::

    BAT1=3950 mV, BAT2=4012 mV\r\n

  * BAT1 -> Drive_CAN encoder memory cell (millivolts)
  * BAT2 -> Arm_CAN  encoder memory cell (millivolts)

battery_monitor.py parses it, publishes /drive_memory_battery_* and
/arm_memory_battery_* (latched), and telemetry_udp_bridge.py forwards the
voltages + ok flags to the dashboard. The low-voltage threshold is 3300 mV
(>= is OK, < is "low" -> that bus gets disarmed and the app flags it).

USAGE
-----
Steady values (defaults: BAT1=3950, BAT2=4012, every 2 s, to 127.0.0.1:9000):
    python3 fake_memory_battery.py

Point it at the ROS host running battery_monitor:
    python3 fake_memory_battery.py --host 192.168.144.50

Pick exact voltages (millivolts):
    python3 fake_memory_battery.py --bat1 3250 --bat2 4010
        # BAT1 below 3300 -> drive_mem_ok flips to false

Send one datagram and exit:
    python3 fake_memory_battery.py --once

Drain mode: ramp both cells from --drain-from down to --drain-to over
--drain-secs, so you can watch them cross the 3300 mV threshold live:
    python3 fake_memory_battery.py --drain-from 4100 --drain-to 3000 --drain-secs 60

Stop with Ctrl-C.
"""

import argparse
import socket
import sys
import time


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Fake STM32 encoder memory-battery UDP sender "
                    "(mimics the BAT1/BAT2 datagram for battery_monitor.py).")
    p.add_argument("--host", default="127.0.0.1",
                   help="destination IP where battery_monitor listens "
                        "(default: 127.0.0.1)")
    p.add_argument("--port", type=int, default=9000,
                   help="destination UDP port (default: 9000)")
    p.add_argument("--interval", type=float, default=2.0,
                   help="seconds between datagrams (default: 2.0; the real "
                        "STM32 uses ~30)")
    p.add_argument("--once", action="store_true",
                   help="send a single datagram and exit")

    # Steady values.
    p.add_argument("--bat1", type=int, default=3950,
                   help="BAT1 (Drive_CAN) millivolts in steady mode "
                        "(default: 3950)")
    p.add_argument("--bat2", type=int, default=4012,
                   help="BAT2 (Arm_CAN) millivolts in steady mode "
                        "(default: 4012)")

    # Drain mode (overrides steady values when --drain-from is given).
    p.add_argument("--drain-from", type=int, default=None,
                   help="start millivolts for both cells (enables drain mode)")
    p.add_argument("--drain-to", type=int, default=3000,
                   help="end millivolts for drain mode (default: 3000)")
    p.add_argument("--drain-secs", type=float, default=60.0,
                   help="seconds to ramp from --drain-from to --drain-to "
                        "(default: 60), then it holds at --drain-to")
    return p.parse_args(argv)


def make_payload(bat1_mv, bat2_mv):
    """Exact wire format battery_monitor._BATT_RE matches."""
    return f"BAT1={bat1_mv} mV, BAT2={bat2_mv} mV\r\n".encode("ascii")


def drained_mv(start, end, secs, elapsed):
    """Linear ramp from start to end over `secs`, clamped at the ends."""
    if secs <= 0 or elapsed >= secs:
        return end
    frac = elapsed / secs
    return int(round(start + (end - start) * frac))


def main(argv=None):
    args = parse_args(argv)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    drain = args.drain_from is not None

    mode = (f"drain {args.drain_from}->{args.drain_to} mV over {args.drain_secs}s"
            if drain else f"steady BAT1={args.bat1} BAT2={args.bat2} mV")
    print(f"fake_memory_battery -> {args.host}:{args.port}  "
          f"({mode}, every {args.interval}s, threshold 3000 mV)  Ctrl-C to stop",
          flush=True)

    t0 = time.monotonic()
    try:
        while True:
            if drain:
                v = drained_mv(args.drain_from, args.drain_to,
                               args.drain_secs, time.monotonic() - t0)
                bat1_mv, bat2_mv = v, v
            else:
                bat1_mv, bat2_mv = args.bat1, args.bat2

            payload = make_payload(bat1_mv, bat2_mv)
            try:
                sock.sendto(payload, dest)
            except OSError as e:
                print(f"send failed: {e}", file=sys.stderr, flush=True)

            d_ok = "OK" if bat1_mv >= 3000 else "LOW"
            a_ok = "OK" if bat2_mv >= 3000 else "LOW"
            print(f"sent  BAT1(drive)={bat1_mv} mV [{d_ok}]  "
                  f"BAT2(arm)={bat2_mv} mV [{a_ok}]", flush=True)

            if args.once:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
