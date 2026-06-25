"""
conversions.py
==============
Standalone unit-conversion and clamping helpers for CiA-402 motor control.

Every function is pure (no side-effects) and takes only the numeric
parameters it needs, so callers without a MotorConfig can still convert.
"""

import math

TWO_PI = 2.0 * math.pi


# ── position ────────────────────────────────────────────────────────────

def rad_to_counts(q_rad: float, counts_per_output_rev: float) -> int:
    """Radians → encoder counts (rounded to nearest int)."""
    return int(round(q_rad * counts_per_output_rev / TWO_PI))


def counts_to_rad(cnt: float, counts_per_output_rev: float) -> float:
    """Encoder counts → radians."""
    return cnt * TWO_PI / counts_per_output_rev


# ── velocity ────────────────────────────────────────────────────────────

def rad_per_s_to_can_vel(w_rad_s: float, gear_ratio: float) -> int:
    """Angular velocity [rad/s] → CANopen velocity counts."""
    return int(round(w_rad_s * gear_ratio * 100.0 / TWO_PI))


def can_vel_to_rad_per_s(v_can: float, gear_ratio: float) -> float:
    """CANopen velocity counts → angular velocity [rad/s]."""
    return v_can * TWO_PI / (gear_ratio * 100.0)


# ── clamping ────────────────────────────────────────────────────────────

def clamp(value: int, lo: int, hi: int) -> int:
    """Clamp *value* to [lo, hi] (integers)."""
    return max(int(lo), min(int(hi), int(value)))
