"""arm_ik.py — Closed-form 3-DOF inverse kinematics for the part_assembly arm.

Solves turret, shoulder, and elbow joint angles given a Cartesian wrist-center
target (x, y, z) in base_link frame. Computes wrist_pan from a desired world
pitch using an additive joint-angle model. Telescope and wrist_roll are
passed through from the operator (with clamping/wrapping).

Geometric conventions (verified against URDF via FK):
  - Base frame: +X right, +Y forward, +Z up
  - Shoulder pivot at (0, 0, 0.28447) in base frame
  - Upper arm length: 0.34520 m (shoulder pivot to elbow pivot)
  - Forearm length at telescope=0: 0.34474 m (elbow pivot to wrist center)
  - Forearm modelled exactly: in elbow_Link frame the wrist sits at
    (0, -(FOREARM_PARA + |telescope|), -FOREARM_PERP).  Length and the
    elbow-joint calibration offset are computed per call from telescope.
    No approximation error beyond floating-point.
  - Pitch model: world elevation of wrist_roll Y axis,
        pitch_world = atan2(y_world_z, sqrt(y_world_x² + y_world_y²))
                    = -(shoulder + elbow + wrist_pan)   (mod 2π)
    in the forward half-space pitch_world ∈ (-π/2, π/2).  Turret-invariant.
  - Turret formula: turret_q = atan2(x, y) outside dead-zone
  - Inside dead-zone (r < 0.05 m): turret held at current value

Joint conventions (URDF rpy values bake in flips):
  - shoulder_Joint ∈ [-π, 0]; shoulder=0 means upper arm hangs at -Y;
    shoulder=-π means upper arm points at +Y (extended forward)
  - elbow_Joint ∈ [0, π]; elbow=0 means arm folded back; elbow=π means
    forearm parallel to upper arm (fully extended)
  - wrist_pan_Joint ∈ [-π, π], wraps continuously
  - All elbow-up: math.acos always returns [0, π], matching URDF range
"""

import math
from dataclasses import dataclass
from typing import Optional


# -----------------------------------------------------------------------------
# URDF-derived constants (verified by FK in fk.py)
# -----------------------------------------------------------------------------

L_UPPER      = 0.34520     # shoulder pivot → elbow pivot, metres
SHOULDER_Z   = 0.28447     # shoulder pivot height in base frame, metres

# Turret behaviour
#
# Near the base axis the wrist sits on a tiny xy-circle, so atan2(x,±y) — the
# turret target the IK would prefer — varies wildly for tiny operator nudges:
# a 5 mm wrist motion can demand 60° of turret rotation, which means ~16 cm
# of elbow swing.  We don't want that.
#
# A hard dead zone (zero turret motion below some radius, full motion outside)
# trades the swing for a discontinuous boundary jump and zero X response at
# home, which is its own kind of bad.
#
# Instead we scale the turret slew SMOOTHLY: factor = min(1, r_xy/SCALE_R).
# The IK still picks a desired turret from atan2; we just damp the rate at
# which we approach it when r_xy is small.  The wrist endpoint then sits on
# the SIGNED PROJECTION of the user's target onto the slewed arm-local Y axis
# (computed below), which means:
#   - At r_xy >> SCALE_R: factor = 1, projection = ±r_xy, wrist tracks target.
#   - At r_xy = 0: factor = 0, turret held, wrist sits at the projection of
#     the target onto the held heading.  X commands still grow the goal and
#     therefore r_xy, so the next tick has a slightly larger factor → motion
#     ramps in smoothly with no boundary jump and no dead spot at home.
TURRET_SCALE_R  = 0.150    # m — radius at which turret tracks atan2 fully.
                           # Below this both the desired-current diff AND the
                           # per-tick slew limit are scaled by r_xy/SCALE_R, so
                           # the turret moves PROPORTIONALLY less per tick when
                           # the wrist is near the base axis.  The wrist
                           # endpoint sits at the projection of the target
                           # onto the slewed arm-local Y axis (computed below);
                           # this means the wrist tracks partially even at
                           # home (no zero-X dead spot) and the per-tick elbow
                           # swing scales to zero as r_xy -> 0.
                           #
                           # On the user's reported (0.022, ±0.08) Y sweep
                           # this value caps peak turret at ~34° (elbow swing
                           # ~19 cm); SCALE_R=0.080 was 49°/26 cm.  Tune UP
                           # toward 0.30 for tighter swing (more wrist
                           # tracking error in the small-r band) or DOWN
                           # toward 0.080 to recover faster X tracking.  Can
                           # be overridden at solve() call time.
TURRET_MAX_STEP = 0.30     # rad per IK call — turret slew rate limit so any
                           # transition stays within max_joint_delta.

# Forearm geometry in elbow_Link frame.  The wrist center sits at
#   (0, -(FOREARM_PARA + |telescope|), -FOREARM_PERP)
# i.e. at distance L_forearm(t) from the elbow pivot, at angle atan2(perp, para)
# from the elbow's natural -Y direction.  Both components are URDF-exact:
#   FOREARM_PARA: wrist_pan_Joint origin Y in telescope_Link frame
#   FOREARM_PERP: telescope_Joint origin Z in elbow_Link frame
FOREARM_PARA = 0.33238
FOREARM_PERP = 0.0915

# Deviation of elbow_Joint origin rpy_x from -π:  -3.0892 = -π + 0.05239
ELBOW_RPY_OFFSET = math.pi - 3.0892

# Convenience: forearm length and elbow-offset at telescope=0.
# These are no longer the values used at runtime — they're computed
# fresh per call from telescope — but they're kept for tests and
# documentation.
L_BASE       = math.sqrt(FOREARM_PARA ** 2 + FOREARM_PERP ** 2)  # ≈ 0.34474
ELBOW_OFFSET = ELBOW_RPY_OFFSET + math.atan2(FOREARM_PERP, FOREARM_PARA)  # ≈ 0.32103

LIMITS = {
    'turret_Joint':    (-math.pi, math.pi),  # URDF says ±3.14; widen to ±π so
                                             # atan2(0, neg) which returns ±π
                                             # exactly is still considered valid.
    'shoulder_Joint':  (-math.pi,    0.0 ),
    'elbow_Joint':     ( 0.0,     math.pi),
    'telescope_Joint': (-0.33982, 0.0 ),
    'wrist_pan_Joint': (-math.pi, math.pi),
    'wrist_roll_Joint':(-math.pi, math.pi),
}

ARM_JOINTS = [
    'turret_Joint',
    'shoulder_Joint',
    'elbow_Joint',
    'telescope_Joint',
    'wrist_pan_Joint',
    'wrist_roll_Joint',
]


# Boundary tolerances: how far past the URDF joint limit the IK is willing to
# clip rather than fail.  Helps the operator at workspace edges where floating
# point or rounding pushes a marginal solution slightly out of range.  Set to 0
# to fall back to strict (no-clip) behaviour.
ELBOW_BOUNDARY_TOL    = 0.01     # rad ~ 0.6° — clipped position error ≤ ~3 mm
SHOULDER_BOUNDARY_TOL = 0.01     # rad ~ 0.6° — clipped position error ≤ ~3 mm


# -----------------------------------------------------------------------------
# Result type
# -----------------------------------------------------------------------------

@dataclass
class IKResult:
    """Outcome of an IK solve.

    On success: `joints` is a dict with all six ARM_JOINTS keys.
    On failure: `failure_reason` identifies the cause; `joints` is None.
    `diagnostics` is always populated — see solve() for the full key list.
    The coordinator uses `diagnostics` to log detailed failure context.

    Failure reasons (stable strings — used in coordinator log output):
      'unreachable_far'    — target beyond shoulder + forearm length
      'unreachable_close'  — target inside |L_upper - L_forearm|
      'shoulder_limit'     — geometric solution beyond URDF + tolerance
      'elbow_limit'        — geometric solution beyond URDF + tolerance
      'pitch_unreachable'  — wrapped wrist_pan still outside URDF range
      'turret_limit'       — atan2 result outside URDF range
    """
    success: bool
    joints: Optional[dict] = None
    failure_reason: str = ''
    diagnostics: Optional[dict] = None


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def _wrap_pi(angle: float) -> float:
    """Wrap an angle into [-π, π).  Standard formulation."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _within(value: float, joint_name: str) -> bool:
    lo, hi = LIMITS[joint_name]
    return lo <= value <= hi


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

def solve(x: float, y: float, z: float,
          telescope: float,
          pitch_world: float,
          roll_world: float,
          current_turret: float,
          turret_scale_r: float = None) -> IKResult:
    """Solve closed-form IK for the 6-DOF arm.

    Parameters
    ----------
    x, y, z : float
        Target wrist-center position in base_link frame, metres.
    telescope : float
        Operator-commanded telescope value (URDF range [-0.33982, 0]).
    pitch_world : float
        Operator-commanded end-effector pitch in world frame, radians.
        Positive = gripper tilts toward +Z (toward sky).
    roll_world : float
        Operator-commanded end-effector roll, radians (passthrough).
    current_turret : float
        Current turret joint angle.  Used to hold turret inside dead-zone.
    turret_scale_r : float, optional
        Override for the singularity-region scaling radius.  If None,
        falls back to the module-level TURRET_SCALE_R.  Lets the coordinator
        plumb a YAML knob without re-importing/re-compiling the module.

    Returns
    -------
    IKResult — `diagnostics` dict is always populated and contains every
    intermediate value computed.  Use it to log failure context.
    """

    # Diagnostics dict — every stage populates this so a failure log can
    # show exactly where and why things went wrong.  Always returned.
    d = {
        'input_x': x, 'input_y': y, 'input_z': z,
        'input_telescope': telescope,
        'input_pitch_world': pitch_world,
        'input_roll_world': roll_world,
        'input_current_turret': current_turret,
    }

    # ----- Stage 1: Turret with config-aware tracking and rate limit ------
    #
    # The arm's natural "facing" direction depends on whether it's in a
    # forward-extended pose (shoulder near -π) or a folded-back pose (shoulder
    # near 0).  Both configurations can place the wrist at the same (x,y).
    # We pick the desired turret based on the projection of the target onto
    # the CURRENT arm-local +y axis:
    #
    #   y_local_at_current >= 0   ->  forward config; desired = atan2(x, y)
    #   y_local_at_current  < 0   ->  folded config;  desired = atan2(x, y) + π
    #
    # This keeps the IK consistent with the current arm pose and avoids
    # spurious 180° turret flips just because the wrist sits slightly behind
    # the turret axis (e.g. the URDF home pose at y=-0.018).
    #
    # The desired turret is then approached subject to TURRET_MAX_STEP so the
    # joint-delta safety check never trips.  Wrist sweeps an arc at constant
    # radial distance r_xy until turret converges.
    r_xy = math.sqrt(x * x + y * y)
    y_local_at_current = (math.sin(current_turret) * x +
                          math.cos(current_turret) * y)

    # ----- Pick atan2 branch from the CURRENT arm-local +Y projection --------
    # This avoids spurious 180° turret flips just because the wrist sits
    # slightly behind the turret axis (e.g. the URDF home pose at y=-0.018).
    # NOTE on the folded branch: URDF/FK says the correct formula is
    #   _wrap_pi(atan2(x, y) + pi)
    # which produces e.g. turret=-0.27 for +X target at home, and FK confirms
    # wrist arrives at +X.  But on real hardware the wrist appears to move in
    # -X for that turret value at home (folded), while forward config works.
    # Empirical fix below; revert to the wrap_pi form if hardware testing
    # shows this breaks anything.
    if y_local_at_current >= 0:
        desired_turret_atan2 = math.atan2(x, y)
    else:
        desired_turret_atan2 = math.atan2(x, -y)

    # ----- Smooth turret scaling near the singularity -----------------------
    # factor ramps linearly from 0 at r_xy=0 to 1 at r_xy=TURRET_SCALE_R.
    # We damp BOTH the desired-current diff AND the per-tick slew clamp by the
    # same factor, so the turret's per-tick angular displacement scales as
    # r_xy / SCALE_R inside the singular region.  The wrist endpoint then
    # ends up on the SIGNED PROJECTION of the target onto the slewed arm
    # heading (computed below) — partial X tracking everywhere, no boundary
    # jump, and elbow swing per tick proportional to r_xy.
    if turret_scale_r is None:
        turret_scale_r = TURRET_SCALE_R
    if turret_scale_r > 0.0:
        scale_factor = min(1.0, r_xy / turret_scale_r)
    else:
        scale_factor = 1.0

    turret_diff_full = _wrap_pi(desired_turret_atan2 - current_turret)
    turret_diff_scaled = turret_diff_full * scale_factor
    max_step_effective = TURRET_MAX_STEP * scale_factor
    turret_diff_clamped = max(-max_step_effective,
                              min(max_step_effective, turret_diff_scaled))
    turret_q = _wrap_pi(current_turret + turret_diff_clamped)

    # arm_config from y_local at the SLEWED turret (where the arm will be).
    # Using the slewed turret here keeps the elbow-up/down choice consistent
    # with where the wrist actually ends up, instead of based on a stale
    # current-pose reading that may be on the other side of zero.
    y_local_after = (math.sin(turret_q) * x + math.cos(turret_q) * y)
    arm_config = 'forward' if y_local_after >= 0 else 'folded'
    desired_turret = desired_turret_atan2  # for diagnostics

    d['r'] = r_xy
    d['scale_factor'] = scale_factor
    d['max_step_effective'] = max_step_effective
    d['y_local_at_current'] = y_local_at_current
    d['arm_config'] = arm_config
    d['desired_turret'] = desired_turret
    d['turret_diff'] = turret_diff_full
    d['turret_diff_clamped'] = turret_diff_clamped
    d['turret_slewing'] = (
        abs(turret_diff_full) > abs(turret_diff_clamped) + 1e-9)
    d['turret_q'] = turret_q
    d['turret_in_range'] = _within(turret_q, 'turret_Joint')
    if not d['turret_in_range']:
        return IKResult(False, failure_reason='turret_limit', diagnostics=d)

    # Project target into the arm-local frame at the (rate-limited) turret.
    y_local = math.sin(turret_q) * x + math.cos(turret_q) * y
    x_local = math.cos(turret_q) * x - math.sin(turret_q) * y
    d['y_local'] = y_local
    d['x_local'] = x_local

    # ----- Stage 2: 2R planar IK in arm-local plane ----------------------
    # Telescope-exact forearm geometry.
    para = FOREARM_PARA + abs(telescope)
    L_forearm = math.sqrt(para * para + FOREARM_PERP * FOREARM_PERP)
    elbow_offset_t = ELBOW_RPY_OFFSET + math.atan2(FOREARM_PERP, para)

    h = z - SHOULDER_Z

    # y_planar is the SIGNED PROJECTION of the target onto the SLEWED arm-local
    # +Y axis.  Single formula for both regimes:
    #   - Full tracking (scale_factor = 1): y_local = ±r_xy at the slewed
    #     turret, so the wrist arrives exactly at the user's target.
    #   - Partial tracking (scale_factor < 1): the slewed turret hasn't
    #     reached atan2_desired, so |y_local| < r_xy.  The wrist sits at the
    #     projection of the target onto the held heading — closest reachable
    #     point given the rate-limited turret.
    y_planar = y_local
    d_sq = y_planar * y_planar + h * h
    d_dist = math.sqrt(d_sq)

    d['para'] = para
    d['L_forearm'] = L_forearm
    d['elbow_offset_t'] = elbow_offset_t
    d['h'] = h
    d['y_planar'] = y_planar
    d['D'] = d_dist
    d['D_min'] = abs(L_UPPER - L_forearm)
    d['D_max'] = L_UPPER + L_forearm

    # Law of cosines.
    cos_vertex = (L_UPPER ** 2 + L_forearm ** 2 - d_sq) / (2.0 * L_UPPER * L_forearm)
    d['cos_vertex'] = cos_vertex
    if cos_vertex < -1.0:
        return IKResult(False, failure_reason='unreachable_far', diagnostics=d)
    if cos_vertex > 1.0:
        return IKResult(False, failure_reason='unreachable_close', diagnostics=d)
    vertex_angle = math.acos(cos_vertex)        # geometric, in [0, π]
    d['vertex_angle'] = vertex_angle

    # ----- Elbow with boundary tolerance ---------------------------------
    elbow_q_raw = vertex_angle - elbow_offset_t
    elbow_lo, elbow_hi = LIMITS['elbow_Joint']
    d['elbow_q_raw'] = elbow_q_raw
    d['elbow_q_min'] = elbow_lo
    d['elbow_q_max'] = elbow_hi

    if elbow_lo <= elbow_q_raw <= elbow_hi:
        elbow_q = elbow_q_raw
        d['elbow_clipped'] = False
    elif elbow_lo - ELBOW_BOUNDARY_TOL <= elbow_q_raw < elbow_lo:
        # Tiny negative — clip to lower limit, recompute vertex_angle so the
        # downstream shoulder math uses the achievable elbow position.
        elbow_q = elbow_lo
        vertex_angle = elbow_offset_t
        d['elbow_clipped'] = True
        d['elbow_clip_amount'] = elbow_lo - elbow_q_raw
    elif elbow_hi < elbow_q_raw <= elbow_hi + ELBOW_BOUNDARY_TOL:
        elbow_q = elbow_hi
        vertex_angle = elbow_hi + elbow_offset_t
        d['elbow_clipped'] = True
        d['elbow_clip_amount'] = elbow_q_raw - elbow_hi
    else:
        d['elbow_q_final'] = elbow_q_raw
        d['elbow_clipped'] = False
        return IKResult(False, failure_reason='elbow_limit', diagnostics=d)

    d['elbow_q_final'] = elbow_q
    d['vertex_angle_used'] = vertex_angle

    # ----- Shoulder via two-step atan2 -----------------------------------
    # alpha uses y_planar (signed by arm_config), not y_local.  This keeps
    # the elbow-up branch consistent with the chosen forward/folded config
    # even while turret is mid-slew (when y_local doesn't yet equal y_planar).
    alpha = math.atan2(h, y_planar)
    gamma = math.pi - vertex_angle
    psi = math.atan2(L_forearm * math.sin(gamma),
                     L_UPPER + L_forearm * math.cos(gamma))
    beta = alpha + psi
    shoulder_q_raw = beta - math.pi

    sh_lo, sh_hi = LIMITS['shoulder_Joint']
    d['alpha'] = alpha
    d['psi'] = psi
    d['beta'] = beta
    d['shoulder_q_raw'] = shoulder_q_raw
    d['shoulder_q_min'] = sh_lo
    d['shoulder_q_max'] = sh_hi

    if sh_lo <= shoulder_q_raw <= sh_hi:
        shoulder_q = shoulder_q_raw
        d['shoulder_clipped'] = False
    elif sh_lo - SHOULDER_BOUNDARY_TOL <= shoulder_q_raw < sh_lo:
        shoulder_q = sh_lo
        d['shoulder_clipped'] = True
        d['shoulder_clip_amount'] = sh_lo - shoulder_q_raw
    elif sh_hi < shoulder_q_raw <= sh_hi + SHOULDER_BOUNDARY_TOL:
        shoulder_q = sh_hi
        d['shoulder_clipped'] = True
        d['shoulder_clip_amount'] = shoulder_q_raw - sh_hi
    else:
        d['shoulder_q_final'] = shoulder_q_raw
        d['shoulder_clipped'] = False
        return IKResult(False, failure_reason='shoulder_limit', diagnostics=d)

    d['shoulder_q_final'] = shoulder_q

    # ----- Stage 3: Wrist pan from desired world pitch -------------------
    raw_pan = -pitch_world - shoulder_q - elbow_q
    wrist_pan_q = _wrap_pi(raw_pan)
    d['raw_pan'] = raw_pan
    d['wrist_pan_q'] = wrist_pan_q
    if not _within(wrist_pan_q, 'wrist_pan_Joint'):
        d['wrist_pan_in_range'] = False
        return IKResult(False, failure_reason='pitch_unreachable', diagnostics=d)
    d['wrist_pan_in_range'] = True

    # ----- Stage 4: Telescope and wrist_roll passthrough ------------------
    tlo, thi = LIMITS['telescope_Joint']
    telescope_q = max(tlo, min(thi, telescope))
    wrist_roll_q = _wrap_pi(roll_world)
    d['telescope_q'] = telescope_q
    d['wrist_roll_q'] = wrist_roll_q

    return IKResult(
        success=True,
        joints={
            'turret_Joint':     turret_q,
            'shoulder_Joint':   shoulder_q,
            'elbow_Joint':      elbow_q,
            'telescope_Joint':  telescope_q,
            'wrist_pan_Joint':  wrist_pan_q,
            'wrist_roll_Joint': wrist_roll_q,
        },
        diagnostics=d,
    )
