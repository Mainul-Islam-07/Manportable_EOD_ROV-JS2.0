"""test_arm_ik.py — Unit tests for the closed-form IK solver.

Strategy:
  - Round-trip: pick joint values, run FK to get wrist position, run IK,
    verify joints match within tolerance.
  - Edge cases: dead zone, unreachable far/close, wrist_pan wrapping,
    telescope clamping.
  - Constants sanity: verify URDF-derived constants in arm_ik match those
    computed by FK at all-zero pose.
"""

import math
import unittest

import numpy as np

import arm_ik
from fk import fk, pos


# -----------------------------------------------------------------------------
# FK helpers used by tests
# -----------------------------------------------------------------------------

def fk_wrist_center(turret_q, shoulder_q, elbow_q, telescope_q):
    """Return (x, y, z) of wrist_pan_Joint origin in base frame."""
    F = fk(turret_q, shoulder_q, elbow_q, telescope_q, 0.0, 0.0)
    return tuple(pos(F, 'wrist_pan'))


def fk_pitch_world(turret_q, shoulder_q, elbow_q, telescope_q,
                   wrist_pan_q, wrist_roll_q):
    """World-elevation pitch: angle of wrist_roll Y axis above horizontal.

    Turret-invariant by construction.  Returns value in [-π/2, π/2].
    """
    F = fk(turret_q, shoulder_q, elbow_q, telescope_q, wrist_pan_q, wrist_roll_q)
    R = F['wrist_roll'][:3, :3]
    y_world = R[:, 1]
    horizontal_mag = math.sqrt(y_world[0] ** 2 + y_world[1] ** 2)
    return math.atan2(y_world[2], horizontal_mag)


# -----------------------------------------------------------------------------
# Tolerances
# -----------------------------------------------------------------------------

POS_TOL  = 1e-4            # 0.1 mm — IK is now exact, modulo floating-point
ANG_TOL  = 1e-2            # 0.01 rad — round-trip joint match


def angle_diff(a, b):
    """Smallest signed angle from a to b, wrapped into [-π, π]."""
    return ((b - a + math.pi) % (2 * math.pi)) - math.pi


# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------

class TestConstants(unittest.TestCase):
    """Verify URDF constants in arm_ik.py match FK at zero pose."""

    def test_shoulder_z(self):
        F = fk(0, 0, 0, 0, 0, 0)
        self.assertAlmostEqual(pos(F, 'shoulder')[2], arm_ik.SHOULDER_Z, places=4)

    def test_l_upper(self):
        F = fk(0, 0, 0, 0, 0, 0)
        d = np.linalg.norm(pos(F, 'elbow') - pos(F, 'shoulder'))
        self.assertAlmostEqual(d, arm_ik.L_UPPER, places=4)

    def test_l_base(self):
        F = fk(0, 0, 0, 0, 0, 0)
        d = np.linalg.norm(pos(F, 'wrist_pan') - pos(F, 'elbow'))
        self.assertAlmostEqual(d, arm_ik.L_BASE, places=4)

    def test_elbow_offset_value(self):
        # Empirical calibration value, verified by FK across grid in calibrate.py
        self.assertAlmostEqual(arm_ik.ELBOW_OFFSET, 0.32103, places=4)


class TestRoundTrip(unittest.TestCase):
    """Pick joints, FK to wrist position, IK back, compare."""

    def _roundtrip(self, turret_q, shoulder_q, elbow_q,
                   telescope_q=0.0, wrist_pan_q=0.0, wrist_roll_q=0.0,
                   tag='', check_joints=True):
        """Round-trip via position recovery.

        FK on input joints → target W.  IK on W → joints'.  FK on joints' → W'.
        Verify |W' - W| < POS_TOL.  Now that the IK uses exact telescope-
        dependent forearm geometry, the same tight tolerance applies at
        all telescope values.
        """
        x, y, z = fk_wrist_center(turret_q, shoulder_q, elbow_q, telescope_q)
        pitch = fk_pitch_world(turret_q, shoulder_q, elbow_q, telescope_q,
                               wrist_pan_q, wrist_roll_q)

        result = arm_ik.solve(
            x, y, z,
            telescope=telescope_q,
            pitch_world=pitch,
            roll_world=wrist_roll_q,
            current_turret=turret_q,
        )
        self.assertTrue(result.success,
                        f'{tag}: IK failed: {result.failure_reason}')
        j = result.joints

        # Position recovery
        x2, y2, z2 = fk_wrist_center(
            j['turret_Joint'], j['shoulder_Joint'], j['elbow_Joint'],
            j['telescope_Joint'])
        err = math.sqrt((x2-x)**2 + (y2-y)**2 + (z2-z)**2)
        self.assertLess(err, POS_TOL,
                        f'{tag}: position error {err*1000:.3f} mm')

        # Pitch recovery
        pitch2 = fk_pitch_world(j['turret_Joint'], j['shoulder_Joint'],
                                j['elbow_Joint'], j['telescope_Joint'],
                                j['wrist_pan_Joint'], j['wrist_roll_Joint'])
        d_pitch = abs(angle_diff(pitch2, pitch))
        self.assertLess(d_pitch, ANG_TOL,
                        f'{tag}: pitch error {d_pitch:.4f} rad')

        # Joint match.
        if check_joints:
            for jn, expected in [('turret_Joint',    turret_q),
                                 ('shoulder_Joint',  shoulder_q),
                                 ('elbow_Joint',     elbow_q),
                                 ('telescope_Joint', telescope_q),
                                 ('wrist_pan_Joint', wrist_pan_q),
                                 ('wrist_roll_Joint',wrist_roll_q)]:
                got = j[jn]
                d = abs(angle_diff(got, expected))
                self.assertLess(d, ANG_TOL,
                    f'{tag}: {jn} = {got:.4f}, expected {expected:.4f} (Δ={d:.4f})')

    def test_extended_forward(self):
        # Max elbow-up reach: vertex_angle = π → elbow_q = π - ELBOW_OFFSET ≈ 2.82.
        # Use a comfortable margin inside that.
        self._roundtrip(turret_q=0.0, shoulder_q=-math.pi+0.01, elbow_q=2.7,
                        tag='extended_forward')

    def test_extended_right(self):
        self._roundtrip(turret_q=math.pi/2, shoulder_q=-math.pi+0.01, elbow_q=2.7,
                        tag='extended_right')

    def test_extended_left(self):
        self._roundtrip(turret_q=-math.pi/2, shoulder_q=-math.pi+0.01, elbow_q=2.7,
                        tag='extended_left')

    def test_natural_reach(self):
        # Image 4: shoulder=-2.41, elbow=1.748 (operator photo)
        self._roundtrip(turret_q=0.0, shoulder_q=-2.41, elbow_q=1.748,
                        wrist_pan_q=-0.764, tag='natural_reach')

    def test_half_extended(self):
        self._roundtrip(turret_q=0.5, shoulder_q=-math.pi/2, elbow_q=math.pi/2,
                        tag='half_extended')

    def test_with_telescope(self):
        self._roundtrip(turret_q=0.3, shoulder_q=-2.0, elbow_q=1.5,
                        telescope_q=-0.15, tag='with_telescope')

    def test_with_full_telescope(self):
        self._roundtrip(turret_q=-0.4, shoulder_q=-1.8, elbow_q=2.3,
                        telescope_q=-0.33982, tag='full_telescope')

    def test_wrist_pan_active(self):
        self._roundtrip(turret_q=0.6, shoulder_q=-1.5, elbow_q=1.2,
                        wrist_pan_q=0.7, tag='wrist_pan_active')

    def test_wrist_roll_passthrough(self):
        self._roundtrip(turret_q=0.0, shoulder_q=-2.0, elbow_q=1.5,
                        wrist_roll_q=1.2, tag='wrist_roll_passthrough')

    def test_grid_workspace(self):
        """Sweep joint space; every reachable elbow-up config should round-trip.

        Filters: skip configs where (a) the wrist falls inside the dead-zone
        (turret hold loses XY direction information) or (b) the wrist ends
        up behind the turret axis in arm-local frame (IK can't distinguish
        these from forward-side mirror configurations).  Both are expected
        IK limitations, not test failures.
        """
        failures = []
        n_total = 0
        n_skipped = 0
        for tu in [-2.0, -0.5, 0.0, 0.7, 2.5]:
            for sh in [-2.8, -2.0, -1.0, -0.3]:
                for el in [0.3, 1.0, 1.8, 2.6]:
                    for tele in [0.0, -0.10, -0.20, -0.33982]:
                        wx, wy, wz = fk_wrist_center(tu, sh, el, tele)
                        r = math.sqrt(wx * wx + wy * wy)
                        arm_local_y = math.sin(tu) * wx + math.cos(tu) * wy
                        if r < arm_ik.TURRET_SCALE_R + 0.005:
                            # Smooth scaling regime: the wrist sits at the
                            # projection of the target onto the slewed turret
                            # heading rather than at the exact target, so the
                            # round-trip won't be exact.  Skip.
                            n_skipped += 1
                            continue
                        if arm_local_y <= 0:
                            n_skipped += 1
                            continue
                        # World elevation has two solutions per pitch value
                        # (forward vs backward half-space); IK always returns
                        # the forward one.  Skip backward inputs.
                        x_sum = sh + el  # wp=0 default
                        if math.cos(x_sum) <= 0:
                            n_skipped += 1
                            continue

                        n_total += 1
                        try:
                            self._roundtrip(turret_q=tu, shoulder_q=sh,
                                            elbow_q=el, telescope_q=tele,
                                            tag=f'grid({tu},{sh},{el},{tele})')
                        except AssertionError as e:
                            failures.append(str(e))
        if failures:
            self.fail(f'{len(failures)}/{n_total} grid points failed '
                      f'({n_skipped} non-reachable skipped):\n' +
                      '\n'.join(failures[:5]))


class TestTurretBehaviour(unittest.TestCase):
    """Turret algorithm: small dead-zone for r=0 singularity only,
    config-aware desired direction, rate-limited slew."""

    def test_singularity_holds_turret(self):
        # Target on the base axis (r=0) — true singularity, hold turret
        result = arm_ik.solve(
            x=0.0, y=0.0, z=0.5,
            telescope=0.0, pitch_world=0.0, roll_world=0.0,
            current_turret=1.234,
        )
        self.assertTrue(result.success, result.failure_reason)
        self.assertAlmostEqual(result.joints['turret_Joint'], 1.234, places=6)

    def test_below_scale_radius_partial_motion(self):
        # r = sqrt(0.002^2 + 0.002^2) = 0.0028 << TURRET_SCALE_R (0.080).
        # Smooth scaling: factor = r/SCALE_R = 0.035, so the turret moves
        # only ~3.5% of the full atan2 diff per tick.  Assert the motion is
        # SMALL but NOT zero — that's the new contract (no dead spot at
        # home, but per-tick swing scales to zero with r).
        result = arm_ik.solve(
            x=0.002, y=0.002, z=0.5,
            telescope=0.0, pitch_world=0.0, roll_world=0.0,
            current_turret=0.5,
        )
        self.assertTrue(result.success, result.failure_reason)
        delta = abs(result.joints['turret_Joint'] - 0.5)
        # Full atan2 from current=0.5 would slew clamp at 0.30; smooth scaling
        # caps the per-tick step at TURRET_MAX_STEP * factor ≈ 0.011.
        self.assertLess(delta, 0.02,
                        f'per-tick step at r=0.003 should be tiny, got {delta:.4f}')

    def test_outside_singularity_tracks_atan2(self):
        # Forward target outside singularity zone; current matches → no slew
        result = arm_ik.solve(
            x=0.0, y=0.5, z=0.5,
            telescope=0.0, pitch_world=0.0, roll_world=0.0,
            current_turret=0.0,
        )
        self.assertTrue(result.success, result.failure_reason)
        self.assertAlmostEqual(result.joints['turret_Joint'], 0.0, places=4)

    def test_slew_rate_limited(self):
        # Target far from current direction → turret slews by max step only
        result = arm_ik.solve(
            x=0.5, y=0.0, z=0.5,             # desired (forward) = π/2 ≈ 1.57
            telescope=0.0, pitch_world=0.0, roll_world=0.0,
            current_turret=0.0,
        )
        self.assertTrue(result.success, result.failure_reason)
        # Should be exactly TURRET_MAX_STEP, not π/2
        self.assertAlmostEqual(result.joints['turret_Joint'],
                               arm_ik.TURRET_MAX_STEP, places=4)
        self.assertTrue(result.diagnostics['turret_slewing'])

    def test_folded_config_at_home(self):
        # Wrist at y=-0.018 (URDF home) — folded config: desired = atan2 + π = 0
        result = arm_ik.solve(
            x=0.0, y=-0.018, z=0.393,
            telescope=0.0, pitch_world=0.0, roll_world=0.0,
            current_turret=0.0,
        )
        self.assertTrue(result.success, result.failure_reason)
        # No slew needed: desired = current
        self.assertAlmostEqual(result.joints['turret_Joint'], 0.0, places=4)
        self.assertEqual(result.diagnostics['arm_config'], 'folded')
        self.assertFalse(result.diagnostics['turret_slewing'])

    def test_x_partial_response_at_home(self):
        # New contract under smooth scaling: at home (r_xy ≈ 0.019), a +x
        # command produces SMALL but non-zero turret motion.  Per-tick step
        # is bounded by TURRET_MAX_STEP * (r_xy / TURRET_SCALE_R) ≈ 0.072,
        # which keeps the elbow swing under ~2.5 cm per tick — vs. the ~9 cm
        # of unscaled atan2 tracking.  X is no longer absorbed (no dead
        # spot at home), but the per-tick gain is reduced.
        import math
        result = arm_ik.solve(
            x=0.005, y=-0.018, z=0.393,
            telescope=0.0, pitch_world=0.0, roll_world=0.0,
            current_turret=0.0,
        )
        self.assertTrue(result.success, result.failure_reason)
        t = result.joints['turret_Joint']
        # Some response (no dead spot)
        self.assertGreater(abs(t), 0.01,
                           f'turret should respond at home, got {t}')
        # ... but bounded well under the unscaled 0.272 (full atan2)
        self.assertLess(abs(t), 0.15,
                        f'turret swing should be damped, got {t}')

    def test_x_tracks_outside_scale_radius(self):
        # Once the wrist is out of the singularity region (r_xy >
        # TURRET_SCALE_R), x commands must produce full atan2 tracking.
        from fk import fk, pos
        # y = +0.100 with x = 0.005 gives r_xy ≈ 0.1001 > TURRET_SCALE_R.
        result = arm_ik.solve(
            x=0.005, y=0.100, z=0.393,
            telescope=0.0, pitch_world=0.0, roll_world=0.0,
            current_turret=0.0,
        )
        self.assertTrue(result.success, result.failure_reason)
        # Turret should be non-zero now (atan2 branch is in play).
        self.assertGreater(abs(result.joints['turret_Joint']), 0.01)

    def test_dead_zone_exit_no_jump_rejection(self):
        # User's failure scenario: previous goal (0.039, 0.026, 0.419) held in
        # dead zone (old r=0.05); next goal (0.044, 0.026, 0.419) exits and
        # would have triggered a 1.04 rad turret jump → delta rejection.
        # New algorithm: rate-limited slew, no jump.
        result = arm_ik.solve(
            x=0.044, y=0.026, z=0.419,
            telescope=0.0, pitch_world=0.0, roll_world=0.0,
            current_turret=-0.001,
        )
        self.assertTrue(result.success, result.failure_reason)
        delta = abs(result.joints['turret_Joint'] - (-0.001))
        self.assertLessEqual(delta, arm_ik.TURRET_MAX_STEP + 1e-6,
                             f'turret delta {delta:.4f} exceeds rate limit')


class TestDeadZone(TestTurretBehaviour):
    """Backwards-compat alias — old test name stays runnable."""


class TestUnreachable(unittest.TestCase):
    """Targets outside the workspace should fail with the right reason."""

    def test_too_far(self):
        # Max reach with telescope=0 is L_upper + L_base ≈ 0.690
        result = arm_ik.solve(
            x=0.0, y=1.5, z=0.4,
            telescope=0.0, pitch_world=-math.pi, roll_world=0.0,
            current_turret=0.0,
        )
        self.assertFalse(result.success)
        self.assertEqual(result.failure_reason, 'unreachable_far')

    def test_too_close(self):
        # Inside |L_upper - L_forearm|.  At telescope = -0.34 (full ext),
        # |L_upper - L_forearm| = |0.345 - 0.685| = 0.340.  A target very close
        # to the shoulder pivot with full telescope is unreachable.
        result = arm_ik.solve(
            x=0.05, y=0.05, z=arm_ik.SHOULDER_Z + 0.05,
            telescope=-0.33982, pitch_world=-math.pi, roll_world=0.0,
            current_turret=0.0,
        )
        self.assertFalse(result.success)
        self.assertEqual(result.failure_reason, 'unreachable_close')


class TestTelescopeClamping(unittest.TestCase):
    """Operator over-commanding telescope should clamp, not fail."""

    def test_clamp_overextend(self):
        # Pass telescope = -0.5 (beyond URDF -0.33982)
        result = arm_ik.solve(
            x=0.0, y=0.5, z=0.4,
            telescope=-0.5, pitch_world=-math.pi, roll_world=0.0,
            current_turret=0.0,
        )
        # Inside the IK, telescope is clamped before forearm-length calc.
        # With telescope = -0.5 → |t|=0.5, L_forearm = 0.345+0.5=0.845.
        # Target 0.5m is well within that, so this would normally succeed.
        # But clamping inside the function uses telescope=-0.5 raw for the
        # math and then clamps for output.  Let's check the output is clamped.
        if result.success:
            self.assertGreaterEqual(result.joints['telescope_Joint'], -0.33982)
            self.assertLessEqual(result.joints['telescope_Joint'], 0.0)

    def test_clamp_overretract(self):
        result = arm_ik.solve(
            x=0.0, y=0.4, z=0.4,
            telescope=0.5, pitch_world=-math.pi, roll_world=0.0,
            current_turret=0.0,
        )
        if result.success:
            self.assertGreaterEqual(result.joints['telescope_Joint'], -0.33982)
            self.assertLessEqual(result.joints['telescope_Joint'], 0.0)


class TestWrapping(unittest.TestCase):
    """Wrist_pan and wrist_roll should wrap into [-π, π)."""

    def test_pan_wraps(self):
        # Pick joints such that raw_pan = pitch - shoulder - elbow + π
        # exceeds π, forcing a wrap.
        # shoulder = -3.0, elbow = 0.1, pitch_world = π - 0.1 (gripper near up)
        # raw_pan = (π-0.1) - (-3.0) - 0.1 - (-π) ≈ 2π + 2.85 → wraps to 2.85
        result = arm_ik.solve(
            x=0.6, y=0.0, z=0.5,
            telescope=0.0, pitch_world=math.pi - 0.1, roll_world=0.0,
            current_turret=math.pi/2,
        )
        # Whether or not this specific target is reachable, if it succeeds
        # the wrist_pan must be inside ±π.
        if result.success:
            self.assertGreaterEqual(result.joints['wrist_pan_Joint'], -math.pi)
            self.assertLessEqual(result.joints['wrist_pan_Joint'], math.pi)

    def test_roll_wraps(self):
        result = arm_ik.solve(
            x=0.0, y=0.5, z=0.4,
            telescope=0.0, pitch_world=-math.pi, roll_world=5.0,
            current_turret=0.0,
        )
        if result.success:
            self.assertGreaterEqual(result.joints['wrist_roll_Joint'], -math.pi)
            self.assertLessEqual(result.joints['wrist_roll_Joint'], math.pi)
            # 5.0 wraps to 5.0 - 2π ≈ -1.28
            self.assertAlmostEqual(result.joints['wrist_roll_Joint'],
                                   5.0 - 2 * math.pi, places=4)


class TestDiagnostics(unittest.TestCase):
    """Confirm IKResult.diagnostics is populated for both success and failure
    cases, and contains the keys the coordinator's failure logger expects."""

    SUCCESS_KEYS = {
        'input_x', 'input_y', 'input_z', 'input_telescope',
        'input_pitch_world', 'input_roll_world', 'input_current_turret',
        'r', 'scale_factor', 'max_step_effective', 'turret_q', 'turret_in_range',
        'y_local', 'x_local',
        'para', 'L_forearm', 'elbow_offset_t', 'h', 'D', 'D_min', 'D_max',
        'cos_vertex', 'vertex_angle',
        'elbow_q_raw', 'elbow_q_final', 'elbow_clipped',
        'elbow_q_min', 'elbow_q_max', 'vertex_angle_used',
        'alpha', 'psi', 'beta',
        'shoulder_q_raw', 'shoulder_q_final', 'shoulder_clipped',
        'shoulder_q_min', 'shoulder_q_max',
        'raw_pan', 'wrist_pan_q', 'wrist_pan_in_range',
        'telescope_q', 'wrist_roll_q',
    }

    def test_success_populates_diagnostics(self):
        r = arm_ik.solve(0.0, 0.4, 0.4, 0, 0, 0, current_turret=0)
        self.assertTrue(r.success)
        self.assertIsNotNone(r.diagnostics)
        missing = self.SUCCESS_KEYS - set(r.diagnostics.keys())
        self.assertFalse(missing, f'Missing diagnostic keys on success: {missing}')

    def test_solver_failure_populates_diagnostics(self):
        # Far unreachable target
        r = arm_ik.solve(0.0, 1.5, 0.4, 0, 0, 0, current_turret=0)
        self.assertFalse(r.success)
        self.assertEqual(r.failure_reason, 'unreachable_far')
        self.assertIsNotNone(r.diagnostics)
        # Should reach at least the triangle stage
        for k in ('input_x', 'r', 'turret_q', 'y_local', 'h', 'cos_vertex'):
            self.assertIn(k, r.diagnostics, f'Missing key on far-unreachable: {k}')

    def test_elbow_limit_records_clip_status(self):
        # Target requiring elbow well below 0 — past tolerance
        r = arm_ik.solve(0.0, -0.018, 0.388, 0, 0, 0, current_turret=0)
        self.assertFalse(r.success)
        self.assertEqual(r.failure_reason, 'elbow_limit')
        self.assertIn('elbow_clipped', r.diagnostics)
        self.assertFalse(r.diagnostics['elbow_clipped'])
        self.assertIn('elbow_q_raw', r.diagnostics)
        self.assertLess(r.diagnostics['elbow_q_raw'], 0)

    def test_boundary_tolerance_clips_elbow(self):
        # At rounded home, raw elbow_q is slightly negative — clip should fire
        r = arm_ik.solve(0.0, -0.018, 0.393, 0, 0, 0, current_turret=0)
        self.assertTrue(r.success, f'home should clip + succeed: {r.failure_reason}')
        self.assertTrue(r.diagnostics['elbow_clipped'])
        self.assertEqual(r.joints['elbow_Joint'], 0.0)


# -----------------------------------------------------------------------------
# Run
# -----------------------------------------------------------------------------

if __name__ == '__main__':
    unittest.main(verbosity=2)
