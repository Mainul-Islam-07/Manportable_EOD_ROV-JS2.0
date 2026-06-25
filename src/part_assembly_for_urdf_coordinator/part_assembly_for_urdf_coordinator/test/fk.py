"""
Forward kinematics walker for the part_assembly_for_urdf chain.
Extracts the constants needed for closed-form 3-DOF IK.

Convention reminder:
  World frame: Z up, Y forward/back, X left/right.
  IK target is the wrist center = wrist_pan_Joint origin in base frame.
"""
import numpy as np

def rpy_to_R(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

def T(xyz, rpy):
    M = np.eye(4)
    M[:3,:3] = rpy_to_R(*rpy)
    M[:3,3]  = xyz
    return M

def Tjoint(xyz, rpy, axis, q, prismatic=False):
    """Joint transform = origin transform * motion transform."""
    M = T(xyz, rpy)
    if prismatic:
        d = np.eye(4); d[:3,3] = np.array(axis) * q
        return M @ d
    else:
        ax = np.array(axis, dtype=float); ax /= np.linalg.norm(ax)
        c, s = np.cos(q), np.sin(q); v = 1-c
        x,y,z = ax
        R = np.array([
            [c+x*x*v,    x*y*v - z*s, x*z*v + y*s],
            [y*x*v+z*s,  c+y*y*v,     y*z*v - x*s],
            [z*x*v-y*s,  z*y*v + x*s, c+z*z*v   ],
        ])
        Mr = np.eye(4); Mr[:3,:3] = R
        return M @ Mr

# ---- chain definition (from URDF) -------------------------------------------
# (xyz, rpy, axis, prismatic?)
turret    = ([0,        0,        0.21647], [3.1416,  0, 0],     [0,0,1], False)
shoulder  = ([0,        0,       -0.068  ], [-3.1416, 0, 0],     [1,0,0], False)
elbow     = ([0,       -0.3452,   0      ], [-3.0892, 0, 0],     [1,0,0], False)
telescope = ([0,        0,       -0.0915 ], [ 0,      0, 0],     [0,1,0], True )
wrist_pan = ([0,       -0.33238,  0      ], [-0.05236,0, 0],     [1,0,0], False)
wrist_rl  = ([0.10485, -0.037,    0      ], [-3.1416, 0, 3.1416],[0,1,0], False)

def fk(turret_q, shoulder_q, elbow_q, tele_q, pan_q=0.0, roll_q=0.0):
    """Return list of frames base->turret, ...->wrist_roll."""
    M = np.eye(4); frames = {}
    M = M @ Tjoint(*turret[:3],    turret_q,    prismatic=False); frames['turret']    = M.copy()
    M = M @ Tjoint(*shoulder[:3],  shoulder_q,  prismatic=False); frames['shoulder']  = M.copy()
    M = M @ Tjoint(*elbow[:3],     elbow_q,     prismatic=False); frames['elbow']     = M.copy()
    M = M @ Tjoint(*telescope[:3], tele_q,      prismatic=True ); frames['telescope'] = M.copy()
    M = M @ Tjoint(*wrist_pan[:3], pan_q,       prismatic=False); frames['wrist_pan'] = M.copy()
    M = M @ Tjoint(*wrist_rl[:3],  roll_q,      prismatic=False); frames['wrist_roll']= M.copy()
    return frames

def pos(F, name): return F[name][:3,3]

# ---- 1) Where is each pivot at the all-zero pose? ---------------------------
F0 = fk(0,0,0,0,0,0)
print("=" * 72)
print("ALL-ZERO POSE — pivot positions in base_link frame  (X right, Y fwd, Z up)")
print("=" * 72)
for name in ['turret','shoulder','elbow','telescope','wrist_pan','wrist_roll']:
    p = pos(F0, name)
    print(f"  {name:11s}  X={p[0]:+.5f}  Y={p[1]:+.5f}  Z={p[2]:+.5f}")

# ---- 2) The shoulder pivot — this is what 'shoulder_height' refers to -------
sh = pos(F0,'shoulder')
print("\nSHOULDER PIVOT (origin of shoulder_Joint in base frame):")
print(f"  position in base frame = ({sh[0]:+.5f}, {sh[1]:+.5f}, {sh[2]:+.5f})")
print(f"  height above ground (Z)= {sh[2]:.5f} m")

# ---- 3) Link lengths used by the closed-form 2R IK --------------------------
# L_upper = distance from shoulder pivot to elbow pivot
# L_base  = distance from elbow pivot to wrist center when telescope = 0
sh_p   = pos(F0,'shoulder')
el_p   = pos(F0,'elbow')
wc0_p  = pos(F0,'wrist_pan')   # wrist center, telescope retracted
L_upper = np.linalg.norm(el_p - sh_p)
L_base  = np.linalg.norm(wc0_p - el_p)

print(f"\nLINK LENGTHS")
print(f"  L_upper = |shoulder -> elbow|              = {L_upper:.5f} m")
print(f"  L_base  = |elbow -> wrist (tele=0)|        = {L_base:.5f} m")

# ---- 4) Does telescope grow the forearm linearly?  Verify w/ real FK --------
print(f"\nFOREARM LENGTH vs TELESCOPE EXTENSION  (signed: tele_q is negative for extend)")
print(f"  {'tele_q':>10s}  {'|elbow->wc|':>13s}  {'expected L_base - tele_q':>26s}")
for tele_q in [0.0, -0.1, -0.2, -0.33982]:
    F = fk(0,0,0, tele_q, 0,0)
    wc = pos(F,'wrist_pan'); el = pos(F,'elbow')
    L = np.linalg.norm(wc - el)
    expected = L_base - tele_q   # tele_q < 0 means extended, so subtract
    print(f"  {tele_q:>+10.5f}  {L:>13.5f}  {expected:>26.5f}")

# ---- 5) Is the elbow->wrist vector collinear with the telescope axis? -------
# i.e. when telescope extends, does the wrist center move purely along
# the elbow-to-wrist line, or is there an angular kink?
F_a = fk(0,0,0, 0.0, 0,0)
F_b = fk(0,0,0, -0.2, 0,0)
v_a = pos(F_a,'wrist_pan') - pos(F_a,'elbow')
v_b = pos(F_b,'wrist_pan') - pos(F_b,'elbow')
v_a /= np.linalg.norm(v_a); v_b /= np.linalg.norm(v_b)
ang = np.degrees(np.arccos(np.clip(v_a @ v_b, -1, 1)))
print(f"\nTELESCOPE COLLINEARITY CHECK")
print(f"  angle between (elbow->wrist) at tele=0 and tele=-0.2  = {ang:.6f} deg")
print(f"  -> if ~0, forearm = L_base + |tele_q| is exact.")

# ---- 6) Pitch offset — what does the EE point at when all joints are 0? -----
# The EE pitch in the YZ plane (since Z is up, Y is fwd) is
#   pitch_world = atan2(forward_component_change, up_component_change)
# We compute it from how the wrist_roll_Link Y axis points in the base frame.
# But for our purposes the relevant quantity is:
#   end_effector_pitch_world = shoulder + elbow + wrist_pan + offset
# where "offset" is everything baked in from URDF rpy values.
# At all-zero pose, shoulder=elbow=wrist_pan=0, so end_effector_pitch_world = offset.

# We define "EE pitch" as the angle of the wrist_roll axis (the long axis of the
# end effector) measured in the YZ plane.  We use the wrist_roll Z axis after the
# joint chain — that's the gripper-pointing direction.
def ee_pitch_world(F):
    """Angle of the gripper-pointing direction in the YZ plane (Z up, Y fwd).
    Returns radians.  0 = pointing along +Y (forward, level).  +pi/2 = up."""
    # The gripper points along the Z axis of gripper_Link, which after the
    # gripper_Joint rpy is along the Y axis of wrist_roll_Link.  So we read
    # the Y column of wrist_roll's rotation matrix in the base frame.
    R = F['wrist_roll'][:3,:3]
    y_axis_world = R[:,1]   # forward direction of the EE in base frame
    return np.arctan2(y_axis_world[2], y_axis_world[1])

p0 = ee_pitch_world(F0)
print(f"\nPITCH OFFSET at all-joints-zero pose")
print(f"  EE 'pointing' angle in YZ-plane = {np.degrees(p0):+.3f} deg  ({p0:+.5f} rad)")
print(f"  This is the constant 'offset' in:")
print(f"    EE_pitch_world = shoulder + elbow + wrist_pan + offset")

# ---- 7) Verify the additive pitch model -------------------------------------
print(f"\nADDITIVE PITCH VERIFICATION")
print(f"  {'shoulder':>10s} {'elbow':>8s} {'pan':>8s}  {'predicted':>12s}  {'actual FK':>12s}  {'err':>10s}")
for sh_q, el_q, pn_q in [(0,0,0), (-0.5,0.5,0), (-1.0,0.5,0.3), (-0.7,1.0,-0.4)]:
    F = fk(0, sh_q, el_q, 0, pn_q, 0)
    actual = ee_pitch_world(F)
    pred   = sh_q + el_q + pn_q + p0
    # wrap into (-pi,pi]
    err = (actual - pred + np.pi) % (2*np.pi) - np.pi
    print(f"  {sh_q:>+10.3f} {el_q:>+8.3f} {pn_q:>+8.3f}  {pred:>+12.5f}  {actual:>+12.5f}  {err:>+10.2e}")

# ---- 8) Workspace summary --------------------------------------------------
tele_min, tele_max = -0.33982, 0.0
L_min = L_upper - (L_base + abs(tele_min))   # if elbow folds back tightly
L_max = L_upper + (L_base + abs(tele_min))
L_at_tele_zero_max = L_upper + L_base
print(f"\nREACH ENVELOPE (radial distance from shoulder pivot)")
print(f"  forearm range          : [{L_base:.4f}, {L_base + abs(tele_min):.4f}] m")
print(f"  max reach (tele extend): {L_max:.4f} m  (arm fully extended along itself)")
print(f"  max reach (tele=0)     : {L_at_tele_zero_max:.4f} m")
print(f"  shoulder pivot at z    = {sh_p[2]:.4f} m above base_link origin")
