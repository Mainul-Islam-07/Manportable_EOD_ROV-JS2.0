# SbusControl.msg
# Interpreted output from the SBUS RC receiver.
#
# Conventions
# -----------
#   control_mode    : uint8  ARM=0  HOME=1  DRIVE=2
#                     CH1 3-state switch
#   operation_mode  : uint8  ARMED=0  FIRING=1  DISARMED=2
#                     CH2 3-state switch
#   3-state fields  : int8   -1 / 0 / +1
#   2-state fields  : uint8   0 / 1
#   Arm position    : integrated by the coordinator from arm_{x,y,z}_cmd deltas.
#   EE orientation  : ee_pitch int8 -1/0/+1 delta (coordinator accumulates)
#                     ee_roll  int8 -1/0/+1 (for external process)
#   Camera          : camera_axis selects pan(0) or tilt(1),
#                     camera_cmd  is the -1/0/+1 command for the selected axis

std_msgs/Header header

# Control mode  ARM=0  HOME=1  DRIVE=2
uint8 CONTROL_MODE_ARM   = 0
uint8 CONTROL_MODE_HOME  = 1
uint8 CONTROL_MODE_DRIVE = 2
uint8 control_mode

# Operation mode  ARMED=0  FIRING=1  DISARMED=2
uint8 OPERATION_MODE_ARMED    = 0
uint8 OPERATION_MODE_FIRING   = 1
uint8 OPERATION_MODE_DISARMED = 2
uint8 operation_mode

# Drive  (-100.0 to +100.0, signed % of drive_speed)
float64 drive_left
float64 drive_right
uint8   drive_speed

# Arm position commands (integrated by coordinator)
# -1 / 0 / +1 per axis
int8    arm_x_cmd
int8    arm_y_cmd
int8    arm_z_cmd

# End-effector orientation
# ee_roll:  int8 -1/0/+1 (raw, for external process)
# ee_pitch: int8 -1/0/+1 (delta, coordinator accumulates)
int8    ee_roll
int8    ee_pitch

# Claw command  (-100.0 to +100.0, scroll wheel)
float64 claw_cmd

# Telescope cmd  RETRACT=-1  HOLD=0  EXTEND=+1
int8    telescope_cmd

# Flippers (always active)  -1/0/+1
int8    rear_flipper
int8    front_flipper

# Light  0=off  1=on
uint8   light_state

# Camera  axis: 0=pan  1=tilt    cmd: -1/0/+1
uint8   camera_axis
int8    camera_cmd
