#!/usr/bin/env python3
"""
bringup_sequence.launch.py
==========================
Starts the full robot stack one process at a time, 3 s apart, in order:

    sim_headless -> sbus -> robot -> light -> fire -> mode
    -> diagnostics -> bms -> coordinator

Each entry mirrors the matching alias in ~/.bashrc. Processes are STARTED 3 s
apart (TimerAction) then keep running; the gap lets each come up before the
next (e.g. coordinator needs controller_manager from sim_headless).

    ros2 launch ros2_canbus bringup_sequence.launch.py
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

DELAY_S = 3.0        # seconds between each successive start
RESPAWN_DELAY_S = 5.0  # seconds before restarting a node that died

# (label, shell command, respawn) in start order — one per ~/.bashrc alias.
#
# respawn=True is for the plain long-running leaf nodes: if one dies (e.g. its
# hardware wasn't ready yet on a cold boot) it comes back on its own instead of
# staying dead for the whole session. It is deliberately OFF for sim_headless
# (a nested ros2 launch) and coordinator (a `spawner ... && ros2 launch` chain)
# — restarting those risks a restart loop or a double-spawned controller.
SEQUENCE = [
    ("sim_headless", "ros2 launch part_assembly_for_urdf_moveit_config demo.launch.py use_rviz:=false", False),
    ("sbus",         "ros2 run sbus_driver sbus_publisher", True),
    ("robot",        "ros2 run ros2_canbus robot", True),
    ("light",        "ros2 run ros2_canbus light", True),
    ("fire",         "ros2 run ros2_canbus fire", True),
    ("mode",         "ros2 run ros2_canbus mode", True),
    ("diagnostics",  "ros2 run ros2_canbus diagnostics", True),
    ("bms",          "ros2 run ros2_canbus bms", True),
    ("coordinator",  "ros2 run controller_manager spawner flipper_controller "
                     "&& ros2 launch part_assembly_for_urdf_coordinator coordinator.launch.py", False),
]


def generate_launch_description():
    actions = []
    for i, (label, cmd, respawn) in enumerate(SEQUENCE):
        actions.append(
            TimerAction(
                period=float(i) * DELAY_S,        # 0, 3, 6, ... seconds
                actions=[ExecuteProcess(
                    cmd=["bash", "-c", cmd],      # bash -c so coordinator's && works
                    name=label,
                    output="screen",
                    respawn=respawn,
                    respawn_delay=RESPAWN_DELAY_S,
                )],
            )
        )
    return LaunchDescription(actions)
