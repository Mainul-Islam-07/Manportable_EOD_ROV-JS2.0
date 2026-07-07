#!/usr/bin/env python3
"""
bringup_sequence.launch.py
==========================
Starts the full robot stack one process at a time, 3 s apart, in order:

    sim_headless -> sbus -> robot -> battery -> light -> fire -> mode
    -> diagnostics -> coordinator

Each entry mirrors the matching alias in ~/.bashrc. Processes are STARTED 3 s
apart (TimerAction) then keep running; the gap lets each come up before the
next (e.g. coordinator needs controller_manager from sim_headless).

    ros2 launch ros2_canbus bringup_sequence.launch.py
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

DELAY_S = 3.0   # seconds between each successive start

# (label, shell command) in start order — one per ~/.bashrc alias.
SEQUENCE = [
    ("sim_headless", "ros2 launch part_assembly_for_urdf_moveit_config demo.launch.py use_rviz:=false"),
    ("sbus",         "ros2 run sbus_driver sbus_publisher"),
    ("robot",        "ros2 run ros2_canbus robot"),
    ("light",        "ros2 run ros2_canbus light"),
    ("fire",         "ros2 run ros2_canbus fire"),
    ("mode",         "ros2 run ros2_canbus mode"),
    ("diagnostics",  "ros2 run ros2_canbus diagnostics"),
    ("coordinator",  "ros2 run controller_manager spawner flipper_controller "
                     "&& ros2 launch part_assembly_for_urdf_coordinator coordinator.launch.py"),
]


def generate_launch_description():
    actions = []
    for i, (label, cmd) in enumerate(SEQUENCE):
        actions.append(
            TimerAction(
                period=float(i) * DELAY_S,        # 0, 3, 6, ... seconds
                actions=[ExecuteProcess(
                    cmd=["bash", "-c", cmd],      # bash -c so coordinator's && works
                    name=label,
                    output="screen",
                )],
            )
        )
    return LaunchDescription(actions)
