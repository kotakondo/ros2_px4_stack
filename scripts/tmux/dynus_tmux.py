#!/bin/usr/env python3

import subprocess
import os
import argparse


def get_tmux_base_index():
    """Get the tmux base-index and pane-base-index settings."""
    try:
        result = subprocess.run(["tmux", "show-option", "-gv", "base-index"],
                                capture_output=True, text=True, check=True)
        win_base = int(result.stdout.strip())
    except (subprocess.CalledProcessError, ValueError):
        win_base = 0
    try:
        result = subprocess.run(["tmux", "show-option", "-gvw", "pane-base-index"],
                                capture_output=True, text=True, check=True)
        pane_base = int(result.stdout.strip())
    except (subprocess.CalledProcessError, ValueError):
        pane_base = 0
    return win_base, pane_base


def run_tmux_commands(session_name, commands, top_pane=None):
    """
    Set up a TMUX session with an optional full-width top pane and a tiled
    grid of named panes below.

    top_pane:  optional (name, command_string) tuple for a full-width pane
               at the top of the window (e.g. htop).
    commands:  list of (name, command_string) tuples for the tiled grid.
    """
    try:
        # Start a new TMUX session (starts the server if not already running)
        subprocess.run(["tmux", "new-session", "-d", "-s", session_name], check=True)

        # Enable pane titles in the status/border
        subprocess.run(["tmux", "set-option", "-t", session_name,
                        "pane-border-status", "top"], check=True)
        subprocess.run(["tmux", "set-option", "-t", session_name,
                        "pane-border-format", " #{pane_title} "], check=True)

        # Query base indices after session exists so the server is running
        win_base, pane_base = get_tmux_base_index()
        w = win_base
        p = pane_base

        # Step 1: Create all command panes + blank shell using tiled layout.
        # (top_pane is added AFTER tiling so it doesn't get mixed into the grid.)
        total_bottom = len(commands) + 1
        for i in range(total_bottom - 1):
            flag = "-h" if i % 2 == 0 else "-v"
            subprocess.run(
                ["tmux", "split-window", flag, "-t", f"{session_name}:{w}"],
                check=True,
            )
            subprocess.run(
                ["tmux", "select-layout", "-t", f"{session_name}:{w}", "tiled"],
                check=True,
            )

        # Final tiled layout for all command panes
        subprocess.run(
            ["tmux", "select-layout", "-t", f"{session_name}:{w}", "tiled"],
            check=True,
        )

        # Assign commands to panes
        for i, entry in enumerate(commands):
            # Support (name, cmd) or (name, cmd, auto_run) tuples
            if len(entry) == 3:
                name, cmd, auto_run = entry
            else:
                name, cmd = entry
                auto_run = True
            pane_target = f"{session_name}:{w}.{p + i}"
            subprocess.run(
                ["tmux", "select-pane", "-t", pane_target, "-T", name],
                check=True,
            )
            full_command = (
                f"source ~/code/mavros_ws/install/setup.bash && "
                f"source ~/code/bridge_ws/install/setup.bash && "
                f"{cmd}"
            )
            send_keys_args = ["tmux", "send-keys", "-t", pane_target, full_command]
            if auto_run:
                send_keys_args.append("C-m")
            subprocess.run(send_keys_args, check=True)

        # Last pane is a blank shell with dynus_ws sourced
        blank_pane = f"{session_name}:{w}.{p + len(commands)}"
        subprocess.run(
            ["tmux", "select-pane", "-t", blank_pane, "-T", "SHELL"],
            check=True,
        )
        subprocess.run(
            ["tmux", "send-keys", "-t", blank_pane,
             "source ~/code/dynus_ws/install/setup.bash", "C-m"],
            check=True,
        )

        # Step 2: Add the top pane AFTER tiling using -f (full-width) and -b (before).
        # This creates a new pane spanning the entire window width above the tiled grid.
        if top_pane:
            subprocess.run(
                ["tmux", "split-window", "-v", "-f", "-b", "-p", "20",
                 "-t", f"{session_name}:{w}.{p}"],
                check=True,
            )
            # The new pane becomes the lowest index; existing panes shift up by 1.
            top_target = f"{session_name}:{w}.{p}"
            subprocess.run(
                ["tmux", "select-pane", "-t", top_target, "-T", top_pane[0]],
                check=True,
            )
            subprocess.run(
                ["tmux", "send-keys", "-t", top_target, top_pane[1], "C-m"],
                check=True,
            )
            # Blank pane index shifted by 1
            blank_pane = f"{session_name}:{w}.{p + len(commands) + 1}"

        # Focus the blank pane so the user can type immediately
        subprocess.run(["tmux", "select-pane", "-t", blank_pane], check=True)

        total_panes = total_bottom + (1 if top_pane else 0)
        print(f"TMUX session '{session_name}' created with {total_panes} panes. Attaching...")
        subprocess.run(["tmux", "attach-session", "-t", session_name])

    except subprocess.CalledProcessError as e:
        print(f"Error setting up TMUX session: {e}")


if __name__ == "__main__":
    veh = os.environ.get("VEH_NAME")
    mav_id = os.environ.get("MAV_SYS_ID")
    session_name = f"{veh}_tmux_session"

    parser = argparse.ArgumentParser()
    parser.add_argument('--odom_type', type=str, default="livox",
                        help="Odometry source: motion capture or lidar")
    parser.add_argument('--planner', type=str, default="dynus",
                        help="Planner to use (dynus or mighty)")
    parser.add_argument('--mode', type=str, default="flight",
                        choices=["flight", "debug"],
                        help="Zenoh mode: flight (minimal topics) or debug (all topics)")
    args = parser.parse_args()
    odom_type = args.odom_type
    planner = args.planner
    zenoh_mode = args.mode

    commands = [
        ("DYNUS", (
            f"source ~/code/dynus_ws/install/setup.bash && "
            f"source ~/code/decomp_ws/install/setup.bash && "
            f"sleep 10 && "
            f"ros2 launch dynus onboard_dynus.launch.py "
            f"x:=0.0 y:=0.0 z:=0.0 yaw:=0 namespace:={veh} "
            f"use_obstacle_tracker:=false use_ground_robot:=false "
            f"use_hardware:=true use_onboard_localization:=true "
            f"depth_camera_name:=d455"
        )),

        ("INIT POSE", (
            'sleep 10.0 && source ~/code/get_init_pose.sh && echo && '
            'printf "\\033[1;32minit pos: (%.2f, %.2f, %.2f)\\033[0m\\n" ${INIT_X} ${INIT_Y} ${INIT_Z} && '
            'printf "\\033[1;32minit att: (%.2f, %.2f, %.2f)\\033[0m\\n" ${INIT_ROLL} ${INIT_PITCH} ${INIT_YAW} && '
            'echo -e "\\033[1;32m****** [INIT POSE] INITIAL POSE RECEIVED ******\\033[0m" && echo'
        )),

        ("ORIENTATION",
            f"sleep 15.0 && python3 ~/code/mavros_ws/src/ros2_px4_stack/scripts/monitor_orientation.py {veh}/mavros/local_position/pose"),

        ("MAVROS",
            f"sleep 5.0 && ros2 launch mavros px4.launch namespace:={veh}/mavros tgt_system:={mav_id} 2>&1"
            r" | grep -v '\[INFO\]'"
            r" | sed -e 's/\[ERROR\]/\x1b[1;31m[ERROR]\x1b[0m/g' -e 's/\[WARN\]/\x1b[1;33m[WARN]\x1b[0m/g'"),

        ("LIVOX", (
            f"source ~/code/livox_ws/install/setup.bash && "
            f"sleep 10 && "
            f"ros2 launch livox_ros_driver2 run_MID360_launch.py namespace:={veh}"
        )),

        ("DLIO", (
            f"source ~/code/dynus_ws/install/setup.bash && "
            f"source ~/code/dlio_ws/install/setup.bash && "
            f"sleep 10 && "
            f"ros2 launch direct_lidar_inertial_odometry dlio.launch.py namespace:={veh} > /dev/null"
        )),

        ("PX4 BRIDGE", (
            f"source ~/code/dynus_ws/install/setup.bash && "
            f"sleep 10 && "
            f"source ~/code/get_init_pose.sh && "
            f"ros2 launch ros2_px4_stack dynus_mavros.launch.py odom_type:={odom_type}"
        )),

        ("BAG RECORD", (
            f"sleep 15 && "
            f"source ~/code/decomp_ws/install/setup.bash && "
            f"source ~/code/dynus_ws/install/setup.bash && "
            f"BAG_NAME=$(date +%Y%m%d_%H%M%S) && "
            f"mkdir -p ~/data/dynus && "
            f"python3 ~/code/dynus_ws/src/dynus/scripts/bag_record.py "
            f"--bag_name $BAG_NAME --bag_path ~/data/dynus --hardware --agents {veh}"
        )),

        ("ZENOH", (
            f"source ~/code/zenoh_ws/install/setup.bash && "
            f"source ~/code/decomp_ws/install/setup.bash && "
            f"source ~/code/dynus_ws/install/setup.bash && "
            f"echo 'Zenoh mode: {zenoh_mode}' && "
            f"ros2 run zenoh_vendor zenoh-bridge-ros2dds "
            f"-c ~/code/zenoh_ws/src/zenoh_vendor/configs/zenoh_agent_{zenoh_mode}.json5"
        )),

        ("ACL MAP", (
            f"source ~/code/dynus_ws/install/setup.bash && "
            f"ros2 launch global_mapper_ros global_mapper_node.launch.py "
            f"quad:={veh} depth_pointcloud_topic:=livox/lidar hardware:=true "
            f"pose_topic:=global_pose"
        )),

        ("DATA DIR",
            "cd ~/data/dynus && ls -lt"),

        ("GOAL MONITOR", (
            f"source ~/code/dynus_ws/install/setup.bash && "
            f"ros2 launch dynus goal_monitor.launch.py"
        ), False),
    ]

    run_tmux_commands(session_name, commands, top_pane=("HTOP", "htop"))
