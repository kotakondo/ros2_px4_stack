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


def run_tmux_commands(session_name, commands):
    """
    Set up a TMUX session with a 2x2 grid of panes, each running a specific command.

    Args:
        session_name (str): Name of the TMUX session.
        commands (list): A list of commands to execute in each pane (up to 4).
    """
    try:
        # Start a new TMUX session
        subprocess.run(["tmux", "new-session", "-d", "-s", session_name], check=True)

        w, p = get_tmux_base_index()

        # Split the terminal into a 2x2 grid
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:{w}"], check=True)  # Split horizontally
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:{w}.{p}"], check=True)  # Split top vertically
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:{w}.{p+1}"], check=True)  # Split bottom vertically
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:{w}.{p+2}"], check=True) # Split bottom-left horizontally
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:{w}.{p+3}"], check=True) # Split bottom-right horizontally
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:{w}.{p+4}"], check=True) # Split bottom-left horizontally
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:{w}.{p+5}"], check=True) # Split bottom-right vertically
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{session_name}:{w}.{p}"], check=True) # Split last horizontally
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{session_name}:{w}.{p}"], check=True) # Split bottom-right vertically

        # Commands to run in each pane
        for i, cmd in enumerate(commands):
            # Construct the full command with setup steps
            full_command = f"source ~/code/trajgen_ws/install/setup.bash && source ~/code/bridge_ws/install/setup.bash && source ~/code/mavros_ws/install/setup.bash && {cmd}"
            # Send the command to the corresponding pane
            subprocess.run(["tmux", "send-keys", "-t", f"{session_name}:{w}.{p+i}", full_command, "C-m"], check=True)

        # Select the first pane and arrange in an even grid
        subprocess.run(["tmux", "select-layout", "-t", session_name, "tiled"], check=True)

        # Attach to the session
        print(f"TMUX session '{session_name}' created with a 3x3 grid. Attaching...")
        subprocess.run(["tmux", "attach-session", "-t", session_name])

    except subprocess.CalledProcessError as e:
        print(f"Error setting up TMUX session: {e}")

if __name__ == "__main__":
    # Define the session name and commands
    veh = os.environ.get("VEH_NAME")
    mav_id = os.environ.get("MAV_SYS_ID")
    session_name = f"{veh}_tmux_session"

    # Get odom_type parameters from user 
    parser = argparse.ArgumentParser() 
    parser.add_argument('--odom_type', type=str, default="mocap", help="Odometry measurement to send flight controller - motion capture or lidar")
    args = parser.parse_args()
    odom_type = args.odom_type 

    if odom_type == "mocap":
        gt_odom_topic = "/world"
    else: 
        gt_odom_topic = "/dlio/odom_node/pose"
        
    commands = [
        f"ros2 launch mavros px4.launch namespace:={veh}/mavros tgt_system:={mav_id}",  # Command for pane 1
        # "ros2 launch trajectory_generator goal_replay.launch.py",
        "ros2 launch trajectory_generator_ros2 onboard.launch.py",  # Command for pane 2
        "ros2 launch trajectory_generator_ros2 base_station.launch.py",  # Command for pane 3
        f"ros2 launch ros2_px4_stack offboard_gen_traj.launch.py odom_type:={odom_type}",  # Command for pane 4,
        # f"source ~/code/livox_ws/install/setup.bash && sleep 10 && ros2 launch livox_ros_driver2 run_MID360_launch.py namespace:={veh}", # Pane 5
        # f"source ~/code/dynus_ws/install/setup.bash && source ~/code/dlio_ws/install/setup.bash && sleep 10 && ros2 launch direct_lidar_inertial_odometry dlio.launch.py namespace:={veh}", # Pane 6
        f"sleep 10.0 && ros2 topic echo {veh}/mavros/local_position/pose", # Pane 7
        f"sleep 10.0 && ros2 topic echo /{veh}" + gt_odom_topic, # Pane  8
        "sleep 10.0 && ros2 topic echo /SQ01/goal", 
        "source ~/code/zenoh_ws/install/setup.bash && ros2 run zenoh_vendor zenoh-bridge-ros2dds -c ~/code/zenoh_ws/src/zenoh_vendor/configs/zenoh_agent_flight.json5"
        # "zenoh_router", # Pane 10
    ]
    run_tmux_commands(session_name, commands)



# #!/bin/usr/env python3

# import subprocess
# import os
# import argparse


# def get_tmux_base_index():
#     """Get the tmux base-index and pane-base-index settings."""
#     try:
#         result = subprocess.run(["tmux", "show-option", "-gv", "base-index"],
#                                 capture_output=True, text=True, check=True)
#         win_base = int(result.stdout.strip())
#     except (subprocess.CalledProcessError, ValueError):
#         win_base = 0
#     try:
#         result = subprocess.run(["tmux", "show-option", "-gvw", "pane-base-index"],
#                                 capture_output=True, text=True, check=True)
#         pane_base = int(result.stdout.strip())
#     except (subprocess.CalledProcessError, ValueError):
#         pane_base = 0
#     return win_base, pane_base


# def run_tmux_commands(session_name, commands, top_pane=None):
#     """
#     Set up a TMUX session with an optional full-width top pane and a tiled
#     grid of named panes below.

#     top_pane:  optional (name, command_string) tuple for a full-width pane
#                at the top of the window (e.g. htop).
#     commands:  list of (name, command_string) tuples for the tiled grid.
#     """
#     try:
#         # Kill any existing session with the same name to avoid stale state
#         subprocess.run(["tmux", "kill-session", "-t", session_name],
#                        check=False, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

#         # Start a new TMUX session (starts the server if not already running)
#         subprocess.run(["tmux", "new-session", "-d", "-s", session_name], check=True)

#         # Enable pane titles in the status/border
#         subprocess.run(["tmux", "set-option", "-t", session_name,
#                         "pane-border-status", "top"], check=True)
#         subprocess.run(["tmux", "set-option", "-t", session_name,
#                         "pane-border-format", " #{pane_title} "], check=True)

#         # Query base indices after session exists so the server is running
#         win_base, pane_base = get_tmux_base_index()
#         w = win_base
#         p = pane_base

#         # Step 1: Create all command panes + blank shell using tiled layout.
#         # (top_pane is added AFTER tiling so it doesn't get mixed into the grid.)
#         total_bottom = len(commands) + 1
#         for i in range(total_bottom - 1):
#             flag = "-h" if i % 2 == 0 else "-v"
#             subprocess.run(
#                 ["tmux", "split-window", flag, "-t", f"{session_name}:{w}"],
#                 check=True,
#             )
#             subprocess.run(
#                 ["tmux", "select-layout", "-t", f"{session_name}:{w}", "tiled"],
#                 check=True,
#             )

#         # Final tiled layout for all command panes
#         subprocess.run(
#             ["tmux", "select-layout", "-t", f"{session_name}:{w}", "tiled"],
#             check=True,
#         )

#         # Assign commands to panes
#         for i, entry in enumerate(commands):
#             # Support (name, cmd) or (name, cmd, auto_run) tuples
#             if len(entry) == 3:
#                 name, cmd, auto_run = entry
#             else:
#                 name, cmd = entry
#                 auto_run = True
#             pane_target = f"{session_name}:{w}.{p + i}"
#             subprocess.run(
#                 ["tmux", "select-pane", "-t", pane_target, "-T", name],
#                 check=True,
#             )
#             full_command = (
#                 f"source ~/code/mavros_ws/install/setup.bash && "
#                 f"source ~/code/bridge_ws/install/setup.bash && "
#                 f"source ~/code/uav_trajectory_simulator_ws/install/setup.bash && "
#                 f"{cmd}"
#             )
#             send_keys_args = ["tmux", "send-keys", "-t", pane_target, full_command]
#             if auto_run:
#                 send_keys_args.append("C-m")
#             subprocess.run(send_keys_args, check=True)

#         # Last pane is a blank shell with mavros_ws sourced
#         blank_pane = f"{session_name}:{w}.{p + len(commands)}"
#         subprocess.run(
#             ["tmux", "select-pane", "-t", blank_pane, "-T", "SHELL"],
#             check=True,
#         )
#         subprocess.run(
#             ["tmux", "send-keys", "-t", blank_pane,
#              "source ~/code/mavros_ws/install/setup.bash", "C-m"],
#             check=True,
#         )

#         # Step 2: Add the top pane AFTER tiling using -f (full-width) and -b (before).
#         # This creates a new pane spanning the entire window width above the tiled grid.
#         if top_pane:
#             subprocess.run(
#                 ["tmux", "split-window", "-v", "-f", "-b", "-p", "20",
#                  "-t", f"{session_name}:{w}.{p}"],
#                 check=True,
#             )
#             # The new pane becomes the lowest index; existing panes shift up by 1.
#             top_target = f"{session_name}:{w}.{p}"
#             subprocess.run(
#                 ["tmux", "select-pane", "-t", top_target, "-T", top_pane[0]],
#                 check=True,
#             )
#             subprocess.run(
#                 ["tmux", "send-keys", "-t", top_target, top_pane[1], "C-m"],
#                 check=True,
#             )
#             # Blank pane index shifted by 1
#             blank_pane = f"{session_name}:{w}.{p + len(commands) + 1}"

#         # Focus the blank pane so the user can type immediately
#         subprocess.run(["tmux", "select-pane", "-t", blank_pane], check=True)

#         total_panes = total_bottom + (1 if top_pane else 0)
#         print(f"TMUX session '{session_name}' created with {total_panes} panes. Attaching...")
#         subprocess.run(["tmux", "attach-session", "-t", session_name])

#     except subprocess.CalledProcessError as e:
#         print(f"Error setting up TMUX session: {e}")


# if __name__ == "__main__":
#     veh = os.environ.get("VEH_NAME")
#     mav_id = os.environ.get("MAV_SYS_ID")
#     session_name = f"{veh}_tmux_session"

#     parser = argparse.ArgumentParser()
#     parser.add_argument('--odom_type', type=str, default="mocap",
#                         help="Odometry source: motion capture or lidar")
#     parser.add_argument('--mode', type=str, default="flight",
#                         choices=["flight", "debug"],
#                         help="Zenoh mode: flight (minimal topics) or debug (all topics)")
#     args = parser.parse_args()
#     odom_type = args.odom_type
#     zenoh_mode = args.mode

#     if odom_type == "mocap":
#         gt_odom_topic = f"/{veh}/world"
#     else:
#         gt_odom_topic = f"/{veh}/dlio/odom_node/pose"

#     commands = [
#         ("TRAJGEN",
#             f"sleep 10 && ros2 launch trajectory_generator_ros2 onboard.launch.py"),

#         ("ORIENTATION",
#             f"sleep 15.0 && python3 ~/code/mavros_ws/src/ros2_px4_stack/scripts/monitor_orientation.py {veh}/mavros/local_position/pose"),

#         ("MAVROS",
#             f"sleep 5.0 && ros2 launch mavros px4.launch namespace:={veh}/mavros tgt_system:={mav_id} 2>&1"
#             r" | grep -v '\[INFO\]'"
#             r" | sed -e 's/\[ERROR\]/\x1b[1;31m[ERROR]\x1b[0m/g' -e 's/\[WARN\]/\x1b[1;33m[WARN]\x1b[0m/g'"),

#         ("PX4 BRIDGE", (
#             f"sleep 20 && "
#             f"ros2 launch ros2_px4_stack offboard_gen_traj.launch.py odom_type:={odom_type}"
#         )),

#         ("LOCAL POSE",
#             f"sleep 15.0 && ros2 topic echo /{veh}/mavros/local_position/pose"),

#         ("MOCAP" if odom_type == "mocap" else "ODOM",
#             f"sleep 15.0 && ros2 topic echo {gt_odom_topic}"),

#         ("GOAL",
#             f"sleep 15.0 && ros2 topic echo /{veh}/goal"),

#         ("ZENOH", (
#             f"source ~/code/zenoh_ws/install/setup.bash && "
#             f"ros2 run zenoh_vendor zenoh-bridge-ros2dds "
#             f"-c ~/code/zenoh_ws/src/zenoh_vendor/configs/zenoh_agent_{zenoh_mode}.json5"
#         )),
#     ]

#     run_tmux_commands(session_name, commands, top_pane=("HTOP", "htop"))
