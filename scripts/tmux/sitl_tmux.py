#!/usr/bin/env python3
"""
One-command launcher for the PX4-SITL + trajectory_generator simulation.

Brings up (in a tmux grid):
  0. PX4 SITL (Gazebo)
  1. MAVROS bridged to the SITL UDP endpoint
  2. trajectory_generator_ros2 (publishes snapstack_msgs2/Goal)
  3. ros2_px4_stack offboard bridge (Goal -> MAVROS setpoints)
  4. behavior_selector2 GUI (GO / END / KILL buttons)

Usage:
    python3 sitl_tmux.py [--px4_dir ~/code/PX4-Autopilot] [--model gz_x500]
"""

import argparse
import os
import subprocess

WS_DIR = os.path.expanduser("~/code/uav_trajectory_simulator_ws")
DEFAULT_PX4_DIR = os.path.expanduser("~/code/PX4-Autopilot")
# Gazebo Classic target baked into build/px4_sitl_default/build.ninja.
# Switch to `gz_x500` only if you install Gazebo Harmonic/Garden and rebuild PX4.
DEFAULT_MODEL = "gazebo-classic_iris"

# Sourced at the start of every pane. Conda needs to be fully out of the
# environment — its ~/anaconda3/lib leaks into LD_LIBRARY_PATH (breaks the PX4
# Gazebo-Classic build via libuuid conflict) and its bin/cl masquerades as a
# C++ compiler (breaks colcon rebuilds). Deactivating once only drops (env)
# back to (base), which is not enough — loop until CONDA_SHLVL hits 0.
PREAMBLE = (
    "while [ \"${CONDA_SHLVL:-0}\" -gt 0 ]; do conda deactivate 2>/dev/null || break; done; "
    # Even with conda deactivated, ~/anaconda3/bin usually stays on PATH (via
    # ~/.bashrc), so CMake find_package still prefers anaconda's Boost/OGRE/etc.
    # Strip any anaconda-tainted dirs from PATH and LD_LIBRARY_PATH.
    "export PATH=$(echo \"$PATH\" | tr ':' '\\n' | grep -v anaconda | paste -sd:); "
    "export LD_LIBRARY_PATH=$(echo \"${LD_LIBRARY_PATH:-}\" | tr ':' '\\n' | grep -v anaconda | paste -sd:); "
    "source /opt/ros/humble/setup.bash && "
    f"source {WS_DIR}/install/setup.bash"
)


def build_commands(px4_dir: str, model: str, veh: str, mav_id: str) -> list[str]:
    sitl = f"cd {px4_dir} && make px4_sitl {model}"

    # PX4 SITL opens UDP 14580 (inbound, MAVROS→PX4) and 14540 (outbound)
    # shortly after "Simulator connected on TCP port 4560" — usually 8–10 s
    # from `make px4_sitl` start. These sleeps leave ~2 s of margin; bump
    # them if you see MAVROS retry on a slow machine.
    mavros = (
        f"sleep 12 && ros2 launch mavros px4.launch "
        f"fcu_url:=udp://:14540@127.0.0.1:14580 "
        f"namespace:={veh}/mavros tgt_system:={mav_id}"
    )

    # onboard.launch.py reads VEHTYPE/VEHNUM from env to build the namespace.
    # The config YAML is keyed by "SQ01:", so we hard-code that split here.
    trajgen = (
        f"sleep 15 && VEHTYPE={veh[:2]} VEHNUM={veh[2:]} "
        f"ros2 launch trajectory_generator_ros2 onboard.launch.py"
    )

    bridge = (
        f"sleep 17 && VEH_NAME={veh} "
        f"ros2 launch ros2_px4_stack offboard_gen_traj.launch.py"
    )

    gui = (
        f"sleep 17 && "
        f"ros2 launch trajectory_generator_ros2 base_station.launch.py"
    )

    return [sitl, mavros, trajgen, bridge, gui]


def run_tmux(session: str, commands: list[str]) -> None:
    subprocess.run(["tmux", "kill-session", "-t", session],
                   stderr=subprocess.DEVNULL, check=False)
    subprocess.run(["tmux", "new-session", "-d", "-s", session], check=True)

    # Resolve the real window index (users may set base-index 1 in ~/.tmux.conf,
    # so we cannot assume window :0).
    window_id = subprocess.run(
        ["tmux", "display-message", "-p", "-t", session, "#I"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    window = f"{session}:{window_id}"

    # Create (len(commands) - 1) extra panes by splitting repeatedly,
    # then rely on `select-layout tiled` for a clean grid.
    for _ in range(len(commands) - 1):
        subprocess.run(["tmux", "split-window", "-t", window], check=True)
        subprocess.run(
            ["tmux", "select-layout", "-t", window, "tiled"], check=True
        )

    # Pane indices may also start at 1 depending on `pane-base-index`.
    pane_ids = subprocess.run(
        ["tmux", "list-panes", "-t", window, "-F", "#P"],
        check=True, capture_output=True, text=True,
    ).stdout.split()

    for pane, cmd in zip(pane_ids, commands):
        full = f"{PREAMBLE} && {cmd}"
        subprocess.run(
            ["tmux", "send-keys", "-t", f"{window}.{pane}", full, "C-m"],
            check=True,
        )

    subprocess.run(["tmux", "select-layout", "-t", window, "tiled"], check=True)
    print(f"tmux session '{session}' started. Attaching...")
    subprocess.run(["tmux", "attach-session", "-t", session])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4_dir", default=DEFAULT_PX4_DIR,
                        help="Path to PX4-Autopilot clone")
    parser.add_argument("--model", default=DEFAULT_MODEL,
                        help="PX4 SITL make target (e.g. gazebo_iris, gazebo_x500_vision)")
    parser.add_argument("--veh", default="SQ01",
                        help="Vehicle namespace (must match config/default.yaml key)")
    parser.add_argument("--mav_id", default="1",
                        help="MAVLink target system id")
    args = parser.parse_args()

    if not os.path.isdir(args.px4_dir):
        raise SystemExit(f"PX4 directory not found: {args.px4_dir}")

    commands = build_commands(args.px4_dir, args.model, args.veh, args.mav_id)
    run_tmux(f"{args.veh}_sitl", commands)


if __name__ == "__main__":
    main()
