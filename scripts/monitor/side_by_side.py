#!/usr/bin/env python3
"""Side-by-side monitor: Fast-LIO Odometry_IMU (converted to world) vs mocap /PX03/world."""

import os, sys, math, time, subprocess, re
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from threading import Thread

def quat_to_yaw(o):
    siny = 2*(o.w*o.z + o.x*o.y)
    cosy = 1 - 2*(o.y*o.y + o.z*o.z)
    return math.atan2(siny, cosy)

def quat_mult(q1, q2):
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return np.array([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2])

def rot_from_euler(roll, pitch, yaw):
    cr,sr = math.cos(roll),math.sin(roll)
    cp,sp = math.cos(pitch),math.sin(pitch)
    cy,sy = math.cos(yaw),math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp,   cp*sr,          cp*cr]])

class Monitor(Node):
    def __init__(self):
        super().__init__('side_by_side_monitor')
        self.veh = os.environ.get("VEH_NAME", "PX03")

        # Init pose
        ix = float(os.environ.get("INIT_X","0"))
        iy = float(os.environ.get("INIT_Y","0"))
        iz = float(os.environ.get("INIT_Z","0"))
        ir = float(os.environ.get("INIT_ROLL","0"))
        ip = float(os.environ.get("INIT_PITCH","0"))
        iw = float(os.environ.get("INIT_YAW","0"))
        self.t_init = np.array([ix, iy, iz])
        self.R_init = rot_from_euler(ir, ip, iw)
        # Quaternion for orientation rotation (x,y,z,w)
        cr,sr = math.cos(ir/2),math.sin(ir/2)
        cp,sp = math.cos(ip/2),math.sin(ip/2)
        cy,sy = math.cos(iw/2),math.sin(iw/2)
        self.q_init = np.array([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                                cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])

        self.flio = None
        self.fastlio = None
        self.mocap = None
        self.px4 = None
        self.goal = None

        self.create_subscription(Odometry, f'/{self.veh}/dlio/odom_node/odom', self.flio_cb, 10)
        self.create_subscription(Odometry, f'/{self.veh}/fast_lio/Odometry', self.fastlio_cb, 10)
        self.create_subscription(PoseStamped, f'/{self.veh}/world', self.mocap_cb, 10)
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, f'/{self.veh}/mavros/local_position/pose', self.px4_cb, qos_be)

        # Subscribe to DYNUS goal (what the planner sends to the tracker)
        try:
            from dynus_interfaces.msg import Goal
            self.create_subscription(Goal, f'/{self.veh}/goal', self.goal_cb, qos_be)
        except ImportError:
            self.get_logger().warn("dynus_interfaces not found, skipping goal subscription")

        # Log file
        ts = time.strftime('%Y%m%d_%H%M%S')
        os.makedirs(os.path.expanduser('~/data/debug'), exist_ok=True)
        self.logf = open(os.path.expanduser(f'~/data/debug/flight_{ts}.csv'), 'w')
        self.logf.write('t,src,x,y,z,yaw,vx,vy,vz\n')

        # CPU log file
        self.cpulogf = open(os.path.expanduser(f'~/data/debug/cpu_{ts}.csv'), 'w')
        self.cpulogf.write('t,process,cpu_pct,mem_mb\n')
        self.cpu_data = {}  # latest cpu readings
        self._cpu_thread = Thread(target=self._cpu_monitor_loop, daemon=True)
        self._cpu_thread.start()

        self.create_timer(0.2, self.print_cb)  # 5 Hz display

    def flio_cb(self, msg):
        p = msg.pose.pose.position
        local_pos = np.array([p.x, p.y, p.z])
        global_pos = self.R_init @ local_pos + self.t_init
        v = msg.twist.twist.linear
        local_vel = np.array([v.x, v.y, v.z])
        global_vel = self.R_init @ local_vel
        # Rotate orientation to world frame: q_global = q_init * q_local
        o = msg.pose.pose.orientation
        local_q = np.array([o.x, o.y, o.z, o.w])
        global_q = quat_mult(self.q_init, local_q)
        # Extract yaw from world-frame quaternion
        yaw = math.atan2(2*(global_q[3]*global_q[2] + global_q[0]*global_q[1]),
                         1 - 2*(global_q[1]**2 + global_q[2]**2))
        self.flio = {'x': global_pos[0], 'y': global_pos[1], 'z': global_pos[2],
                     'yaw': yaw, 'vx': global_vel[0], 'vy': global_vel[1], 'vz': global_vel[2]}
        self.logf.write(f'{time.time()},dlio_world,{global_pos[0]:.4f},{global_pos[1]:.4f},{global_pos[2]:.4f},{yaw:.3f},{global_vel[0]:.3f},{global_vel[1]:.3f},{global_vel[2]:.3f}\n')

    def fastlio_cb(self, msg):
        p = msg.pose.pose.position
        local_pos = np.array([p.x, p.y, p.z])
        global_pos = self.R_init @ local_pos + self.t_init
        o = msg.pose.pose.orientation
        local_q = np.array([o.x, o.y, o.z, o.w])
        global_q = quat_mult(self.q_init, local_q)
        yaw = math.atan2(2*(global_q[3]*global_q[2] + global_q[0]*global_q[1]),
                         1 - 2*(global_q[1]**2 + global_q[2]**2))
        self.fastlio = {'x': global_pos[0], 'y': global_pos[1], 'z': global_pos[2], 'yaw': yaw}
        self.logf.write(f'{time.time()},flio_world,{global_pos[0]:.4f},{global_pos[1]:.4f},{global_pos[2]:.4f},{yaw:.3f},,,\n')

    def mocap_cb(self, msg):
        p = msg.pose.position; o = msg.pose.orientation
        yaw = quat_to_yaw(o)
        self.mocap = {'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw}
        self.logf.write(f'{time.time()},mocap,{p.x:.4f},{p.y:.4f},{p.z:.4f},{yaw:.3f},,,\n')

    def px4_cb(self, msg):
        p = msg.pose.position; o = msg.pose.orientation
        yaw = quat_to_yaw(o)
        self.px4 = {'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw}
        self.logf.write(f'{time.time()},px4,{p.x:.4f},{p.y:.4f},{p.z:.4f},{yaw:.3f},,,\n')

    def goal_cb(self, msg):
        self.goal = {'x': msg.p.x, 'y': msg.p.y, 'z': msg.p.z, 'yaw': msg.yaw,
                     'vx': msg.v.x, 'vy': msg.v.y, 'vz': msg.v.z}
        self.logf.write(f'{time.time()},goal,{msg.p.x:.4f},{msg.p.y:.4f},{msg.p.z:.4f},{msg.yaw:.3f},{msg.v.x:.3f},{msg.v.y:.3f},{msg.v.z:.3f}\n')

    def _cpu_monitor_loop(self):
        """Poll CPU usage of key processes every 1s."""
        targets = {
            'dynus': 'dynus',
            'dlio': 'dlio_odom',
            'fastlio': 'fastlio_mapping',
            'mapper': 'global_mapper',
            'tracker': 'track_dynus',
            'mavros': 'mavros_node',
        }
        while True:
            try:
                result = subprocess.run(
                    ['ps', '-eo', 'pid,pcpu,rss,comm', '--no-headers'],
                    capture_output=True, text=True, timeout=2)
                t = time.time()
                readings = {}
                for line in result.stdout.strip().split('\n'):
                    parts = line.split()
                    if len(parts) >= 4:
                        pid, cpu, rss, comm = parts[0], parts[1], parts[2], ' '.join(parts[3:])
                        for name, pattern in targets.items():
                            if pattern in comm:
                                cpu_val = float(cpu)
                                mem_mb = float(rss) / 1024
                                readings[name] = {'cpu': cpu_val, 'mem': mem_mb}
                                self.cpulogf.write(f'{t},{name},{cpu_val:.1f},{mem_mb:.1f}\n')
                self.cpu_data = readings
                self.cpulogf.flush()
            except Exception:
                pass
            time.sleep(1.0)

    def print_cb(self):
        self.logf.flush()
        # Clear screen and print header
        sys.stdout.write('\033[2J\033[H')
        print(f'\033[1m{"":>12} {"x":>8} {"y":>8} {"z":>8} {"yaw":>7}  {"vx":>7} {"vy":>7} {"vz":>7}\033[0m')
        if self.flio:
            f = self.flio
            print(f'\033[36m{"DLIO→world":>12} {f["x"]:8.3f} {f["y"]:8.3f} {f["z"]:8.3f} {f["yaw"]:7.2f}  {f["vx"]:7.3f} {f["vy"]:7.3f} {f["vz"]:7.3f}\033[0m')
        else:
            print(f'\033[36m{"DLIO→world":>12} --- waiting ---\033[0m')

        if self.fastlio:
            fl = self.fastlio
            print(f'\033[34m{"FLIO→world":>12} {fl["x"]:8.3f} {fl["y"]:8.3f} {fl["z"]:8.3f} {fl["yaw"]:7.2f}\033[0m')
        else:
            print(f'\033[34m{"FLIO→world":>12} --- waiting ---\033[0m')

        if self.mocap:
            m = self.mocap
            print(f'\033[32m{"Mocap":>12} {m["x"]:8.3f} {m["y"]:8.3f} {m["z"]:8.3f} {m["yaw"]:7.2f}\033[0m')
        else:
            print(f'\033[32m{"Mocap":>12} --- waiting ---\033[0m')

        if self.px4:
            p = self.px4
            print(f'\033[33m{"PX4 local":>12} {p["x"]:8.3f} {p["y"]:8.3f} {p["z"]:8.3f} {p["yaw"]:7.2f}\033[0m')
        else:
            print(f'\033[33m{"PX4 local":>12} --- no data ---\033[0m')

        if self.goal:
            g = self.goal
            print(f'\033[35m{"DYNUS goal":>12} {g["x"]:8.3f} {g["y"]:8.3f} {g["z"]:8.3f} {g["yaw"]:7.2f}  {g["vx"]:7.3f} {g["vy"]:7.3f} {g["vz"]:7.3f}\033[0m')

        # Error: DLIO vs mocap
        if self.flio and self.mocap:
            f, m = self.flio, self.mocap
            dx, dy, dz = f['x']-m['x'], f['y']-m['y'], f['z']-m['z']
            dist = (dx**2+dy**2+dz**2)**0.5
            dyaw = f['yaw'] - m['yaw']
            if dyaw > math.pi: dyaw -= 2*math.pi
            if dyaw < -math.pi: dyaw += 2*math.pi
            color = '\033[32m' if dist < 0.15 else '\033[33m' if dist < 0.3 else '\033[31m'
            print(f'{color}{"DLIO err":>12} {dx:+8.3f} {dy:+8.3f} {dz:+8.3f} {dyaw:+7.2f}  |{dist:.3f}|m\033[0m')

        # Error: FAST-LIO vs mocap
        if self.fastlio and self.mocap:
            fl, m = self.fastlio, self.mocap
            dx, dy, dz = fl['x']-m['x'], fl['y']-m['y'], fl['z']-m['z']
            dist = (dx**2+dy**2+dz**2)**0.5
            dyaw = fl['yaw'] - m['yaw']
            if dyaw > math.pi: dyaw -= 2*math.pi
            if dyaw < -math.pi: dyaw += 2*math.pi
            color = '\033[32m' if dist < 0.15 else '\033[33m' if dist < 0.3 else '\033[31m'
            print(f'{color}{"FLIO err":>12} {dx:+8.3f} {dy:+8.3f} {dz:+8.3f} {dyaw:+7.2f}  |{dist:.3f}|m\033[0m')

        # CPU usage
        if self.cpu_data:
            print()
            total_cpu = sum(v['cpu'] for v in self.cpu_data.values())
            ncpus = os.cpu_count() or 1
            max_cpu = ncpus * 100
            print(f'\033[1m  CPU Usage (total: {total_cpu:.0f}% / {max_cpu}%)\033[0m')
            for name in ['dynus', 'dlio', 'fastlio', 'mapper', 'tracker', 'mavros']:
                if name in self.cpu_data:
                    c = self.cpu_data[name]
                    bar_len = int(c['cpu'] / 10)
                    bar = '█' * bar_len + '░' * (16 - bar_len)
                    color = '\033[31m' if c['cpu'] > 200 else '\033[33m' if c['cpu'] > 100 else '\033[32m'
                    print(f'  {color}{name:>10} {bar} {c["cpu"]:6.1f}% {c["mem"]:6.0f}MB\033[0m')

def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.logf.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
