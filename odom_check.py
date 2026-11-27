# compares odometry with vicon while the robot is moved by hand
# vicon yaw offset is read like in vicon_yaw_calibration.py

import argparse
import bisect
import cmath
import math
import sys
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_utils import (add_yaw_offset_arguments, init_ros, normalize, resolve_yaw_offset,
                       yaw_from_quaternion)

LAGS = [0.01 * i for i in range(-5, 31)]  # vicon lag behind odometry [s]
MIN_PATH = 0.3  # [m]
MIN_TURN = 0.8   # [rad]


def rate_and_gap(times):
    # mean rate [Hz] and longest gap [s]
    gaps = [b - a for a, b in zip(times, times[1:])]
    span = times[-1] - times[0]
    return (len(gaps) / span if span > 0 else 0.0), max(gaps, default=0.0)


def unwrap(angles):
    # angles without the 2 pi skips
    out = [angles[0]]
    for a in angles[1:]:
        out.append(out[-1] + normalize(a - out[-1]))
    return out


def path_length(points, step=0.01):
    # length of the path
    length, last = 0.0, points[0]
    for p in points[1:]:
        d = math.hypot(p[0] - last[0], p[1] - last[1])
        if d >= step:
            length += d
            last = p
    return length


def interpolate(times, values, t):
    # linear interpolation of values at time t
    i = bisect.bisect_left(times, t)
    if i == 0 or i == len(times):
        return None
    f = (t - times[i - 1]) / (times[i] - times[i - 1])
    return tuple(a + f * (b - a) for a, b in zip(values[i - 1], values[i]))


def solve(a, b):
    # solves a x = b
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, n):
            f = m[r][col] / m[col][col]
            m[r] = [x - f * y for x, y in zip(m[r], m[col])]
    x = [0j] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


def fit(pairs, with_offset):
    # least squares vicon = a * odom + b (+ c * exp(i yaw)) for pairs (odom, vicon, vicon yaw)
    # a is scale and rotation of the odometry frame, b is shift, c is vicon origin in the robot frame
    rows = [[zo, 1.0 + 0j] + ([cmath.exp(1j * yaw)] if with_offset else [])
            for zo, _, yaw in pairs]
    n = len(rows[0])
    gram = [[sum(r[j].conjugate() * r[k] for r in rows) for k in range(n)] for j in range(n)]
    rhs = [sum(r[j].conjugate() * zv for r, (_, zv, _) in zip(rows, pairs)) for j in range(n)]
    x = solve(gram, rhs)
    res = [abs(zv - sum(c * v for c, v in zip(r, x))) for r, (_, zv, _) in zip(rows, pairs)]
    rms = math.sqrt(sum(e * e for e in res) / len(res))
    return x, rms, max(res)


class OdomCheck(Node):
    # records odometry and vicon arrivals and compared with report()

    def __init__(self, args):
        super().__init__('odom_check')
        self.a = args
        self.yaw_offset_deg = resolve_yaw_offset(self, args, args.vicon_yaw_offset_deg)
        self.odom = []   # (arrival time, x, y, yaw)
        self.vicon = []  # (arrival time, x, y, yaw with offset)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, odom_qos)
        self.create_subscription(PoseStamped, args.vicon_topic, self.vicon_cb, qos)

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.odom.append((time.monotonic(), p.position.x, p.position.y,
                          yaw_from_quaternion(p.orientation)))

    def vicon_cb(self, msg):
        p = msg.pose
        q = p.orientation
        if not abs(math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2) - 1.0) < 0.01:
            return  # invalid (also nan) quaternion
        yaw = normalize(yaw_from_quaternion(q) + math.radians(self.yaw_offset_deg))
        self.vicon.append((time.monotonic(), p.position.x, p.position.y, yaw))

    def report(self):
        a = self.a
        print(f'\nOdometry: {a.odom_topic} | messages: {len(self.odom)}')
        print(f'Vicon: {a.vicon_topic} | messages: {len(self.vicon)}')
        if len(self.odom) < 10 or len(self.vicon) < 10:
            print('not enough messages on both topics\n')
            return False

        ot = [s[0] for s in self.odom]
        vt = [s[0] for s in self.vicon]
        for name, times in (('Odometry', ot), ('Vicon', vt)):
            rate, gap = rate_and_gap(times)
            print(f'{name}: {rate:.1f} Hz | longest gap {gap * 1000:.0f} ms')

        o_yaw = unwrap([s[3] for s in self.odom])
        v_yaw = unwrap([s[3] for s in self.vicon])
        o_path = path_length([s[1:3] for s in self.odom])
        v_path = path_length([s[1:3] for s in self.vicon])
        o_turn, v_turn = o_yaw[-1] - o_yaw[0], v_yaw[-1] - v_yaw[0]
        print(f'Travelled: vicon {v_path:.3f} m, odometry {o_path:.3f} m')
        print(f'Turned: vicon {math.degrees(v_turn):+.1f} deg, '
              f'odometry {math.degrees(o_turn):+.1f} deg')

        if min(o_path, v_path) < MIN_PATH:
            print(f'too little movement to compare (odometry {o_path:.2f} m, '
                  f'VICON {v_path:.2f} m), push the robot at least 1 m\n')
            return False

        # vicon sample paired with the odometry at the same moment, for every lag in LAGS
        o_vals = [(s[1], s[2], y) for s, y in zip(self.odom, o_yaw)]
        v_span = max(v_yaw) - min(v_yaw)
        with_offset = v_span >= MIN_TURN
        best = None
        for lag in LAGS:
            pairs = []
            for s, y in zip(self.vicon, v_yaw):
                o = interpolate(ot, o_vals, s[0] - lag)
                if o is not None:
                    pairs.append((complex(o[0], o[1]), complex(s[1], s[2]), s[3], o[2], y))
            if len(pairs) < 10:
                continue
            x, rms, worst = fit([p[:3] for p in pairs], with_offset)
            if best is None or rms < best[0]:
                best = (rms, worst, x, lag, pairs)
        if best is None:
            print('the odometry and vicon recordings do not overlap in time\n')
            return False
        rms, worst, x, lag, pairs = best

        scale, rotation = 1.0 / abs(x[0]), cmath.phase(x[0])  # scale: odometry / vicon distance
        yaw_gap = [normalize(vy - oy) for _, _, _, oy, vy in pairs]
        yaw_rotation = cmath.phase(sum(cmath.exp(1j * g) for g in yaw_gap))
        print(f'\nBest fit with the vicon {lag * 1000:.0f} ms behind the odometry:')
        print(f'Odometry distance / vicon distance: {scale:.3f} (1 is exact)')
        print(f'Odometry frame rotated {math.degrees(rotation):+.1f} deg to the vicon frame '
              f'from positions, {math.degrees(yaw_rotation):+.1f} deg from headings')
        print(f'Position error after the fit: {rms * 1000:.1f} mm rms, {worst * 1000:.1f} mm max')
        if with_offset:
            print(f'Vicon origin on the robot: {x[2].real * 1000:+.0f} mm forward, '
                  f'{x[2].imag * 1000:+.0f} mm left of the odometry point')
        n = max(len(pairs) // 10, 1)
        gap = [normalize(g - yaw_rotation) for g in yaw_gap]  # no skip at +-180 deg
        drift = sum(gap[:n]) / n - sum(gap[-n:]) / n
        print(f'Heading drift of the odometry over the run: {math.degrees(drift):+.2f} deg')

        warnings = []
        if not with_offset:
            warnings.append('little rotation, turn the robot by hand to locate the vicon origin')
        if abs(normalize(rotation - yaw_rotation)) > math.radians(2.0):
            warnings.append('headings and positions disagree about the frame rotation, '
                            'check the yaw offset')
        if abs(scale - 1.0) > 0.05:
            warnings.append(f'odometry measures {100 * (scale - 1.0):+.1f} % too much distance')
        print('\nWARNINGS')
        for w in warnings:
            print(f'{w}')
        if not warnings:
            print('none')
        return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--odom-topic', default='/do150_0092/platform/odom')
    p.add_argument('--vicon-topic', default='/vicon/Dingo/Dingo')
    p.add_argument('--duration', type=float, default=30.0)
    add_yaw_offset_arguments(p, '--vicon-yaw-offset-deg')
    args = p.parse_args()

    init_ros()
    node = OdomCheck(args)
    ok = False
    try:
        end = time.monotonic() + args.duration
        node.get_logger().info(f'recording for {args.duration:.0f} s, move the robot now')
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ok = node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
