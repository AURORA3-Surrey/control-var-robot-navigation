import argparse
import json
import math
import sys
import time
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_utils import (DEFAULT_YAW_OFFSET_FILE, TimedNode, TwistSender, clamp, ignore_stop_signals,
                       init_ros, normalize, send_zero_burst, stop_on_signals, yaw_from_quaternion)

RATE = 25.0
VICON_TIMEOUT = 0.5  # s without a valid vicon pose before stopping
LEG_MARGIN = 5.0  # s added to the driving time of each leg


class Abort(RuntimeError):
    pass


class YawCalibNode(TimedNode):
    # drives forward, compares the direction of motion with the vicon yaw, drives back

    def __init__(self, args):
        super().__init__('vicon_yaw_calibration')
        self.topic = args.vicon_topic
        self.pose = None
        self.pose_time = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        self.create_subscription(PoseStamped, args.vicon_topic, self.vicon_cb, qos)
        self.sender = TwistSender(self, pub, 0.3, 1.0, 1.0 / RATE)

    def vicon_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        # negated so NaN is rejected too
        if not (abs(math.hypot(q.x, q.y, q.z, q.w) - 1.0) < 0.01 and math.isfinite(p.x + p.y)):
            return
        self.pose = (p.x, p.y, yaw_from_quaternion(q))
        self.pose_time = time.time()

    def send(self, vx=0.0, wz=0.0):
        self.sender.send(vx, None, wz)

    def tick(self):
        time.sleep(1.0 / RATE)
        rclpy.spin_once(self, timeout_sec=0.005)
        if self.pose_time is None or time.time() - self.pose_time > VICON_TIMEOUT:
            raise Abort(f'no valid vicon pose on {self.topic} for {VICON_TIMEOUT} s')
        return self.pose


def drive(node, vx, yaw0, timeout, done):
    # straight line holding yaw0 until done(x, y)
    t0 = time.time()
    while time.time() - t0 < timeout:
        x, y, yaw = node.tick()
        if done(x, y):
            node.send()
            return x, y, yaw
        node.send(vx, clamp(1.5 * normalize(yaw0 - yaw), -0.3, 0.3))
    raise Abort(f'leg not finished in {timeout:.0f} s')


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--vicon-topic', default='/vicon/Dingo/Dingo')
    p.add_argument('--cmd-vel-topic', default='/do150_0092/cmd_vel')
    p.add_argument('--distance', type=float, default=0.12)
    p.add_argument('--speed', type=float, default=0.05)
    p.add_argument('--save-file', default=DEFAULT_YAW_OFFSET_FILE)
    return p.parse_args()


def main():
    args = parse_args()
    init_ros()
    node = YawCalibNode(args)
    log = node.get_logger()
    stop_on_signals()
    exit_code = 1
    try:
        t0 = time.time()
        while node.pose is None and time.time() - t0 < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is None:
            raise Abort(f'no vicon pose on {args.vicon_topic}')
        x0, y0, yaw0 = node.tick()
        time.sleep(0.5)
        xs, ys, _ = node.tick()
        if math.hypot(xs - x0, ys - y0) > 0.005:
            raise Abort('robot not still')
        log.info(f'start ({x0:+.3f}, {y0:+.3f}) vicon yaw {math.degrees(yaw0):+.1f} deg')

        timeout = args.distance / args.speed + LEG_MARGIN
        x1, y1, yaw1 = drive(node, args.speed, yaw0, timeout,
                             lambda x, y: math.hypot(x - x0, y - y0) >= args.distance)
        bearing = math.atan2(y1 - y0, x1 - x0)
        off = math.degrees(normalize(bearing - (yaw0 + 0.5 * normalize(yaw1 - yaw0))))
        snap = round(off / 90.0) * 90.0
        residual = off - snap
        snap = 180.0 if snap == -180.0 else snap
        log.info(f'moved {math.hypot(x1 - x0, y1 - y0):.3f} m at bearing '
                 f'{math.degrees(bearing):+.1f} deg: offset {off:+.1f} deg, snapped {snap:+.0f}')

        xe, ye, _ = drive(node, -args.speed, yaw0, timeout, lambda x, y: (
            (x - x0) * math.cos(bearing) + (y - y0) * math.sin(bearing) <= 0.0))
        log.info(f'back at start within {math.hypot(xe - x0, ye - y0) * 1000:.0f} mm')

        if abs(residual) > 20.0:
            raise Abort(f'residual {residual:+.1f} deg is not clean, nothing saved')
        if abs(snap) == 90.0:
            log.warning('rigid body is rotated by 90 deg, redefine it in Tracker')
        with open(args.save_file, 'w') as f:
            json.dump({'offset_deg': snap, 'measured_deg': round(off, 1),
                       'snap_residual_deg': round(residual, 1), 'vicon_topic': args.vicon_topic,
                       'created': time.strftime('%Y-%m-%d %H:%M:%S')}, f, indent=2)
        log.info(f'saved {args.save_file}')
        exit_code = 0
    except Abort as exc:
        log.error(str(exc))
    except KeyboardInterrupt:
        log.warning('stopping robot')
    finally:
        ignore_stop_signals()
        node.sender.reset()
        send_zero_burst(node, 10, 0.01)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()