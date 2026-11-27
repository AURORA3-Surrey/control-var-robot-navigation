# runs a plan of motion commands (motion_commands.py) on a pose combined from odometry and vicon
# initialize(), then start_control_loop(plan), control_tick() runs one command at a time

import math
from collections import deque
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_utils import (PoseFusion, TimedNode, TwistSender, add_yaw_offset_arguments,
                       ignore_stop_signals, init_ros, normalize,
                       require_fresh_yaw_offset, resolve_yaw_offset, send_zero_burst,
                       stop_on_signals, yaw_from_quaternion)
from motion_commands import TrackingCorrection

VICON_FRESH = 0.2  # [s] vicon sample older than this does not confirm pose

def add_motion_arguments(p):
    p.add_argument('--max-speed', type=float, default=0.1)
    p.add_argument('--turn-speed', type=float, default=0.25)
    p.add_argument('--corr-gain', type=float, default=1.5)
    p.add_argument('--corr-damping-gain', type=float, default=0.3)
    p.add_argument('--corr-filter-tau', type=float, default=0.08)
    p.add_argument('--corr-max-d-error', type=float, default=1.0)
    p.add_argument('--position-tol', type=float, default=0.005)
    p.add_argument('--angle-tol', type=float, default=0.0087)
    p.add_argument('--homing-timeout', type=float, default=3.0)
    p.add_argument('--max-tracking-error', type=float, default=0.3)
    p.add_argument('--rate', type=float, default=25.0)
    p.add_argument('--max-accel', type=float, default=0.3)
    p.add_argument('--max-ang-accel', type=float, default=0.5)
    p.add_argument('--cmd-vel-topic', default='/do150_0092/cmd_vel')
    p.add_argument('--odom-topic', default='/do150_0092/platform/odom')
    p.add_argument('--odom-timeout', type=float, default=0.5)
    p.add_argument('--vicon-topic', default='/vicon/Dingo/Dingo')
    # longest time without vicon messg before aborting
    p.add_argument('--vicon-timeout', type=float, default=3.0)
    p.add_argument('--fuse-tau', type=float, default=0.2)
    p.add_argument('--max-vicon-jump', type=float, default=0.1)
    p.add_argument('--max-vicon-yaw-jump', type=float, default=0.2)
    add_yaw_offset_arguments(p, '--vicon-yaw-offset-deg')


class MotionNode(TimedNode):

    def __init__(self, args):
        super().__init__('motion_node')
        self.args = args
        self.yaw_offset_deg = resolve_yaw_offset(self, args, args.vicon_yaw_offset_deg)
        self.fusion = PoseFusion(args.fuse_tau, args.max_vicon_jump, args.max_vicon_yaw_jump)
        self.home = None

        self.plan = deque()      # queue of command objects
        self.current_cmd = None  # command being executed

        # filled by on_odom()
        self.pose_seq = 0
        self.last_odom_time = None
        self.pose_stamp = None
        self.speed = 0.0         # [m/s]
        self.yaw_rate = 0.0      # [rad/s]
        self.aborted = False
        self.is_finished = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        feed_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(PoseStamped, args.vicon_topic, self.on_vicon, feed_qos)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, feed_qos)
        self.timer_period = 1.0 / self.args.rate
        self.control_timer = None
        self.sender = TwistSender(
            self, self.pub, args.max_accel, args.max_ang_accel, self.timer_period)
        self.correction = TrackingCorrection(self.get_time_sec, args)

    def on_odom(self, msg):
        p = msg.pose.pose
        twist = msg.twist.twist
        x, y, yaw = p.position.x, p.position.y, yaw_from_quaternion(p.orientation)
        values = (x, y, yaw, twist.linear.x, twist.linear.y, twist.angular.z)
        if not all(map(math.isfinite, values)):
            return
        self.fusion.update_odom(x, y, yaw)
        self.speed = math.hypot(twist.linear.x, twist.linear.y)
        self.yaw_rate = twist.angular.z
        self.last_odom_time = self.get_time_sec()
        self.pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose_seq += 1

    def on_vicon(self, msg):
        q = msg.pose.orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        # wait for the first odometry message
        if self.fusion.odom is None or not abs(norm - 1.0) < 0.01:
            return
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = normalize(yaw_from_quaternion(q) + math.radians(self.yaw_offset_deg))
        if not self.fusion.update_vicon(x, y, yaw, self.get_time_sec()):
            self.get_logger().warning('vicon pose rejected', throttle_duration_sec=1.0)

    def pose(self):
        return self.fusion.pose()

    def verified_pose(self):
        # last accepted vicon pose if recent else None
        if self.get_time_sec() - self.fusion.vicon_time > VICON_FRESH:
            return None
        return self.fusion.vicon

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        self.sender.send(vx, vy, wz)

    def abort(self, reason):
        self.get_logger().error(f'{reason}, aborting')
        self.aborted = True
        self.is_finished = True

    def correct(self, axis, error, max_speed, tol):
        return self.correction.compute(axis, error, max_speed, tol, self.pose_seq, self.pose_stamp)

    def initialize(self):
        # wait for pose, save starting pose
        self.get_logger().info(f'waiting for {self.args.odom_topic} and {self.args.vicon_topic}')
        deadline = self.get_time_sec() + 10.0
        while rclpy.ok() and self.fusion.anchor is None and self.get_time_sec() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.fusion.odom is None:
            raise RuntimeError(f'no odometry on {self.args.odom_topic} within 10s')
        if self.fusion.anchor is None:
            raise RuntimeError(f'no vicon pose on {self.args.vicon_topic} within 10s')

        self.home = self.pose()
        self.get_logger().info(f'home: x={self.home[0]:.3f} y={self.home[1]:.3f} '
                               f'yaw={math.degrees(self.home[2]):+.2f} deg')

    def start_control_loop(self, plan):
        self.get_logger().info(f'{len(plan)} commands planned')
        self.plan.clear()
        self.plan.extend(plan)
        self.control_timer = self.create_timer(self.timer_period, self.control_tick)

    def control_tick(self):
        if self.is_finished:
            return

        now = self.get_time_sec()

        if now - self.last_odom_time > self.args.odom_timeout:
            self.abort('odometry stale')
            return
        if now - self.fusion.vicon_time > self.args.vicon_timeout:
            self.abort('vicon pose missing or rejected')
            return

        # if no current command get next from plan
        if self.current_cmd is None:
            if not self.plan:
                fx, fy, fyaw = self.pose()
                self.get_logger().info(
                    f'final at ({fx:+.3f}, {fy:+.3f}) yaw {math.degrees(fyaw):+.1f} deg; '
                    f'dx={fx - self.home[0]:+.3f}m dy={fy - self.home[1]:+.3f}m '
                    f'dyaw={math.degrees(normalize(fyaw - self.home[2])):+.2f} deg')
                self.get_logger().info('Complete!')
                self.send(0.0, 0.0, 0.0)
                self.is_finished = True
                return

            self.current_cmd = self.plan.popleft()
            self.current_cmd.start_time = now
            self.current_cmd.start_pose = self.pose()
            self.correction.reset()
            self.get_logger().info(f"executing {self.current_cmd.name}")

        if self.current_cmd.step(self, now):
            self.current_cmd = None


def run_motion(args, build_plan):
    # check the yaw offset, create the node, run build_plan(args, home), finish with a zero burst
    # returns exit status, 0 only if the plan finished
    require_fresh_yaw_offset(args, args.vicon_yaw_offset_deg)
    init_ros()
    node = MotionNode(args)
    stop_on_signals()

    try:
        node.initialize()
        node.start_control_loop(build_plan(args, node.home))
        while rclpy.ok() and not node.is_finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().warning('stop requested')
        node.aborted = True
    except Exception as exc:
        node.get_logger().error(f'{exc}')
        raise
    finally:
        node.is_finished = True
        ignore_stop_signals()
        node.sender.reset()
        send_zero_burst(node, 10, 0.02)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 1 if node.aborted else 0
