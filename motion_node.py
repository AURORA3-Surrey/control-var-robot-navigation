# runs a plan of motion commands (motion_commands.py) using the vicon pose
# initialize(), then start_control_loop(plan), control_tick() runs one command at a time

import math
from collections import deque
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_utils import (TimedNode, TwistSender, VelocityFilter, add_yaw_offset_arguments,
                       ignore_stop_signals, init_ros, normalize, require_fresh_yaw_offset,
                       resolve_yaw_offset, send_zero_burst, stop_on_signals,
                       yaw_from_quaternion)
from motion_commands import TrackingCorrection


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
    p.add_argument('--rate', type=float, default=25.0)
    p.add_argument('--max-accel', type=float, default=0.3)
    p.add_argument('--max-ang-accel', type=float, default=0.5)
    p.add_argument('--cmd-vel-topic', default='/do150_0092/cmd_vel')
    p.add_argument('--vicon-topic', default='/vicon/Dingo/Dingo')
    p.add_argument('--vicon-timeout', type=float, default=0.5)
    p.add_argument('--vel-filter-tau', type=float, default=0.08)
    add_yaw_offset_arguments(p, '--vicon-yaw-offset-deg')


class MotionNode(TimedNode):

    def __init__(self, args):
        super().__init__('motion_node')
        self.args = args
        self.yaw_offset_deg = resolve_yaw_offset(self, args, args.vicon_yaw_offset_deg)
        self.velocity = VelocityFilter(args.vel_filter_tau)
        self.x = self.y = self.yaw = 0.0
        self.have_pose = False
        self.home = None

        self.plan = deque()      # queue of command objects
        self.current_cmd = None  # command being executed

        # filled by on_vicon()
        self.pose_seq = 0
        self.last_pose_time = None
        self.pose_stamp = None
        self.speed = 0.0         # [m/s]
        self.yaw_rate = 0.0      # [rad/s]
        self.aborted = False
        self.is_finished = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(TwistStamped, args.cmd_vel_topic, qos)
        self.create_subscription(PoseStamped, args.vicon_topic, self.on_vicon, qos)
        self.timer_period = 1.0 / self.args.rate
        self.control_timer = None
        self.sender = TwistSender(
            self, self.pub, args.max_accel, args.max_ang_accel, self.timer_period)
        self.correction = TrackingCorrection(self.get_time_sec, args)

    def on_vicon(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = normalize(
            yaw_from_quaternion(msg.pose.orientation) + math.radians(self.yaw_offset_deg))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # velocity from filtered finite differences of vicon samples
        self.velocity.update(x, y, yaw, stamp)
        self.speed = self.velocity.v
        self.yaw_rate = self.velocity.wz
        self.x = x
        self.y = y
        self.yaw = yaw
        self.have_pose = True
        self.last_pose_time = self.get_time_sec()
        self.pose_stamp = stamp
        self.pose_seq += 1

    def pose(self):
        return self.x, self.y, self.yaw

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
        self.get_logger().info(f'waiting for {self.args.vicon_topic}')
        deadline = self.get_time_sec() + 10.0
        while rclpy.ok() and not self.have_pose and self.get_time_sec() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.have_pose:
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

        if self.last_pose_time is not None and now - self.last_pose_time > self.args.vicon_timeout:
            self.abort(f'vicon pose stale for {now - self.last_pose_time:.1f}s')
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
