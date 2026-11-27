# motion commands for grid runners, commands follows a minimum-jerk position profile
# step(node, now) is called once per control tick and returns True when done
# node is the GridMotionNode which provides 'args', 'pose()', 'send()', 'tracking_correction()',
# 'label' if 'vicon' or 'odom', 'speed', 'yaw_rate', 'home' and the 'plan' queue

from nav_utils import clamp, normalize
import math

PEAK_FACTOR = 35.0/16.0    # velocity_factor at peak (tau=0.5)
HEADING_GAIN = 1.8         # yaw P-gain for heading hold
HEADING_FIX_LIMIT = 0.5    # heading_fix clamp


def world_to_body(vx_world, vy_world, yaw):
    # rotate world velocity into robot body frame
    c, s = math.cos(yaw), math.sin(yaw)
    vx_body = vx_world * c + vy_world * s
    vy_body = -vx_world * s + vy_world * c
    return vx_body, vy_body


def position_factor(tau):
    tau = clamp(tau, 0.0, 1.0)
    return 35 * tau**4 - 84 * tau**5 + 70 * tau**6 - 20 * tau**7


def velocity_factor(tau):
    tau = clamp(tau, 0.0, 1.0)
    return 140 * tau**3 - 420 * tau**4 + 420 * tau**5 - 140 * tau**6


def duration_for_peak(distance, max_speed):
    distance = abs(distance)
    if distance <= 0 or max_speed <= 0:
        return 0.0
    return PEAK_FACTOR * distance / max_speed


def heading_hold(node, error):
    # yaw rate that removes error
    limit = HEADING_FIX_LIMIT * node.args.turn_speed
    return clamp(HEADING_GAIN * error, -limit, limit)


class TrackingCorrection:
    # single axis pd correction followed by lp filter

    def __init__(self, clock, args):
        self.clock = clock  # node time in seconds
        self.a = args
        self.state = {}     # axis to (error, stamp, now, seq) of the last sample used for d error
        self.filtered = {}  # axis to (value, time) of the last filter output

    def reset(self):
        self.state.clear()
        self.filtered.clear()

    def compute(self, axis, error, max_speed, tol, pose_seq, pose_stamp):
        gain, damping_gain = self.a.corr_gain, self.a.corr_damping_gain
        now = self.clock()
        seq = pose_seq
        stamp = pose_stamp if pose_stamp is not None else now
        prev = self.state.get(axis)
        d_error = 0.0

        if prev is None:
            self.state[axis] = (error, stamp, now, seq)
        else:
            prev_error, prev_stamp, prev_now, prev_seq = prev
            if seq != prev_seq:
                dt = stamp - prev_stamp if stamp > prev_stamp else now - prev_now
                if dt > 1e-6:
                    d_error = (error - prev_error) / dt
                    d_error = clamp(d_error, -self.a.corr_max_d_error, self.a.corr_max_d_error)
                self.state[axis] = (error, stamp, now, seq)

        if abs(error) <= tol:
            raw_target = 0.0
        else:
            raw_target = gain * error + damping_gain * d_error
            raw_target = clamp(raw_target, -max_speed, max_speed)

        prev_filt = self.filtered.get(axis)
        if prev_filt is None or self.a.corr_filter_tau <= 0:
            filtered = raw_target
        else:
            prev_val, prev_t = prev_filt
            dt_f = now - prev_t
            alpha = clamp(dt_f / (self.a.corr_filter_tau + dt_f), 0.0, 1.0)
            filtered = prev_val + (raw_target - prev_val) * alpha

        self.filtered[axis] = (filtered, now)
        return filtered


class Command:
    name = ''
    start_time = 0.0
    start_pose = (0.0, 0.0, 0.0)  # set by the node when the command is started

    def step(self, node, now):
        raise NotImplementedError


class Wait(Command):
    name = 'wait'

    def __init__(self, duration):
        self.duration = duration

    def step(self, node, now):
        elapsed = now - self.start_time
        node.send(0.0, 0.0, 0.0)
        return elapsed >= self.duration


class Turn(Command):
    # rotate to absolute yaw
    name = 'turn'

    def __init__(self, target_yaw):
        self.target_yaw = target_yaw
        self.logged = False

    def step(self, node, now):
        a = node.args
        elapsed = now - self.start_time
        x, y, yaw = node.pose()
        start_yaw = self.start_pose[2]
        angle = normalize(self.target_yaw - start_yaw)

        if not self.logged:
            self.logged = True
            node.get_logger().info(
                f"turn cmd {math.degrees(angle):+.1f} deg "
                f"(start yaw {math.degrees(start_yaw):+.1f}) "
                f"-> target {node.label} yaw {math.degrees(normalize(self.target_yaw)):+.1f} deg")

        T = duration_for_peak(angle, a.turn_speed)
        if T <= 0:
            return True

        homing = elapsed >= T
        tau = min(elapsed / T, 1.0)
        w_ff = 0.0 if homing else (angle / T) * velocity_factor(tau)

        turned = normalize(yaw - start_yaw)
        expected = angle * position_factor(tau)
        error = normalize(expected - turned)

        w_correction = node.correct('turn', error, a.turn_speed, a.angle_tol)
        node.send(wz=w_ff + w_correction)

        if homing and abs(error) <= a.angle_tol and abs(node.yaw_rate) < 0.05:
            node.get_logger().info(
                f"landed {math.degrees(normalize(self.target_yaw - yaw)):+.2f} deg off")
            node.send(0.0, 0.0, 0.0)
            return True
        if homing and elapsed >= T + a.homing_timeout:
            node.get_logger().warning(f'turn residual {math.degrees(error):.2f} deg at timeout')
            return True
        return False


class MoveTo(Command):
    # straight line to a point in the world frame
    name = 'move_to'

    def __init__(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.logged = False

    def step(self, node, now):
        a = node.args
        elapsed = now - self.start_time
        x, y, yaw = node.pose()
        start_x, start_y, start_yaw = self.start_pose

        if not self.logged:
            self.logged = True
            node.get_logger().info(
                f"move_to {node.label} ({start_x:+.3f}, {start_y:+.3f}) -> "
                f"({self.target_x:+.3f}, {self.target_y:+.3f})")
        dx = self.target_x - start_x
        dy = self.target_y - start_y
        T = max(duration_for_peak(dx, a.max_speed), duration_for_peak(dy, a.max_speed))
        if T <= 0:
            return True

        homing = elapsed >= T
        tau = min(elapsed / T, 1.0)
        vf = 0.0 if homing else velocity_factor(tau)
        pf = position_factor(tau)

        expected_x, expected_y = start_x + dx * pf, start_y + dy * pf
        err_x, err_y = expected_x - x, expected_y - y

        vx_corr = node.correct('move_x', err_x, a.max_speed, a.position_tol)
        vy_corr = node.correct('move_y', err_y, a.max_speed, a.position_tol)

        vx_world = (dx / T) * vf + vx_corr
        vy_world = (dy / T) * vf + vy_corr
        vx_body, vy_body = world_to_body(vx_world, vy_world, yaw)
        heading_fix = heading_hold(node, normalize(start_yaw - yaw))

        node.send(vx=vx_body, vy=vy_body, wz=heading_fix)

        if homing and math.hypot(err_x, err_y) <= a.position_tol and abs(node.speed) < 0.02:
            node.send(0.0, 0.0, 0.0)
            return True
        if elapsed >= T + a.homing_timeout:
            node.get_logger().warning(
                f'move_to residual {math.hypot(err_x, err_y):.3f}m at timeout')
            return True
        return False
