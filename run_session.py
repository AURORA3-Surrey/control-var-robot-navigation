# runs a session file on the robot and writes results csv
# python3 run_session.py my_session.csv --room room.json

import argparse
import csv
import json
import math
import os
import select
import sys
import time
from datetime import datetime, timezone
from motion_commands import Command, MoveTo, Turn, Wait
from motion_node import add_motion_arguments, run_motion
from session import check_points, check_session, read_session

COLUMNS = ['label', 'status', 'target_x', 'target_y', 'target_yaw_deg', 'x', 'y', 'yaw_deg',
           'error_mm', 'start', 'end']


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def emit(**event):
    print(json.dumps(event), flush=True)


class Results:
    # results file row is written when a point is done

    def __init__(self, path):
        self.file = open(path, 'w', newline='')
        self.writer = csv.DictWriter(self.file, COLUMNS)
        self.writer.writeheader()
        self.file.flush()
        self.labels = set()

    def add(self, point, status, pose=None, start='', end=''):
        yaw = '' if point.yaw is None else f'{math.degrees(point.yaw):.2f}'
        row = dict(label=point.label, status=status, target_x=str(point.x), target_y=str(point.y),
                   target_yaw_deg=yaw, start=start, end=end)
        if pose:
            row.update(x=f'{pose[0]:.4f}', y=f'{pose[1]:.4f}',
                       yaw_deg=f'{math.degrees(pose[2]):.2f}',
                       error_mm=f'{1000 * math.hypot(pose[0] - point.x, pose[1] - point.y):.1f}')
        self.writer.writerow(row)
        self.file.flush()
        self.labels.add(point.label)
        emit(event='point', **row)

    def close(self, points, status):
        for point in points:
            if point.label not in self.labels:
                self.add(point, status)
        self.file.close()


class Hold(Command):
    # stands still and writes its row,

    name = 'hold'

    def __init__(self, point, results, enter):
        self.point = point
        self.results = results
        self.enter = enter
        self.utc_start = None
        self.pose = None

    def stdin_ready(self):
        return bool(select.select([sys.stdin], [], [], 0)[0])

    def step(self, node, now):
        node.send(0.0, 0.0, 0.0)
        if self.utc_start is None:
            while self.enter and self.stdin_ready() and os.read(sys.stdin.fileno(), 1024):
                pass
            self.utc_start = utc_now()
            self.pose = node.fusion.vicon
            emit(event='arrived', label=self.point.label, x=self.pose[0], y=self.pose[1],
                 yaw_deg=math.degrees(self.pose[2]))
            if self.enter:
                node.get_logger().info(f'point {self.point.label}: press Enter to continue')
        if self.enter:
            done = self.stdin_ready()
            if done and not os.read(sys.stdin.fileno(), 1024):
                node.abort('stdin closed')
                return True
        else:
            done = now - self.start_time >= self.point.wait
        if done:
            self.results.add(self.point, 'reached', self.pose, self.utc_start, utc_now())
        return done

    def skip(self):
        self.results.add(self.point, 'skipped')


def build_session_plan(args, home):
    problems = check_points(home[:2], args.points, args.room, args.clearance,
                            [p.label for p in args.points])
    if problems:
        sys.exit('\n  ' + '\n  '.join(problems))
    plan = [Wait(0.3)]
    for point in args.points:
        plan.append(MoveTo(point.x, point.y, args.retries))
        if point.yaw is not None:
            plan.append(Turn(point.yaw))
        plan.append(Hold(point, args.results, args.trigger == 'enter'))
    return plan


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('session')
    p.add_argument('--room', required=True)
    p.add_argument('--results')
    p.add_argument('--trigger', choices=('wait', 'enter'), default='wait')
    p.add_argument('--retries', type=int, default=3)  # extra tries before skipping a point
    add_motion_arguments(p)
    args = p.parse_args()
    with open(args.room) as f:
        args.room = json.load(f)
    try:
        args.max_speed, args.turn_speed = args.room['max_speed'], args.room['turn_speed']
    except KeyError:
        p.error('room file needs max_speed and turn_speed')
    args.clearance = args.room.get('clearance', 0.5)
    args.points, problems = read_session(args.session)
    problems = problems or check_session(args.points, args.room)
    if problems:
        sys.exit('\n  ' + '\n  '.join(problems))
    args.results = Results(args.results or os.path.splitext(args.session)[0]
                           + time.strftime('_results_%Y%m%d_%H%M%S.csv'))
    return args


def main():
    args = parse_args()
    status = 1
    try:
        status = run_motion(args, build_session_plan)
    finally:
        args.results.close(args.points, 'not run')
    sys.exit(0 if status == 2 else status)  # skipped points in results


if __name__ == '__main__':
    main()
