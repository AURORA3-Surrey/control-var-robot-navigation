# moves robot to points in vicon coordinates
# --point x y [yaw_deg] or --points-file, checked against --room

import argparse
import json
import math
import sys
from motion_commands import MoveTo, Turn, Wait
from motion_node import add_motion_arguments, run_motion
from session import check_points


def make_point(numbers):
    # [x, y] / [x, y, yaw_deg] to (x, y, yaw in radians or None)
    x, y, *yaw = numbers
    if len(yaw) > 1:
        raise ValueError(numbers)
    return x, y, math.radians(yaw[0]) if yaw else None


def read_points_file(path):
    with open(path) as f:
        lines = [line.split('#')[0].replace(',', ' ').split() for line in f]
    return [make_point([float(n) for n in line]) for line in lines if line]


def build_goto_plan(args, home):
    if args.room:
        with open(args.room) as f:
            room = json.load(f)
        problems = check_points(home[:2], args.points, room, room.get('clearance', 0.5))
        if problems:
            sys.exit('\n  ' + '\n  '.join(problems))
    plan = [Wait(0.3)]
    for x, y, yaw in args.points:
        plan.append(MoveTo(x, y, args.retries))
        if yaw is not None:
            plan.append(Turn(yaw))
        plan.append(Wait(args.wait))
    return plan


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--point', type=float, nargs='+', action='append', default=[])
    p.add_argument('--points-file')
    p.add_argument('--room')
    p.add_argument('--no-room-check', action='store_true')
    p.add_argument('--wait', type=float, default=5.0)  # [s] at each point
    p.add_argument('--retries', type=int, default=3)  # extra tries before skipping a point
    add_motion_arguments(p)
    args = p.parse_args()
    try:
        args.points = [make_point(numbers) for numbers in args.point]
        args.points += read_points_file(args.points_file) if args.points_file else []
    except ValueError:
        p.error('a point is x y or x y yaw_deg')
    if not args.points:
        p.error('give --point or --points-file')
    if not args.room and not args.no_room_check:
        p.error('give --room or --no-room-check')
    return args


def main():
    sys.exit(run_motion(parse_args(), build_goto_plan))


if __name__ == '__main__':
    main()
