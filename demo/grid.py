# grid demo: goes to grid of points starting at the current pose, one move_to per point
# (the motion is in motion_node.py and motion_commands.py)
# requires yaw calibration (vicon_yaw_calibration.py) unless bypassed

import argparse
from motion_commands import MoveTo, Turn, Wait
from motion_node import add_motion_arguments, run_motion


def add_grid_arguments(p):
    # rows (height points) width set by --grid-cols
    p.add_argument('--grid-size', type=int, default=4)
    # columns (width points) default = --grid-size (square)
    p.add_argument('--grid-cols', type=int, default=None)
    p.add_argument('--cell-size', type=float, default=0.25)
    p.add_argument('--cell-size-x', type=float, default=None)
    p.add_argument('--cell-size-y', type=float, default=None)
    p.add_argument('--no-return-home', action='store_true')


def grid_dimensions(args):
    # (rows, cols, cell size along x, cell size along y)
    rows = args.grid_size
    cols = args.grid_cols if args.grid_cols is not None else args.grid_size
    csx = args.cell_size_x if args.cell_size_x is not None else args.cell_size
    csy = args.cell_size_y if args.cell_size_y is not None else args.cell_size
    return rows, cols, csx, csy


def build_grid_plan(args, home):
    # path over the grid starting at home = (x, y, yaw) then back
    rows, cols, csx, csy = grid_dimensions(args)
    hx, hy, hyaw = home
    plan = [Wait(0.3)]

    waypoints = []
    for row in range(rows):
        wy = hy + row * csy
        col_order = range(cols) if row % 2 == 0 else reversed(range(cols))
        for col in col_order:
            waypoints.append((hx + col * csx, wy))
    for wx, wy in waypoints[1:]:
        plan.append(MoveTo(wx, wy))

    if not args.no_return_home:
        plan.append(MoveTo(hx, hy))
        plan.append(Turn(hyaw))

    plan.append(Wait(0.8))
    return plan


def parse_args():
    p = argparse.ArgumentParser()
    add_grid_arguments(p)
    add_motion_arguments(p)
    return p.parse_args()


def main():
    run_motion(parse_args(), build_grid_plan)


if __name__ == '__main__':
    main()
