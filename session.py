# csv header label,x,y,yaw_deg,wait_s, one row per point, x y in vicon coordinates [m]
# (label, yaw_deg and wait_s can be blank)
# room file: {"x": [min, max], "y": [min, max], "keep_out": [[x0, y0, x1, y1], ...],
# "clearance": 0.5, "max_speed": 0.1, "turn_speed": 0.25}

import csv
import math
from collections import namedtuple

PATH_STEP = 0.05    # [m]
DEFAULT_WAIT = 5.0  # [s]

Point = namedtuple('Point', 'x y yaw wait label')  # yaw in radians or None


def read_session(path):
    with open(path, newline='', encoding='utf-8-sig') as f:
        return parse_session(f)


def parse_session(lines):
    # returns points and issues found
    points, problems, labels = [], [], set()
    reader = csv.DictReader(lines, skipinitialspace=True)
    if not {'x', 'y'} <= set(reader.fieldnames or []):
        return [], ['the first line must be the header label,x,y,yaw_deg,wait_s']
    for line, row in enumerate(reader, 2):
        label = (row.get('label') or '').strip() or str(line - 1)
        yaw, wait = (row.get('yaw_deg') or '').strip(), (row.get('wait_s') or '').strip()
        try:
            numbers = [float(row['x']), float(row['y']),
                       float(yaw) if yaw else 0.0, float(wait) if wait else DEFAULT_WAIT]
        except (KeyError, TypeError, ValueError):
            numbers = [math.nan]
        if not all(map(math.isfinite, numbers)):
            problems.append(f'line {line}: x, y, yaw_deg and wait_s must be numbers')
        elif not label.isprintable():
            problems.append(f'line {line}: the label has control characters')
        elif label in labels:
            problems.append(f'line {line}: the label {label} is used twice')
        else:
            points.append(Point(numbers[0], numbers[1],
                                math.radians(numbers[2]) if yaw else None, numbers[3], label))
            labels.add(label)
    if not points and not problems:
        problems.append('the session has no points')
    return points, problems


def near_box(x, y, box, clearance):
    x0, x1 = sorted(box[0::2])
    y0, y1 = sorted(box[1::2])
    return x0 - clearance <= x <= x1 + clearance and y0 - clearance <= y <= y1 + clearance


def check_points(start, points, room, clearance, labels=None):
    (xmin, xmax), (ymin, ymax) = room['x'], room['y']
    problems = []
    px, py = start
    for number, (x, y, *_) in enumerate(points, 1):
        name = labels[number - 1] if labels else number
        if not (xmin + clearance <= x <= xmax - clearance
                and ymin + clearance <= y <= ymax - clearance):
            problems.append(f'point {name} ({x:.2f}, {y:.2f}) is outside the room')
            continue  # path not checked
        steps = max(int(math.hypot(x - px, y - py) / PATH_STEP), 1)
        for i in range(steps + 1):
            cx, cy = px + (x - px) * i / steps, py + (y - py) * i / steps
            if any(near_box(cx, cy, box, clearance) for box in room.get('keep_out', [])):
                problems.append(f'the move to point {name} passes a keep-out box')
                break
        px, py = x, y
    return problems


def check_session(points, room):
    # room problems of points, first move from robot start is checked when it runs
    start = points[0][:2]
    return check_points(start, points, room, room.get('clearance', 0.5), [p.label for p in points])


def describe(points, room):
    path = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:]))
    waits = sum(p.wait for p in points)
    speed = room.get('max_speed', 0.1)
    minutes = (path / speed + waits) / 60
    return (f'{len(points)} points, path {path:.1f} m, waits {waits:.0f} s, '
            f'at least {minutes:.1f} min at {speed} m/s')
