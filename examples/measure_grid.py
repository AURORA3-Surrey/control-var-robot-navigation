# measures on a grid, robot stands still while record() runs

from aurora3_client import Robot

ADDRESS = 'http://ROBOT_ADDRESS:8000'


def record(point):
    # recording goes here
    print(f'recording at {point["label"]}: x={point["x"]:.3f} y={point["y"]:.3f}')


robot = Robot(ADDRESS)
points = [(x, y, 0) for x in (-2.0, 0.0, 2.0) for y in (-1.0, 1.0)]  # x, y, yaw_deg
problems = robot.check(points)
if problems:
    raise SystemExit('not accepted: ' + '; '.join(problems))
for result in robot.run(points, on_arrival=record):
    print(result['label'], result['status'], result['start'], result['end'])
