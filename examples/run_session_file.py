# runs a session file, uses aurora3_client.py

import csv
import sys
from aurora3_client import Robot

robot = Robot('http://ROBOT_ADDRESS:8000')
with open(sys.argv[1], newline='') as f:
    points = list(csv.DictReader(f))
problems = robot.check(points)
if problems:
    raise SystemExit('not accepted: ' + '; '.join(problems))
for result in robot.run(points):
    print(result['label'], result['status'], result['x'], result['y'], result['yaw_deg'])
