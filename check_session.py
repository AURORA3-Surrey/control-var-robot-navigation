import argparse
import json
import sys
from session import check_session, describe, read_session


def main():
    p = argparse.ArgumentParser()
    p.add_argument('session')
    p.add_argument('--room', required=True)
    args = p.parse_args()
    with open(args.room) as f:
        room = json.load(f)
    points, problems = read_session(args.session)
    if not problems:
        problems = check_session(points, room)
    if problems:
        sys.exit('session not accepted:\n  ' + '\n  '.join(problems))
    print('session accepted')
    print(describe(points, room))


if __name__ == '__main__':
    main()
