import json
import time
import urllib.error
import urllib.request

NUMBERS = ('target_x', 'target_y', 'target_yaw_deg', 'x', 'y', 'yaw_deg', 'error_mm')


class Refused(Exception):
    def __init__(self, problems):
        super().__init__('; '.join(problems))
        self.problems = problems


class Aborted(Exception):
    def __init__(self, messages, results):
        super().__init__('; '.join(messages))
        self.results = results


def as_row(number, point):
    # (x, y), (x, y, yaw_deg), (x, y, yaw_deg, wait_s) or dict
    if isinstance(point, dict):
        return point
    return dict(zip(('x', 'y', 'yaw_deg', 'wait_s'), point), label=str(number))


def clean(row):
    return {k: (None if v in ('', None) else float(v)) if k in NUMBERS else v
            for k, v in row.items()}


class Robot:

    def __init__(self, url, timeout=10.0):
        self.url = url.rstrip('/')
        self.timeout = timeout

    def call(self, path, body=None):
        request = urllib.request.Request(
            self.url + path, data=None if body is None else json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.load(response)
        except urllib.error.HTTPError as error:
            raise Refused(json.load(error).get('problems', [str(error)])) from None

    def room(self):
        return self.call('/room')

    def status(self):
        return self.call('/status')

    def check(self, points):
        # empty if accepted
        return self.call('/check', {'points': [as_row(n, p) for n, p in enumerate(points, 1)]})[
            'problems']

    def stop(self, session=None):
        # with an id only that session
        self.call('/stop', {'id': session})

    def run(self, points, on_arrival=None, poll=0.3):
        # returns one result per point, on_arrival(point) gets label, x, y, yaw_deg
        session = self.call('/session', {
            'points': [as_row(n, p) for n, p in enumerate(points, 1)],
            'trigger': 'wait' if on_arrival is None else 'client'})['id']
        seen = None
        try:
            while True:
                status = self.status()
                if status.get('id') != session:
                    raise Refused(['session replaced'])
                if status['state'] == 'finished':
                    break
                if status['state'] == 'waiting' and status['at']['label'] != seen:
                    seen = status['at']['label']
                    on_arrival(clean(status['at']))
                    self.call('/next', {'id': session, 'label': seen})
                else:
                    time.sleep(poll)
        except BaseException:
            self.stop(session)  # script failed
            raise
        results = [clean(row) for row in status['results']]
        if status['exit']:
            raise Aborted(status['messages'] or [f'session stopped, exit {status["exit"]}'],
                          results)
        return results

    def goto(self, x, y, yaw_deg=None, wait_s=0.0):
        return self.run([(x, y, yaw_deg, wait_s)])[0]
