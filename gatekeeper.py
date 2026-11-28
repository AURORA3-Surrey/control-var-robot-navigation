# lets people on the local network send sessions, checks sessions and starts run_session.py
# python3 gatekeeper.py --room room.json

import argparse
import copy
import csv
import io
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from session import check_session, describe, parse_session

COLUMNS = ['label', 'x', 'y', 'yaw_deg', 'wait_s']


class Refused(Exception):
    def __init__(self, code, *problems):
        super().__init__(*problems)
        self.code = code
        self.problems = list(problems)


class Gate:

    def __init__(self, room_path, room, folder, runner):
        self.room_path = room_path
        self.room = room
        self.folder = folder
        self.runner = runner
        self.lock = threading.Lock()
        self.closed = False
        self.proc = None
        self.client_paced = False
        self.count = 0
        self.state = 'idle'  # idle, running, waiting, finished
        self.session = {}    # id, labels, at, results, exit, messages
        self.other = []

    def say(self, text):
        print(time.strftime('%H:%M:%S'), text, flush=True)

    def status(self):
        with self.lock:
            return copy.deepcopy(dict(state=self.state, **self.session))

    def check(self, rows):
        # returns csv text
        if not (isinstance(rows, list) and all(isinstance(row, dict) for row in rows)):
            return '', [], ['points must be a list of objects with x and y']
        text = io.StringIO()
        writer = csv.writer(text)
        writer.writerow(COLUMNS)
        writer.writerows([('' if row.get(c) is None else row[c]) for c in COLUMNS] for row in rows)
        points, problems = parse_session(io.StringIO(text.getvalue()))
        return text.getvalue(), points, problems or check_session(points, self.room)

    def start(self, rows, client_paced, source):
        text, points, problems = self.check(rows)
        with self.lock:
            if self.closed:
                raise Refused(403, 'shutting down')
            if self.proc:
                raise Refused(409, 'session is running')
            if problems:
                raise Refused(400, *problems)
            self.count += 1
            sid = time.strftime('%Y%m%d_%H%M%S') + f'_{self.count}'
            base = os.path.join(self.folder, sid)
            with open(base + '.csv', 'w', newline='') as f:
                f.write(text)
            with open(base + '.log', 'w') as log:
                self.proc = subprocess.Popen(
                    [sys.executable, self.runner, base + '.csv', '--room', self.room_path,
                     '--results', base + '_results.csv',
                     '--trigger', 'enter' if client_paced else 'wait'],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, text=True,
                    errors='replace')
            self.client_paced = client_paced
            self.state = 'running'
            self.session = dict(id=sid, labels=[p.label for p in points], at=None, results=[],
                                exit=None, messages=[])
            self.other = []
            threading.Thread(target=self.follow, args=(self.proc, base), daemon=True).start()
        self.say(f'session {sid} from {source}: {describe(points, self.room)}')
        return sid

    def follow(self, proc, base):
        # reads runner events
        for line in proc.stdout:
            try:
                event = json.loads(line)
                with self.lock:
                    self.on_event(event)
            except (ValueError, KeyError, TypeError):
                self.other.append(line.strip())
        code = proc.wait()
        with open(base + '.log', errors='replace') as f:
            tail = f.read().splitlines()[-5:]
        try:
            with open(base + '_results.csv', newline='') as f:
                results = list(csv.DictReader(f))
        except OSError:
            results = []
        with self.lock:
            self.session.update(at=None, exit=code, results=results,
                                messages=(self.other[-5:] + tail) if code else [])
            self.state = 'finished'
            self.proc = None
            sid = self.session['id']
        self.say(f'session {sid} finished, exit status {code}')

    def on_event(self, event):
        if event['event'] == 'arrived':
            self.session['at'] = {k: event[k] for k in ('label', 'x', 'y', 'yaw_deg')}
            if self.client_paced:
                self.state = 'waiting'
            self.say(f'at point {event["label"]}')
        else:
            self.session['at'] = None
            self.state = 'running'
            self.session['results'].append({k: v for k, v in event.items() if k != 'event'})

    def next(self, sid, label):
        with self.lock:
            if self.state != 'waiting' or (sid, label) != (self.session['id'],
                                                           self.session['at']['label']):
                raise Refused(409, 'robot is not waiting')
            try:
                self.proc.stdin.write('\n')
                self.proc.stdin.flush()
            except OSError:
                raise Refused(409, 'the session has ended')
            self.state = 'running'

    def stop(self, sid=None):
        with self.lock:
            stopped = self.proc is not None and sid in (None, self.session['id'])
            if stopped:
                self.proc.send_signal(signal.SIGINT)
            return stopped

    def close(self):
        with self.lock:
            self.closed = True
        self.stop()
        for _ in range(200):
            if not self.proc:
                return
            time.sleep(0.1)
        proc = self.proc
        if proc:
            proc.kill()


def make_handler(gate):
    class Handler(BaseHTTPRequestHandler):
        timeout = 10

        def reply(self, code, body):
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/status':
                self.reply(200, gate.status())
            elif self.path == '/room':
                self.reply(200, {k: gate.room[k] for k in ('x', 'y', 'keep_out', 'clearance')
                                 if k in gate.room})
            else:
                self.reply(404, {'problems': ['unknown path']})

        def do_POST(self):
            try:
                size = int(self.headers.get('Content-Length') or 0)
                if not 0 <= size <= 1000000:
                    raise Refused(413, 'request is too large')
                body = json.loads(self.rfile.read(size) or b'{}')
                if not isinstance(body, dict):
                    raise ValueError
                self.reply(200, self.act(body))
            except ValueError:
                self.reply(400, {'problems': ['body must be json']})
            except Refused as refused:
                self.reply(refused.code, {'problems': refused.problems})
            except Exception:  # e.g. json nested too deep
                self.reply(400, {'problems': ['request could not be read']})

        def act(self, body):
            if self.path == '/check':
                _, points, problems = gate.check(body.get('points'))
                return dict(ok=not problems, problems=problems,
                            summary='' if problems else describe(points, gate.room))
            if self.path == '/session':
                trigger = body.get('trigger', 'wait')
                if trigger not in ('wait', 'client'):
                    raise Refused(400, 'trigger is wait or client')
                return dict(id=gate.start(body.get('points'), trigger == 'client',
                                          self.client_address[0]))
            if self.path == '/next':
                gate.next(body.get('id'), body.get('label'))
                return {}
            if self.path == '/stop':
                return dict(stopped=gate.stop(body.get('id')))
            raise Refused(404, 'unknown path')

        def log_message(self, *args):
            pass

    return Handler


def raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--room', required=True)
    p.add_argument('--port', type=int, default=8000)
    # sessions, results and runner logs are kept here
    p.add_argument('--folder', default=os.path.expanduser('~/aurora3_sessions'))
    p.add_argument('--runner', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'run_session.py'))
    args = p.parse_args()
    with open(args.room) as f:
        room = json.load(f)
    if not {'x', 'y', 'max_speed', 'turn_speed'} <= set(room):
        p.error('room file needs x, y, max_speed and turn_speed')
    os.makedirs(args.folder, exist_ok=True)

    gate = Gate(args.room, room, args.folder, args.runner)
    server = ThreadingHTTPServer(('', args.port), make_handler(gate))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, raise_keyboard_interrupt)
    gate.say(f'port {args.port}, Ctrl-C to stop')
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        gate.close()


if __name__ == '__main__':
    main()
