# top-down room view, click to send the robot to a specific point (runs vicon_goto.py)

import argparse
import json
import math
import shlex
import signal
import subprocess
import sys
import tempfile
import rclpy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QGridLayout, QLabel, QPushButton, QWidget
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_utils import (add_yaw_offset_arguments, init_ros, normalize, resolve_yaw_offset,
                       yaw_from_quaternion)
from session import check_points

SCALE = 85   # [px/m]
MARGIN = 40  # [px]


class RoomView(QWidget):
    # draws room, keep-out boxes, target and robot
    def __init__(self, gui):
        super().__init__()
        self.gui = gui
        (self.x0, x1), (y0, self.y1) = gui.room['x'], gui.room['y']
        self.setFixedSize(round((x1 - self.x0) * SCALE) + 2 * MARGIN,
                          round((self.y1 - y0) * SCALE) + 2 * MARGIN)
        self.setMouseTracking(True)

    def to_screen(self, x, y):
        return QPointF(MARGIN + (x - self.x0) * SCALE, MARGIN + (self.y1 - y) * SCALE)

    def to_room(self, event):
        return self.x0 + (event.x() - MARGIN) / SCALE, self.y1 - (event.y() - MARGIN) / SCALE

    def box(self, x0, y0, x1, y1):
        return QRectF(self.to_screen(x0, y0), self.to_screen(x1, y1)).normalized()

    def mouseMoveEvent(self, event):
        self.gui.cursor = self.to_room(event)

    def mousePressEvent(self, event):
        self.gui.go_to(*self.to_room(event))

    def paintEvent(self, event):
        gui, p = self.gui, QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        (x0, x1), (y0, y1), c = gui.room['x'], gui.room['y'], gui.clearance
        p.fillRect(self.box(x0, y0, x1, y1), Qt.white)
        p.fillRect(self.box(x0 + c, y0 + c, x1 - c, y1 - c), QColor('#eef3fe'))  # reachable
        p.setPen(QColor('#dde5f5'))
        for x in range(math.ceil(x0), math.floor(x1) + 1):  # 1 m grid
            p.drawLine(self.to_screen(x, y0), self.to_screen(x, y1))
        for y in range(math.ceil(y0), math.floor(y1) + 1):
            p.drawLine(self.to_screen(x0, y), self.to_screen(x1, y))
        p.setPen(QPen(Qt.black, 3))
        p.drawRect(self.box(x0, y0, x1, y1))
        p.setPen(Qt.gray)
        o = self.to_screen(0, 0)  # vicon origin
        p.drawLine(o - QPointF(6, 0), o + QPointF(6, 0))
        p.drawLine(o - QPointF(0, 6), o + QPointF(0, 6))
        p.setBrush(Qt.lightGray)
        for keep_out in gui.room.get('keep_out', []):
            p.drawRect(self.box(*keep_out))
        p.setBrush(Qt.NoBrush)
        if gui.target:
            t = self.to_screen(*gui.target)
            p.setPen(QPen(QColor('#f1b53d'), 2, Qt.DotLine))
            if gui.pose:
                p.drawLine(self.to_screen(*gui.pose[:2]), t)
            p.setPen(QPen(QColor('#f1b53d'), 2))
            p.drawEllipse(t, 9, 9)
        if gui.pose:
            x, y, yaw = gui.pose
            length, width = (side * SCALE for side in gui.robot_size)
            p.translate(self.to_screen(x, y))
            p.rotate(-math.degrees(yaw))  # screen y points down
            p.setPen(Qt.NoPen)
            p.setBrush(QColor('#356ef0'))
            p.drawRect(QRectF(-length / 2, -width / 2, length, width))
            p.setBrush(QColor('#f1b53d'))
            p.drawEllipse(QPointF(length / 4, 0), 5, 5)  # robot front


class RoomGui(QWidget):

    def __init__(self, room, robot_size, start_goto):
        super().__init__()
        self.room, self.robot_size, self.start_goto = room, robot_size, start_goto
        self.clearance = room.get('clearance', 0.5)
        self.pose = self.target = self.cursor = self.proc = None  # pose (x, y, yaw)
        self.view = RoomView(self)
        self.status = QLabel()
        self.coordinates = QLabel()
        self.origin = QPushButton('Return to origin', clicked=lambda: self.go_to(0.0, 0.0))
        self.stop_button = QPushButton('Stop', clicked=self.stop)
        layout = QGridLayout(self)
        layout.addWidget(self.view, 0, 0, 1, 2)
        layout.addWidget(self.status, 1, 0, 1, 2)
        layout.addWidget(self.coordinates, 2, 0, 1, 2)
        layout.addWidget(self.origin, 3, 0)
        layout.addWidget(self.stop_button, 3, 1)
        self.setWindowTitle('room')
        self.say('click to send the robot')
        self.refresh()

    def say(self, text, colour='black'):
        self.status.setText(text)
        self.status.setStyleSheet(f'color: {colour}; font-weight: bold')

    def go_to(self, x, y):
        if self.proc:
            self.say('still moving, press Stop first', 'red')
        elif not self.pose:
            self.say('no vicon pose yet', 'red')
        elif problems := check_points(self.pose[:2], [(x, y, None)], self.room, self.clearance):
            self.say(f'refused: {problems[0]}', 'red')
        else:
            self.target = (x, y)
            self.log = tempfile.TemporaryFile('w+')
            self.proc = self.start_goto(x, y, self.log)
            self.say(f'moving to ({x:+.2f}, {y:+.2f})', 'blue')

    def stop(self):
        if self.proc:
            self.proc.terminate()  # robot side gets SIGHUP and stops
            self.say('stopping', 'blue')

    def refresh(self):
        if self.proc and self.proc.poll() is not None:
            self.log.seek(0)
            last = (self.log.read().splitlines() or [''])[-1]
            if self.proc.returncode == 0:
                self.say('arrived', 'green')
            else:
                self.say(f'not arrived: {last[:90]}', 'red')
            self.proc = self.target = None
        text = ''
        if self.pose:
            x, y, yaw = self.pose
            text += f'robot ({x:+.2f}, {y:+.2f}) {math.degrees(yaw):+.0f} deg'
        if self.cursor:
            text += f'    cursor ({self.cursor[0]:+.2f}, {self.cursor[1]:+.2f})'
        self.coordinates.setText(text)
        self.origin.setEnabled(not self.proc)
        self.stop_button.setEnabled(bool(self.proc))
        self.view.update()

    def closeEvent(self, event):
        self.stop()
        event.accept()


def goto_command(args, x, y):
    # runs vicon_goto.py on the robot with its room.json and yaw calibration
    goto = ['flock', '-n', '--verbose', '/tmp/vicon_goto.lock', 'python3', 'vicon_goto.py',
            '--room', 'room.json', '--vicon-topic', args.vicon_topic,
            '--point', f'{x:.3f}', f'{y:.3f}', *shlex.split(args.goto_args)]
    remote = f'cd aurora3-nav && exec {shlex.join(goto)}'
    return ['ssh', '-tt', '-o', 'BatchMode=yes', '-o', 'LogLevel=ERROR', args.robot,
            'bash', '-ic', shlex.quote(remote)]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--room', required=True)
    p.add_argument('--vicon-topic', default='/vicon/Dingo/Dingo')
    p.add_argument('--robot-size', type=float, nargs=2, default=(0.686, 0.517))
    p.add_argument('--goto-args', default='')
    p.add_argument('--robot', default='robot@10.78.115.148')
    add_yaw_offset_arguments(p, '--vicon-yaw-offset-deg')
    return p.parse_args()


def main():
    args = parse_args()
    with open(args.room) as f:
        room = json.load(f)
    init_ros()
    node = Node('room_gui')
    offset = math.radians(resolve_yaw_offset(node, args, args.vicon_yaw_offset_deg))
    app = QApplication(sys.argv[:1])
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # ctrl-c closes
    gui = RoomGui(room, args.robot_size, lambda x, y, log: subprocess.Popen(
        goto_command(args, x, y), stdin=subprocess.DEVNULL, stdout=log,
        stderr=subprocess.STDOUT))

    def on_vicon(msg):
        q = msg.pose.orientation
        if abs(math.hypot(q.x, q.y, q.z, q.w) - 1.0) < 0.01:
            yaw = normalize(yaw_from_quaternion(q) + offset)
            gui.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    node.create_subscription(PoseStamped, args.vicon_topic, on_vicon, qos_profile_sensor_data)

    def tick():
        rclpy.spin_once(node, timeout_sec=0)
        gui.refresh()

    timer = QTimer(timeout=tick)
    timer.start(40)
    gui.show()
    app.exec_()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()