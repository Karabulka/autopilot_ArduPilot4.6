#!/usr/bin/env python3
"""Read-only flight dashboard. Never publishes navigation or control data."""
import bisect
import argparse
from collections import deque
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from python_qt_binding import QtCore, QtGui, QtWidgets
from dashboard_plots import Timeline, ChartPage, GUIDANCE, ACCURACY, command_at

ROOT = Path(os.environ.get('GLIDER_WORKSPACE', '/home/nikol/autopilot'))


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def attitude(q):
    return (math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
            math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x)))),
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


def world_velocity(q, v):
    return ((1-2*(q.y*q.y+q.z*q.z))*v.x + 2*(q.x*q.y-q.z*q.w)*v.y + 2*(q.x*q.z+q.y*q.w)*v.z,
            2*(q.x*q.y+q.z*q.w)*v.x + (1-2*(q.x*q.x+q.z*q.z))*v.y + 2*(q.y*q.z-q.x*q.w)*v.z)


def cross_track(a, b, p):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length = math.hypot(dx, dy)
    return (dx*(p[1]-a[1])-dy*(p[0]-a[0]))/length if length > 1e-8 else None


def sample(msg):
    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
    vx, vy = world_velocity(q, v)
    return dict(t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                x=p.x, y=p.y, z=p.z, yaw=attitude(q)[2], roll=attitude(q)[0],
                vx=v.x, vy=v.y, wx=vx, wy=vy, speed=math.hypot(vx, vy),
                course=math.atan2(vy, vx) if math.hypot(vx, vy) > 1. else None)


def interpolate(history, stamp):
    """Interpolate truth at the acquisition timestamp; never extrapolate."""
    if len(history) < 2 or stamp < history[0]['t'] or stamp > history[-1]['t']:
        return None
    i = bisect.bisect_left([s['t'] for s in history], stamp)
    if i == 0:
        return dict(history[0])
    a, b = history[i-1], history[i]
    dt = b['t']-a['t']
    if dt <= 0 or dt > .2:
        return None
    f = (stamp-a['t'])/dt
    keys = ('t', 'x', 'y', 'z', 'yaw', 'roll', 'vx', 'vy', 'speed')
    keys += tuple(k for k in ('wx', 'wy') if k in a and k in b)
    return {k: a[k]+f*wrap(b[k]-a[k]) if k in ('yaw', 'roll') else
            a[k]+f*(b[k]-a[k]) for k in keys}


class FlightMonitor:
    def __init__(self, node):
        self.node = node
        self.route = []
        self.route_verified = False
        self.loop = True
        self.nav = self.truth = self.target = None
        self.state = 'Нет данных'
        self.ages = {}
        self.traces = {'nav': deque(maxlen=12000), 'truth': deque(maxlen=12000)}
        self.history = deque(maxlen=400)
        self.pending = deque(maxlen=500)
        self.errors = deque(maxlen=36000)
        self.sample_count = 0
        self.timeline = Timeline()
        self.commands = deque(maxlen=10000)
        self.last_goal = None
        self.switch_count = 0
        self.squares = {k: [0., 0] for k in ('vx', 'vy', 'yaw_deg', 'cte')}
        self.origin = None
        self.path = 0.
        self.last_truth = None
        self.image = None
        self.segment = None
        self.camera_count = 0
        self.subs = [node.create_subscription(Odometry, '/glider/odometry', self.on_nav, qos_profile_sensor_data),
                     node.create_subscription(Odometry, '/evaluation/ground_truth', self.on_truth, qos_profile_sensor_data),
                     node.create_subscription(PoseStamped, '/glider/target_pose', self.on_target, qos_profile_sensor_data),
                     node.create_subscription(String, '/glider/interface_state', self.on_state, qos_profile_sensor_data),
                     node.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)]
        self.client = AsyncParameterClient(node, '/los_controller_node')
        self.future = None
        self.session = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output = ROOT/'log_visual_nav'/('dashboard_'+self.session)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.raw = self.output.with_suffix('.jsonl').open('w')
        self.last_export = time.monotonic()

    def on_nav(self, msg):
        s = sample(msg)
        if not all(math.isfinite(v) for v in s.values() if v is not None):
            return
        if self.nav and s['t'] < self.nav['t']:
            self.clear()
        self.nav = s
        self.ages['nav'] = time.monotonic()
        self.traces['nav'].append((s['x'], s['y']))
        self.pending.append((s, self.segment))
        self.record_sample('nav', s)
        self.match()

    def on_truth(self, msg):
        s = sample(msg)
        if not all(math.isfinite(v) for v in s.values() if v is not None):
            return
        if self.truth and s['t'] < self.truth['t']:
            self.clear()
        if self.last_truth:
            self.path += math.hypot(s['x']-self.last_truth['x'], s['y']-self.last_truth['y'])
        self.last_truth = self.truth = s
        self.history.append(s)
        self.ages['truth'] = time.monotonic()
        self.traces['truth'].append((s['x'], s['y']))
        self.record_sample('truth', s)
        self.match()

    def record_sample(self, prefix, s):
        for key, value in [('course', math.degrees(s['course']) if s['course'] is not None else None),
                           ('roll', math.degrees(s['roll'])), ('height', s['z']), ('speed', s['speed'])]:
            self.timeline.add(prefix+'_'+key, s['t'], value)

    def match(self):
        while self.pending and self.history:
            s, segment = self.pending[0]
            if s['t'] > self.history[-1]['t']:
                break
            self.pending.popleft()
            truth = interpolate(self.history, s['t'])
            if truth is None:
                continue
            if self.origin is None:
                self.origin = (truth['x']-s['x'], truth['y']-s['y'])
            dx = s['x']-(truth['x']-self.origin[0])
            dy = s['y']-(truth['y']-self.origin[1])
            command = command_at(self.commands, s['t'])
            segment = command['segment'] if command else None
            cte, nav_cte = None, None
            if segment is not None and self.route_verified:
                a, b = self.route[segment], self.route[(segment+1) % len(self.route)]
                cte = cross_track(a, b, (truth['x']-self.origin[0], truth['y']-self.origin[1]))
                nav_cte = cross_track(a, b, (s['x'], s['y']))
            e = dict(t=s['t'], vx=s['vx']-truth['vx'], vy=s['vy']-truth['vy'],
                     yaw_deg=math.degrees(wrap(s['yaw']-truth['yaw'])), drift=math.hypot(dx, dy), cte=cte)
            self.errors.append(e)
            self.sample_count += 1
            for key, value in [('error_vx', e['vx']), ('error_vy', e['vy']),
                               ('error_yaw', e['yaw_deg']), ('drift', e['drift']),
                               ('nav_cte', nav_cte), ('truth_cte', cte)]:
                self.timeline.add(key, s['t'], value)
            for key, total in self.squares.items():
                if e[key] is not None:
                    total[0] += e[key]*e[key]
                    total[1] += 1
            self.raw.write(json.dumps(dict(nav=s, truth=truth, errors=e, command=command), allow_nan=False)+'\n')

    def on_target(self, msg):
        roll, pitch, course = attitude(msg.pose.orientation)
        t = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not all(math.isfinite(x) for x in (roll, pitch, course, msg.pose.position.x,
                                             msg.pose.position.y, msg.pose.position.z)):
            return
        self.target = msg
        self.ages['target'] = time.monotonic()
        self.update_segment()
        command = dict(t=t, segment=self.segment, roll_deg=math.degrees(roll),
                       course_deg=math.degrees(course), height=msg.pose.position.z)
        if not self.commands or t >= self.commands[-1]['t']:
            self.commands.append(command)
        for key, value in [('desired_course', command['course_deg']), ('desired_roll', command['roll_deg']),
                           ('desired_height', command['height'])]:
            self.timeline.add(key, t, value)
        goal = (msg.pose.position.x, msg.pose.position.y)
        if self.segment is not None and goal != self.last_goal:
            if self.last_goal is not None:
                self.switch_count += 1
            self.last_goal = goal
            self.timeline.event(t, self.segment, f'→{(self.segment+1) % len(self.route)+1}')
        self.raw.write(json.dumps(dict(kind='target', **command, goal=goal), allow_nan=False)+'\n')

    def update_segment(self):
        self.segment = None
        if not self.target or not self.route_verified:
            return
        p = self.target.pose.position
        for i, point in enumerate(self.route):
            if math.hypot(p.x-point[0], p.y-point[1]) < .01 and (self.loop or i > 0):
                self.segment = (i-1) % len(self.route)
                return

    def on_state(self, msg):
        self.state = msg.data
        self.ages['state'] = time.monotonic()

    def on_image(self, msg):
        if msg.encoding not in ('mono8', 'rgb8', 'bgr8'):
            return
        fmt = QtGui.QImage.Format_Grayscale8 if msg.encoding == 'mono8' else QtGui.QImage.Format_RGB888
        image = QtGui.QImage(bytes(msg.data), msg.width, msg.height, msg.step, fmt).copy()
        self.image = image.rgbSwapped() if msg.encoding == 'bgr8' else image
        self.camera_count += 1
        self.ages['camera'] = time.monotonic()

    def poll_parameters(self):
        if self.route_verified:
            return
        if self.future is None and self.client.services_are_ready():
            self.future = self.client.get_parameters(['waypoints', 'loop_waypoints'])
        if self.future and self.future.done():
            try:
                values = self.future.result().values
                points = list(values[0].double_array_value)
                if len(points) >= 4 and len(points) % 2 == 0 and all(math.isfinite(x) for x in points):
                    self.route = list(zip(points[::2], points[1::2]))
                    self.loop = values[1].bool_value
                    self.route_verified = True
                    self.update_segment()
            except Exception as error:
                self.node.get_logger().warning(str(error))
            self.future = None

    def clear(self):
        self.export()
        self.raw.close()
        self.output = ROOT/'log_visual_nav'/('dashboard_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.raw = self.output.with_suffix('.jsonl').open('w')
        self.errors.clear()
        self.sample_count = 0
        self.timeline.__init__()
        self.commands.clear()
        self.last_goal = None
        self.switch_count = 0
        self.squares = {k: [0., 0] for k in self.squares}
        self.pending.clear()
        self.history.clear()
        for trace in self.traces.values():
            trace.clear()
        self.origin = None
        self.path = 0.
        self.last_truth = self.nav = self.truth = None
        self.target = None
        self.segment = None
        self.ages.clear()

    def fresh(self, key):
        return key in self.ages and time.monotonic()-self.ages[key] < 2.

    def summary(self):
        def rmse(key):
            total, count = self.squares[key]
            return math.sqrt(total/count) if count else None
        return dict(samples=self.sample_count, rmse_vx=rmse('vx'), rmse_vy=rmse('vy'),
                    rmse_yaw_deg=rmse('yaw_deg'), rmse_cross_track_m=rmse('cte'),
                    drift_m=self.errors[-1]['drift'] if self.errors else None,
                    truth_path_m=self.path, route=self.route,
                    waypoint_switches=self.switch_count,
                    chart_retention_seconds=300,
                    alignment='First matched XY origin; no yaw/scale fit. Truth is evaluation only.',
                    note='Metrics since dashboard opening/reset; not a full-flight qualification.')

    def export(self):
        if not self.raw.closed:
            self.raw.flush()
        self.output.with_suffix('.json').write_text(json.dumps(self.summary(), ensure_ascii=False, indent=2, allow_nan=False))


class Map(QtWidgets.QWidget):
    def __init__(self, monitor):
        super().__init__()
        self.m = monitor
        self.setMinimumSize(580, 480)

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#111b2a'))
        m = self.m
        truth = [(x-m.origin[0], y-m.origin[1]) for x, y in m.traces['truth']] if m.origin else []
        points = m.route + list(m.traces['nav']) + truth
        if not points:
            p.setPen(QtGui.QColor('white'))
            p.drawText(25, 35, 'Ожидание ROS: маршрут и телеметрия…')
            return
        xmin, xmax = min(x for x, y in points)-20, max(x for x, y in points)+20
        ymin, ymax = min(y for x, y in points)-20, max(y for x, y in points)+20
        scale = min((self.width()-80)/max(100, xmax-xmin), (self.height()-80)/max(100, ymax-ymin))
        cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
        def point(x, y):
            return QtCore.QPointF(self.width()/2+(x-cx)*scale, self.height()/2-(y-cy)*scale)
        p.setPen(QtGui.QColor('#27364a'))
        for x in range(math.floor(xmin/20)*20, math.ceil(xmax/20)*20+1, 20):
            p.drawLine(point(x, ymin), point(x, ymax))
        for y in range(math.floor(ymin/20)*20, math.ceil(ymax/20)*20+1, 20):
            p.drawLine(point(xmin, y), point(xmax, y))
        def line(trace, color, width):
            if len(trace) < 2:
                return
            p.setPen(QtGui.QPen(QtGui.QColor(color), width))
            p.drawPolyline(QtGui.QPolygonF([point(x, y) for x, y in trace]))
        route = m.route + ([m.route[0]] if m.route and m.loop else [])
        line(route, '#e9c46a', 3)
        if m.segment is not None:
            line([m.route[m.segment], m.route[(m.segment+1) % len(m.route)]], '#ffffff', 4)
        for i, (x, y) in enumerate(m.route):
            pos = point(x, y)
            p.setPen(QtGui.QColor('#e9c46a'))
            p.drawEllipse(pos, 5, 5)
            shift = -165 if pos.x() > self.width()-175 else 8
            p.drawText(pos+QtCore.QPointF(shift, -8), f'ППМ {i+1} ({x:g}, {y:g})')
        line(truth, '#f28e65', 2)
        line(list(m.traces['nav']), '#45d4e8', 2)
        if m.nav:
            s = m.nav
            pos = point(s['x'], s['y'])
            p.setPen(QtGui.QPen(QtGui.QColor('#45d4e8'), 2))
            p.setBrush(QtGui.QColor('#45d4e8'))
            p.drawEllipse(pos, 6, 6)
            p.drawLine(pos, pos+QtCore.QPointF(30*math.cos(s['yaw']), -30*math.sin(s['yaw'])))
            if m.target and m.fresh('target'):
                yaw = attitude(m.target.pose.orientation)[2]
                p.setPen(QtGui.QPen(QtGui.QColor('#e9c46a'), 3))
                p.drawLine(pos, pos+QtCore.QPointF(55*math.cos(yaw), -55*math.sin(yaw)))
        if truth:
            p.setPen(QtGui.QPen(QtGui.QColor('#f28e65'), 3))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawEllipse(point(*truth[-1]), 8, 8)
        p.setPen(QtGui.QColor('#dce5ef'))
        p.drawText(15, 22, 'ENU: север ↑, восток → | Сетка 20 м | Масштаб автоматический')
        p.drawText(15, self.height()-15, 'Желтый: маршрут / χd   Голубой: навигация   Оранжевый: Gazebo (эталон)')


class Window(QtWidgets.QMainWindow):
    def __init__(self, monitor):
        super().__init__()
        self.m = monitor
        self.setWindowTitle('Glider Visual Nav — контроль полета без GPS')
        self.resize(1400, 960)
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(widget)
        self.map = Map(monitor)
        layout.addWidget(self.map, 3)
        right = QtWidgets.QVBoxLayout()
        self.info = QtWidgets.QLabel()
        self.info.setMinimumWidth(400)
        self.info.setWordWrap(True)
        right.addWidget(self.info)
        self.camera = QtWidgets.QLabel('Ожидание камеры…')
        self.camera.setMinimumSize(400, 260)
        self.camera.setAlignment(QtCore.Qt.AlignCenter)
        right.addWidget(self.camera)
        export = QtWidgets.QPushButton('Сохранить JSON с метриками')
        export.clicked.connect(self.save)
        right.addWidget(export)
        reset = QtWidgets.QPushButton('Новый интервал измерений (не меняет полет)')
        reset.clicked.connect(monitor.clear)
        right.addWidget(reset)
        note = QtWidgets.QLabel('Окно только наблюдает. Ground truth не передается в управление.\n'
                               'Углы ENU: 0° — восток, +90° — север.\n'
                               'χ — направление скорости; ψ — направление носа.\n'
                               'Метрики считаются с момента открытия / сброса окна.')
        note.setWordWrap(True)
        right.addWidget(note)
        layout.addLayout(right, 2)
        self.view = {'end': None, 'seconds': 60.}
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(widget, 'Обзор: маршрут и камера')
        self.guidance = ChartPage(monitor.timeline, GUIDANCE, self.view)
        self.accuracy = ChartPage(monitor.timeline, ACCURACY, self.view)
        self.tabs.addTab(self.guidance, 'Наведение: курс, крен, высота')
        self.tabs.addTab(self.accuracy, 'Точность навигации')
        self.setCentralWidget(self.tabs)
        toolbar = self.addToolBar('Графики')
        toolbar.setMovable(False)
        toolbar.addWidget(QtWidgets.QLabel('  Интервал графиков: '))
        interval = QtWidgets.QComboBox()
        for seconds in (30, 60, 120, 300):
            interval.addItem(f'{seconds} с', seconds)
        interval.setCurrentIndex(1)
        interval.currentIndexChanged.connect(lambda index: self.view.update(seconds=float(interval.itemData(index))))
        toolbar.addWidget(interval)
        self.pause = QtWidgets.QPushButton('Пауза графиков')
        self.pause.setCheckable(True)
        self.pause.toggled.connect(self.pause_charts)
        toolbar.addWidget(self.pause)
        save = QtWidgets.QPushButton('Сохранить отчет: JSON + CSV + PNG')
        save.clicked.connect(self.save_report)
        toolbar.addWidget(save)
        toolbar.addSeparator()
        toolbar.addWidget(QtWidgets.QLabel('  Пунктир — смена цели ППМ | Наведите мышь для значений'))
        self.health = QtWidgets.QLabel()
        self.statusBar().addPermanentWidget(self.health)
        self.recording = monitor.output
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(200)

    def tick(self):
        m = self.m
        for _ in range(80):
            rclpy.spin_once(m.node, timeout_sec=0)
        m.poll_parameters()
        if self.recording != m.output:
            self.recording = m.output
            self.pause.setChecked(False)
        def fmt(value, digits=1):
            return '—' if value is None else f'{value:.{digits}f}'
        def degrees(value):
            return fmt(math.degrees(value)) if value is not None else '—'
        status = ' | '.join(f'{name}: {"OK" if m.fresh(key) else "НЕТ / УСТАРЕЛО"}'
                            for key, name in [('nav', 'Навигация'), ('truth', 'Эталон'), ('camera', 'Камера')])
        self.health.setText(status)
        text = [f'<h2>{m.state if m.fresh("state") else "Нет свежего состояния PX4"}</h2>', status,
                '<br>Маршрут: '+('получен из LOS' if m.route_verified else 'ожидание параметров LOS')]
        if m.nav:
            s = m.nav
            text += [f'<p>X={s["x"]:.1f}, Y={s["y"]:.1f} м | Высота корпуса={s["z"]:.1f} м<br>'
                     f'Скорость={s["speed"]:.1f} м/с | ψ магнитный={degrees(s["yaw"])}°<br>'
                     f'Крен оцененный={degrees(s["roll"])}°<br>'
                     f'χ фактический (навигация)={degrees(s["course"])}°</p>']
        if m.truth:
            text += [f'χ фактический (Gazebo)={degrees(m.truth["course"])}°<br>']
        if m.target and m.fresh('target'):
            roll, pitch, yaw = attitude(m.target.pose.orientation)
            err = wrap(yaw-m.nav['course']) if m.nav and m.nav['course'] is not None else None
            text += [f'χ желаемый LOS={degrees(yaw)}° | ошибка χ={degrees(err)}°<br>'
                     f'Крен заданный={degrees(roll)}° | тангаж вверх={degrees(-pitch)}°<br>']
        if m.segment is not None and m.nav:
            i = m.segment
            cte = cross_track(m.route[i], m.route[(i+1) % len(m.route)], (m.nav['x'], m.nav['y']))
            text += [f'Участок {i+1} → {(i+1) % len(m.route)+1} | e⊥={fmt(cte)} м<br>']
        summary = m.summary()
        text += ['<h3>Независимая оценка по Gazebo</h3>',
                 f'Сопоставлено по времени: {summary["samples"]} отсчетов<br>',
                 f'RMSE Vx/Vy: {fmt(summary["rmse_vx"], 3)} / {fmt(summary["rmse_vy"], 3)} м/с<br>',
                 f'RMSE ψ: {fmt(summary["rmse_yaw_deg"])}°<br>',
                 f'RMSE e⊥: {fmt(summary["rmse_cross_track_m"])} м<br>',
                 f'Дрейф XY: {fmt(summary["drift_m"])} м']
        self.info.setText(''.join(text))
        if m.image is not None:
            self.camera.setPixmap(QtGui.QPixmap.fromImage(m.image).scaled(
                self.camera.width(), 280, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        if not m.fresh('camera'):
            self.camera.setText('Камера: нет свежих кадров')
        self.map.update()
        if self.tabs.currentWidget() in (self.guidance, self.accuracy):
            self.tabs.currentWidget().refresh()
        if time.monotonic()-m.last_export > 5.:
            m.export()
            m.last_export = time.monotonic()

    def save(self):
        self.m.export()
        self.statusBar().showMessage('Сохранено: '+str(self.m.output.with_suffix('.json')), 12000)

    def pause_charts(self, paused):
        self.view['end'] = self.m.timeline.latest if paused else None
        self.pause.setText('Продолжить графики' if paused else 'Пауза графиков')
        self.statusBar().showMessage('Пауза просмотра; полет и запись продолжаются.' if paused else 'Графики в реальном времени.', 5000)
        self.guidance.refresh()
        self.accuracy.refresh()

    def export_charts(self, prefix):
        self.timer.stop()
        previous = self.tabs.currentIndex()
        try:
            for index, name in ((0, 'overview'), (1, 'guidance'), (2, 'accuracy')):
                self.tabs.setCurrentIndex(index)
                QtWidgets.QApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents)
                path = str(prefix)+'_'+name+'.png'
                if not self.tabs.currentWidget().grab().save(path):
                    raise OSError('Cannot save '+path)
        finally:
            self.tabs.setCurrentIndex(previous)
            self.timer.start(200)

    def save_report(self):
        self.m.export()
        prefix = self.m.output.parent/(self.m.output.name+'_snapshot_'+datetime.now().strftime('%H%M%S_%f'))
        self.m.timeline.export_csv(prefix.with_suffix('.csv'))
        report = self.m.summary()
        report['charts'] = dict(window_seconds=self.view['seconds'], paused=self.pause.isChecked(),
                                end_sim_time=self.view['end'] if self.view['end'] is not None else self.m.timeline.latest,
                                origin_sim_time=self.m.timeline.origin,
                                units={key: spec[1] for spec in GUIDANCE+ACCURACY for key, label, color in spec[2]})
        prefix.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        self.export_charts(prefix)
        self.statusBar().showMessage('Сохранены JSON, CSV и три PNG: '+str(prefix), 15000)

    def closeEvent(self, event):
        self.m.export()
        self.m.timeline.export_csv(self.m.output.with_suffix('.csv'))
        self.m.raw.close()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-check', action='store_true')
    parser.add_argument('--require-flight', action='store_true')
    parser.add_argument('--check-seconds', type=float, default=12.)
    args = parser.parse_args()
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setStyle('Fusion')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('read_only_flight_dashboard')
    monitor = FlightMonitor(node)
    window = Window(monitor)
    window.show()
    def shutdown(signum, frame):
        window.close()
        app.quit()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    if args.self_check:
        def check():
            checks = dict(route=monitor.route_verified,
                          navigation=monitor.fresh('nav'), truth=monitor.fresh('truth'),
                          camera=monitor.fresh('camera'), matched_samples=len(monitor.errors),
                          frames=monitor.camera_count,
                          chart_channels=len(monitor.timeline.channels),
                          plot_samples=sum(len(points) for points in monitor.timeline.channels.values()))
            if args.require_flight:
                checks['desired_course'] = bool(monitor.timeline.channels.get('desired_course'))
                checks['cross_track'] = any(v is not None for t, v in monitor.timeline.channels.get('truth_cte', ()))
                checks['moving'] = bool(monitor.nav and monitor.nav['speed'] > 3.)
            window.grab().save(str(ROOT/'log_visual_nav/dashboard_preview.png'))
            window.save_report()
            window.export_charts(ROOT/'log_visual_nav/dashboard_charts')
            monitor.timeline.export_csv(ROOT/'log_visual_nav/dashboard_charts.csv')
            (ROOT/'log_visual_nav/dashboard_self_check.json').write_text(json.dumps(checks, indent=2))
            print(json.dumps(checks), flush=True)
            monitor.export()
            app.exit(0 if all(checks.values()) else 2)
        QtCore.QTimer.singleShot(int(max(1., args.check_seconds)*1000), check)
    try:
        result = app.exec_()
    finally:
        monitor.export()
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
