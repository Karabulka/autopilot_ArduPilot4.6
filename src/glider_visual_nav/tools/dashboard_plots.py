"""Bounded, simulator-time charts for the independent flight observer."""
from collections import deque
import csv
import math
from python_qt_binding import QtCore, QtGui, QtWidgets

COLORS = {'desired': '#e9c46a', 'nav': '#45d4e8', 'truth': '#f28e65',
          'error': '#c6a0f6', 'second': '#65d49a'}


def line_runs(points, start, end, angular=False, gap=.5):
    """Do not connect missing intervals or opposite ends of wrapped angles."""
    runs, run = [], []
    for t, value in points:
        if t < start:
            continue
        if t > end:
            break
        valid = value is not None and math.isfinite(value)
        split = run and (t-run[-1][0] > gap or
                         (valid and angular and abs(value-run[-1][1]) > 180.))
        if not valid or split:
            if run:
                runs.append(run)
            run = []
        if valid:
            run.append((t, value))
    if run:
        runs.append(run)
    return runs


def command_at(commands, stamp, max_age=.5):
    for command in reversed(commands):
        dt = stamp-command['t']
        if dt >= -1e-7:
            return command if dt <= max_age else None
    return None


class Timeline:
    def __init__(self):
        self.channels = {}
        self.events = deque(maxlen=500)
        self.latest = None
        self.origin = None
        self.sequence = 0

    def add(self, key, stamp, value):
        if not math.isfinite(stamp) or (value is not None and not math.isfinite(value)):
            return
        if self.origin is None:
            self.origin = stamp
        self.latest = max(stamp, self.latest if self.latest is not None else stamp)
        series = self.channels.setdefault(key, deque(maxlen=36000))
        if series and stamp < series[-1][0]:
            return
        if series and stamp == series[-1][0]:
            series.pop()
        series.append((stamp, value))
        while series and series[0][0] < self.latest-305.:
            series.popleft()

    def event(self, stamp, segment, label):
        self.events.append((stamp, segment, label))

    def export_csv(self, path):
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_time_s', 'interval_time_s', 'channel', 'value'])
            for key, points in self.channels.items():
                for stamp, value in points:
                    writer.writerow([stamp, stamp-self.origin, key, '' if value is None else value])
            for stamp, segment, label in self.events:
                writer.writerow([stamp, stamp-self.origin, 'waypoint_event', label])


class TimePlot(QtWidgets.QWidget):
    def __init__(self, timeline, title, unit, channels, view, angular=False, zero=False):
        super().__init__()
        self.timeline, self.title, self.unit = timeline, title, unit
        self.channels, self.view = channels, view
        self.angular, self.zero = angular, zero
        self.setMinimumSize(380, 220)
        self.setMouseTracking(True)
        self.mouse = None

    def mouseMoveEvent(self, event):
        self.mouse = event.pos()
        self.update()

    def leaveEvent(self, event):
        self.mouse = None
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#111b2a'))
        p.setPen(QtGui.QColor('#edf3fa'))
        font = p.font()
        font.setPointSize(10)
        font.setBold(True)
        p.setFont(font)
        p.drawText(14, 23, self.title+' ['+self.unit+']')
        font.setPointSize(9)
        font.setBold(False)
        p.setFont(font)
        for i, (_, label, color) in enumerate(self.channels):
            col, row = i % 2, i // 2
            p.setPen(QtGui.QPen(QtGui.QColor(color), 2))
            x = 15+col*(self.width()//2)
            p.drawLine(x, 44+row*18, x+17, 44+row*18)
            p.drawText(x+23, 48+row*18, label)
        top = 62 if len(self.channels) <= 2 else 80
        rect = QtCore.QRectF(64, top, max(1, self.width()-84), max(1, self.height()-top-38))
        latest = self.view['end'] if self.view['end'] is not None else self.timeline.latest
        if latest is None:
            p.setPen(QtGui.QColor('#a8b6c8'))
            p.drawText(rect, QtCore.Qt.AlignCenter, 'Ожидание телеметрии…')
            return
        origin = self.timeline.origin if self.timeline.origin is not None else latest
        start, end = max(origin, latest-self.view['seconds']), latest
        if end-start < 1.:
            end = start+1.
        series = [(key, color, line_runs(self.timeline.channels.get(key, ()), start, end, self.angular))
                  for key, label, color in self.channels]
        values = [value for key, color, runs in series for run in runs for t, value in run]
        if self.angular:
            low, high = -180., 180.
        elif values:
            low, high = min(values), max(values)
            if self.zero:
                low, high = min(low, 0.), max(high, 0.)
            margin = max(.15, (high-low)*.12)
            low, high = low-margin, high+margin
        else:
            low, high = -1., 1.
        def xy(t, v):
            return QtCore.QPointF(rect.left()+(t-start)/(end-start)*rect.width(),
                                 rect.bottom()-(v-low)/(high-low)*rect.height())
        p.setPen(QtGui.QColor('#2a3a50'))
        for i in range(5):
            value = low+(high-low)*i/4
            y = xy(start, value).y()
            p.drawLine(QtCore.QPointF(rect.left(), y), QtCore.QPointF(rect.right(), y))
            p.setPen(QtGui.QColor('#a8b6c8'))
            p.drawText(QtCore.QRectF(0, y-9, 58, 18), QtCore.Qt.AlignRight, f'{value:.1f}')
            p.setPen(QtGui.QColor('#2a3a50'))
        for i in range(5):
            t = start+(end-start)*i/4
            x = xy(t, low).x()
            p.drawLine(QtCore.QPointF(x, rect.top()), QtCore.QPointF(x, rect.bottom()))
            p.setPen(QtGui.QColor('#a8b6c8'))
            p.drawText(QtCore.QRectF(x-30, rect.bottom()+5, 60, 18), QtCore.Qt.AlignCenter, f'{t-origin:.1f}')
            p.setPen(QtGui.QColor('#2a3a50'))
        p.setClipRect(rect.adjusted(-1, -1, 1, 1))
        for t, segment, label in self.timeline.events:
            if start <= t <= end:
                p.setPen(QtGui.QPen(QtGui.QColor('#72839a'), 1, QtCore.Qt.DashLine))
                p.drawLine(xy(t, low), xy(t, high))
                p.drawText(xy(t, high)+QtCore.QPointF(3, 13), label)
        if self.zero and low <= 0 <= high:
            p.setPen(QtGui.QPen(QtGui.QColor('#7b8ca2'), 1, QtCore.Qt.DotLine))
            p.drawLine(xy(start, 0), xy(end, 0))
        for key, color, runs in series:
            p.setPen(QtGui.QPen(QtGui.QColor(color), 1.8))
            for run in runs:
                # Keep rendering cost bounded by screen resolution; raw CSV is unchanged.
                if len(run) > rect.width()*3:
                    bins = {}
                    for t, value in run:
                        bucket = int(xy(t, value).x())
                        bins.setdefault(bucket, []).append((t, value))
                    run = [pair for bucket in bins.values() for pair in sorted(set(
                        (bucket[0], min(bucket, key=lambda x: x[1]), max(bucket, key=lambda x: x[1]), bucket[-1])))]
                if len(run) > 1:
                    p.drawPolyline(QtGui.QPolygonF([xy(t, value) for t, value in run]))
                elif run:
                    p.drawEllipse(xy(*run[0]), 2, 2)
        p.setClipping(False)
        if not values:
            p.setPen(QtGui.QColor('#a8b6c8'))
            p.drawText(rect, QtCore.Qt.AlignCenter, 'Нет данных в выбранном интервале')
        p.setPen(QtGui.QColor('#a8b6c8'))
        p.drawText(self.width()-165, self.height()-4, 't симуляции, с')
        if self.mouse and rect.contains(self.mouse):
            t = start+(self.mouse.x()-rect.left())/rect.width()*(end-start)
            p.setPen(QtGui.QPen(QtGui.QColor('#dce5ef'), 1, QtCore.Qt.DotLine))
            p.drawLine(xy(t, low), xy(t, high))
            rows = [f't={t-origin:.2f} с']
            for key, label, color in self.channels:
                candidates = [(abs(stamp-t), value) for stamp, value in self.timeline.channels.get(key, ())
                              if value is not None and abs(stamp-t) <= .25]
                rows.append(label+': '+(f'{min(candidates)[1]:.2f}' if candidates else '—'))
            QtWidgets.QToolTip.showText(self.mapToGlobal(self.mouse), '\n'.join(rows), self)


class ChartPage(QtWidgets.QWidget):
    def __init__(self, timeline, specs, view):
        super().__init__()
        layout = QtWidgets.QGridLayout(self)
        self.plots = []
        for i, spec in enumerate(specs):
            plot = TimePlot(timeline, *spec[:3], view, **spec[3])
            layout.addWidget(plot, i//2, i % 2)
            self.plots.append(plot)

    def refresh(self):
        for plot in self.plots:
            plot.update()


GUIDANCE = [
    ('Курс движения χ (ENU)', '°', [('desired_course', 'Заданный LOS', COLORS['desired']),
      ('nav_course', 'Навигация', COLORS['nav']), ('truth_course', 'Gazebo', COLORS['truth'])], {'angular': True}),
    ('Поперечное отклонение от участка e⊥', 'м', [('nav_cte', 'Навигация', COLORS['nav']),
      ('truth_cte', 'Gazebo', COLORS['truth'])], {'zero': True}),
    ('Крен: команда и исполнение', '°', [('desired_roll', 'Заданный', COLORS['desired']),
      ('nav_roll', 'IMU в навигации', COLORS['nav']), ('truth_roll', 'Gazebo', COLORS['truth'])], {'zero': True}),
    ('Высота корпуса', 'м', [('desired_height', 'Целевая / опорная', COLORS['desired']),
      ('nav_height', 'Барометрическая', COLORS['nav']), ('truth_height', 'Gazebo', COLORS['truth'])], {}),
]

ACCURACY = [
    ('Ошибка оценки скорости в FLU', 'м/с', [('error_vx', 'ΔVx', COLORS['nav']),
      ('error_vy', 'ΔVy', COLORS['second'])], {'zero': True}),
    ('Ошибка магнитного курса ψ', '°', [('error_yaw', 'Навигация − Gazebo', COLORS['error'])], {'zero': True}),
    ('Накопленное расхождение XY', 'м', [('drift', 'После совмещения начала XY', COLORS['error'])], {'zero': True}),
    ('Горизонтальная скорость', 'м/с', [('nav_speed', 'Навигация', COLORS['nav']),
      ('truth_speed', 'Gazebo', COLORS['truth'])], {}),
]
