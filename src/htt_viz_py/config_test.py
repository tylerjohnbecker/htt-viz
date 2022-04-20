import functools
import time

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QMainWindow, QWidget, QPushButton, QAction, QFrame
from PyQt5.QtCore import QSize, Qt	
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic

from htt_viz_py.tree import Tree, Node
import yaml
import htt_viz
import random as r

import pytest
from pytestqt.qt_compat import qt_api

pytest_plugins = "pytester"


def run_time():
    """
    This test is a fixture that makes it easier for tests to ensure signals 
    emitted and timeouts are being respected in the proper areas.
    """

    class RunTime:
        def __init__(self):
            self._start_time = None
            self.elapsed = None

        def start(self):
            self._start_time = time.monotonic()

        def stop(self):
            self.elapsed = (time.monotonic() - self._start_time) * 1000.0

        def check(self, timeout, *delays):
            """
            Make sure either timeout (if given) or at most of the given
            delays used to trigger a signal has passed.
            """
            self.stop()
            if timeout is None:
                timeout = max(delays) * 1.35  # 35% tolerance
            max_wait_ms = max(delays + (timeout,))
            assert self.elapsed < max_wait_ms

    return RunTime()


def timer():
    """
    This Test returns a kinda timer like object which can be used 
    to trigger signals and callbacks after some time.
  """

    class Timer(qt_api.QtCore.QObject):
        
        def __init__(self):
            qt_api.QtCore.QObject.__init__(self)
            self.timers_and_slots = []

        def shutdown(self):
            while self.timers_and_slots:
                t, slot = self.timers_and_slots.pop(-1)
                t.stop()
                t.timeout.disconnect(slot)

        def single_shot(self, signal, delay):
            t = qt_api.QtCore.QTimer(self)
            t.setSingleShot(True)
            slot = functools.partial(self._emit, signal)
            t.timeout.connect(slot)
            t.start(delay)
            self.timers_and_slots.append((t, slot))

        def single_shot_callback(self, callback, delay):
            t = qt_api.QtCore.QTimer(self)
            t.setSingleShot(True)
            t.timeout.connect(callback)
            t.start(delay)
            self.timers_and_slots.append((t, callback))

        def _emit(self, signal):
            signal.emit()

    timer = Timer()
    yield timer
    timer.shutdown()
