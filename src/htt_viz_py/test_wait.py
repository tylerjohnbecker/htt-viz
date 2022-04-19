import pytest

from htt_viz_py.tree import Tree, Node
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QMainWindow, QWidget, QPushButton, QAction, QFrame
from PyQt5.QtCore import QSize, Qt	
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic
import yaml
import htt_viz.srv
import random as r
from pytestqt.exception import TimeoutError

def test_wait(qtbot, wait_4_ticks_callback, tick_counter):
   tick_counter.start(100)
   qtbot.waitUntil(wait_4_ticks_callback, timeout = 1000)
   assert tick_counter.ticks >= 4
    
    
def test_wait_until timeout(qtbot, wait_4_ticks_callback, tick_counter):
      tick_counter.start(200)
      with pytest.raises(TimeoutError):
        qtbot.waitUntil(wait_4_ticks_callback, timeout = 100)
      assert tick_counter.ticks < 4
      
      
      
def test_invaild_callback_return_value(qtbot):
  with pytest.raises(ValueError):
    qtbot.waitUntil(lambda: [])
    
    
@pytest.fixture(params = ["predicate", "assert"])
def wait_4_ticks_callback(request, tick_counter):
  if request param == "predicate":
    return lambda: tick_counter.ticks >= 4
  else: 
    
    return check_ticks
  
@pytest.fixture
def tick_counter():
   
  #This returns an object which counts the timer every so often

  from pytest.qt_combat import qt_api
  
  class Counter:
    def __init__(self):
      self._ticks = 0
      self.timer = qt_api.QtCore.QTimer()
      self.timer.timeout.connect(self._tick)
      
    def start(self, ms):
        self.timer.start(ms)
        
    def _tick(self):
      self.ticks += 1
      
    @property
    def ticks(self):
      return self._ticks
    
  counter = Counter()
  yield counter()
  yield counter
  counter.timer.stop()







