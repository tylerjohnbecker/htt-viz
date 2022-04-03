import pytest
from pytestqt.exceptions import (
  _is_exception_capture_enable,
  _QtExceptionCaptureManager,
)

from pytestqt.logging import QtLoggingPlugin, QtMessageCapture
from pytestqt.qt_compat import qt_api
from pytestqt import QtBot, _close_widgets


@pytest.fixture(scope = "session" )
def qapp_args():
  
  return []

@pytest.fixture(scope = "session" )
def qapp(qapp_args, pytestconfig):
  
  app = qt_api.QtWidgets.QApplication.instance()
  if app is None:
    global _qapp_instance
    _qapp_instance = qt_api.QtWidgets.QApplication(qapp_args)
    name = pytestconfig.getini("qt_qapp_name")
    _qapp_instance.setApplicationName(name)
    return _qapp_instance
  else:
    return app # pragma: no cover
  
  _qapp_instance = None
  
  def qtbot(qapp, request):
    
    result = QtBot(request)
    return result
  
  def qtmodeltester(request)
  
  from pytestqt.modeltest import ModelTester
   tester = ModelTester(request.config)
    yield tester 
    tester._cleanup()
   
def pytest_addoption(parser):
    parser.addini (
  
      
def pytest_rutest_setup(item)
      capture_enabled = _is_exception_capture_enable(item)
      
def pytest_runtest_call(item)
      yield 
      _process_events()
      capture_enabled = _is_exception_capture_enabled(item)
      
def pytest_runtest_teardown(item)
      
      """ 
      This allows any pending events for the test tear down as well as
      avoiding any events leaking into the next set of tests. If there are 
      any exceptions that have been captured during this test, we fail the test 
      """
      
      _process_events()
      _close_widgets(item)
      _process_events()
      yield
      _process_events()
      capture_enabled = _is_exception_capture_enabled(item)
      if capture_enabled:
      
def _process_events():
      app = qt_api.QWidgets.QApplication.instance()
  if app is not None:
    app.processEvents()
      
def pytest_configure(config):
      config.addinivalue_line()
      
def pytest_report_header():
  from pytest.qt
