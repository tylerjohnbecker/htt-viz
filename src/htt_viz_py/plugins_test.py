import pytest
from pytestqt.exceptions import 
(
  _is_exception_capture_enable,
  _QtExceptionCaptureManager
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
  
