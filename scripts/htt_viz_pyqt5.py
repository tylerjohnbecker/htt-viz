#!/usr/bin/env python3

import sys
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QMainWindow, QWidget, QPushButton, QAction, QFrame, QApplication, QLabel
from PyQt5.QtCore import QSize, Qt    
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic

class MainWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.initUI()

    def initUI(self):
        self.setMinimumSize(QSize(350, 275)) 
        self.resize(700, 550)   
        self.setWindowTitle("HTT-Viz")
	
	label = QLabel(self)
	pixmap = QPixmap('image.jpeg')
	label.setPixmap(pixmap)
	self.resize(pixmap.width(),pixmap.height())
	
	self.show()

        # New Action
        newAction = QAction(QIcon('new.png'), '&New', self)        
        newAction.setShortcut('Ctrl+N')
        newAction.setStatusTip('New document')
        newAction.triggered.connect(self.newCall)

        # Open Action
        openAction = QAction(QIcon('open.png'), '&Open', self)        
        openAction.setShortcut('Ctrl+O')
        openAction.setStatusTip('Open document')
        openAction.triggered.connect(self.openCall)
        
        # Save Action
        saveAction = QAction(QIcon('save.png'), '&Save', self)        
        saveAction.setShortcut('Ctrl+S')
        saveAction.setStatusTip('Save document')
        saveAction.triggered.connect(self.saveCall)
        
        # Save As Action
        saveAsAction = QAction(QIcon('saveas.png'), '&Save As...', self)        
        saveAsAction.setShortcut('Ctrl+Shift+S')
        saveAsAction.setStatusTip('Save document as...')
        saveAsAction.triggered.connect(self.saveAsCall)
	
        # Exit Action
        exitAction = QAction(QIcon('exit.png'), '&Exit', self)        
        exitAction.setShortcut('Ctrl+Q')
        exitAction.setStatusTip('Exit application')
        exitAction.triggered.connect(self.exitCall)
        
        # Undo Action
        undoAction = QAction(QIcon('undo.png'), '&Undo', self)        
        undoAction.setShortcut('Ctrl+Z')
        undoAction.setStatusTip('Undo previous action')
        undoAction.triggered.connect(self.undoCall)
        
        # Redo Action
        redoAction = QAction(QIcon('redo.png'), '&Redo', self)        
        redoAction.setShortcut('Ctrl+Shift+Z')
        redoAction.setStatusTip('Redo previously undone action')
        redoAction.triggered.connect(self.redoCall)
        
        # Cut Action
        cutAction = QAction(QIcon('cut.png'), '&Cut', self)        
        cutAction.setShortcut('Ctrl+X')
        cutAction.setStatusTip('Cut content to clipboard')
        cutAction.triggered.connect(self.cutCall)
        
        # Copy Action
        copyAction = QAction(QIcon('copy.png'), '&Copy', self)        
        copyAction.setShortcut('Ctrl+C')
        copyAction.setStatusTip('Copy content to clipboard')
        copyAction.triggered.connect(self.copyCall)
        
        # Paste Action
        pasteAction = QAction(QIcon('paste.png'), '&Paste', self)        
        pasteAction.setShortcut('Ctrl+V')
        pasteAction.setStatusTip('Paste content on clipboard')
        pasteAction.triggered.connect(self.pasteCall)
        
        # Debug Action
        debugAction = QAction(QIcon('debug.png'), '&Debug', self)
        debugAction.setStatusTip('View the debug console')
        debugAction.triggered.connect(self.debugCall)
        
        # About Action
        aboutAction = QAction(QIcon('about.png'), '&About', self)
        aboutAction.setStatusTip('View the about window')
        aboutAction.triggered.connect(self.aboutCall)

        # Create menu bar and add actions
        menuBar = self.menuBar()
        
        fileMenu = menuBar.addMenu('&File')
        fileMenu.addAction(newAction)
        fileMenu.addAction(openAction)
        fileMenu.addAction(saveAction)
        fileMenu.addAction(saveAsAction)
        fileMenu.addSeparator()
        fileMenu.addAction(exitAction)
        
        editMenu = menuBar.addMenu('&Edit')
        editMenu.addAction(undoAction)
        editMenu.addAction(redoAction)
        editMenu.addSeparator()
        editMenu.addAction(cutAction)
        editMenu.addAction(copyAction)
        editMenu.addAction(pasteAction)
        
        viewMenu = menuBar.addMenu('&View')
        viewMenu.addAction(debugAction)
        
        helpMenu = menuBar.addMenu('&Help')
        helpMenu.addAction(aboutAction)
        
        # Splitter
        self.subWindow1 = SubWindow()
        self.subWindow2 = SubWindow()
        
        self.subsplitter = QSplitter(Qt.Horizontal)
        self.subsplitter.setStyleSheet('background-color: rgb(50,50,50)') 
        self.subsplitter.addWidget(self.subWindow1)
        self.subsplitter.addWidget(self.subWindow2)
        
        self.setCentralWidget(self.subsplitter)
        
        

    def openCall(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","YAML Files (*.yaml)", options=options)
        if fileName:
            print(fileName)

    def newCall(self):
        print('new')
    
    # https://pythonspot.com/pyqt5-file-dialog/    
    def saveCall(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getSaveFileName(self,"QFileDialog.getSaveFileName()","","YAML Files (*.yaml)", options=options)
        if fileName:
            print(fileName)
        
    def saveAsCall(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getSaveFileName(self,"QFileDialog.getSaveFileName()","","YAML Files (*.yaml)", options=options)
        if fileName:
            print(fileName)

    def exitCall(self):
        self.close()
        
    def undoCall(self):
        print('undo')

    def redoCall(self):
        print('redo')
        
    def cutCall(self):
        print('cut')
        
    def copyCall(self):
        print('copy')

    def pasteCall(self):
        print('paste')
        
    def debugCall(self):
        print('debug')

    def aboutCall(self):
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)
        msg.setText("HTT-Viz About...")
        msg.setWindowTitle("HTT-Viz About")
        
        msg.exec()
        
        
class SubWindow(QWidget):
    def __init__(self):
        super(SubWindow, self).__init__()

        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

class ScrollArea(QWidget):

    _style = '''
            QScrollArea{
                background: white;
            }
            
            QScrollBar:handle{
                background: gray;
                max-width: 20px;
                color:green;       
            }
            '''
    factor = 1.5
	
    def __init__(self, parent=None):
        super(ScrollArea, self).__init__()

        self.v_layout = QVBoxLayout(self)
        self.v_layout.setContentsMargins(0, 0, 0, 0)
        self.v_layout.setSpacing(0)

        self.container_widget = QWidget()

        # Scroll Area Properties
        self.scroll = QScrollArea()
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scroll.setWidgetResizable(False)
        self.scroll.setWidget(self.container_widget)
        self.scroll.setStyleSheet(ScrollArea._style)

        l = QLabel('Hello world', self.container_widget)
        l.setStyleSheet('color: red; font-size: 30px')

        self.container_widget.setGeometry(0, 0, self.width(), self.height())
        self.v_layout.addWidget(self.scroll)
        self.setLayout(self.v_layout)

        self._zoom = 0
        self.mousepos = QPoint(0, 0)
        self.setMouseTracking(True)
        self.showMaximized()

    def fitInView(self, scale=True):
        rect = QtCore.QRectF(self._photo.pixmap().rect())
        if not rect.isNull():
            self.setSceneRect(rect)
            if self.hasPhoto():
                unity = self.transform().mapRect(QtCore.QRectF(0, 0, 1, 1))
                self.scale(1 / unity.width(), 1 / unity.height())
                viewrect = self.viewport().rect()
                scenerect = self.transform().mapRect(rect)
                factor = min(viewrect.width() / scenerect.width(),
                             viewrect.height() / scenerect.height())
                self.scale(factor, factor)
            self._zoom = 0

    def wheelEvent(self, wheel_event):

        if wheel_event.modifiers() == Qt.ControlModifier:
            delta = wheel_event.angleDelta().y()
            if delta > 0:
                self.zoom_in()

            elif delta < 0:
                self.zoom_out()

        else:
            return super().wheelEvent(wheel_event)

    def mousePressEvent(self, event):
        cursor = self.container_widget.cursor().pos()
        print(cursor)
        if event.button() == Qt.MidButton:
            self.setCursor(Qt.OpenHandCursor)

        super(ScrollArea, self).mousePressEvent(event)

    def mouseMoveEvent(self, event):

        delta = event.localPos() - self.mousepos

        # panning area
        if event.buttons() == Qt.MidButton:
            h = self.scroll.horizontalScrollBar().value()
            v = self.scroll.verticalScrollBar().value()

            self.scroll.horizontalScrollBar().setValue(int(h - delta.x()))
            self.scroll.verticalScrollBar().setValue(int(v - delta.y()))

        self.mousepos = event.localPos()

        super(ScrollArea, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):

        self.unsetCursor()
        self.mousepos = event.localPos()
        super(ScrollArea, self).mouseReleaseEvent(event)

    def resizeEvent(self, event):
        self.container_widget.resize(self.width(), self.height())

        super(ScrollArea, self).resizeEvent(event)

    def resize_container(self, option):

        option = int(option)

        if option == 0:
            self.container_widget.resize(self.width()+50, self.height())

        elif option == 1:
            self.container_widget.resize(self.width()+50, self.height())

        elif option == 2:
            self.container_widget.resize(self.width()+50, self.height()+50)

    @QtCore.pyqtSlot()
    def zoom_in(self):
        self.container_widget.setGeometry(200, 200, self.container_widget.width() + 4,
                                          self.container_widget.height() + 4)

    @QtCore.pyqtSlot()
    def zoom_out(self):
   
        self.container_widget.setGeometry(0, 0, self.container_widget.width() - 4,
                                          self.container_widget.height() - 4)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit( app.exec_() )
