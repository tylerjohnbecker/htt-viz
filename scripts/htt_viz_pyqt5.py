import sys
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QMainWindow, QWidget, QPushButton, QAction, QFrame
from PyQt5.QtCore import QSize, Qt    
from PyQt5.QtGui import QIcon
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
        self.subWindow1 = SubWindow("left")
        self.subWindow2 = SubWindow("right")
        
        self.subsplitter = QSplitter(Qt.Horizontal)
        self.subsplitter.addWidget(self.subWindow1)
        self.subsplitter.addWidget(self.subWindow2)
        
        self.setCentralWidget(self.subsplitter)
        
        

    def openCall(self):
        print('Open')

    def newCall(self):
        print('New')
        
    def saveCall(self):
        print('Save')
        
    def saveAsCall(self):
        print('Save')

    def exitCall(self):
        print('Exit app')
        
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
        print('about')
        
        
class SubWindow(QWidget):
    def __init__(self, label):
        super(SubWindow, self).__init__()
        
        self.label = QLabel(label)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("QLabel {font-size:40px;}")

        self.main_layout = QVBoxLayout()
        self.main_layout.addWidget(self.label)
        self.setLayout(self.main_layout)



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit( app.exec_() )
