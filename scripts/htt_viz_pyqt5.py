#!/usr/bin/env python3

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
        self.subWindow1 = SubWindow()
        self.subWindow2 = SubWindow()
        
        self.subsplitter = QSplitter(Qt.Horizontal)
        self.subsplitter.setStyleSheet('background-color: rgb(50,50,50)') 
        self.subsplitter.addWidget(self.subWindow1)
        self.subsplitter.addWidget(self.subWindow2)
        
        # Setup Task Tree Display
        self.taskTree = TaskTree()
        taskTreeDisplayWidget = HTTDisplayWidget(self.taskTree)
        self.subWindow2.layout().addWidget(taskTreeDisplayWidget)
        
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


class HTTDisplayWidget(QGraphicsView):
    def __init__(self, taskTree):
        super().__init__()
        
        self.taskTree = taskTree
        
        self.scene = QGraphicsScene()
        self.scene.addText("Hello, world!")
        
        self.scene.addItem(self.taskTree.rootNode.qGraphics)
        
        self.setScene(self.scene)
        self.show()
        
class TaskTree:
    def __init__(self):
        ROOT_NAME = "ROOT_4_0_000"
        
        self.rootNode = TaskTreeNode(ROOT_NAME)
        self.nodes = [self.rootNode]
        
class TaskTreeNode:
    def __init__(self, name, x=0, y=0, parent=None, scene=None):
        self.name = name
        
        self.parent = parent
        self.children = None
        
        # Display State
        self.qGraphics = QGraphicsTaskTreeNode(self.name)
        self.qGraphics.setX(x)
        self.qGraphics.setY(y)
        
    # Returns `true` if this is a root. 
    # Roots are nodes that lack a parent. There should only be one.
    def isRoot(self):
        return self.parent is not None
        
class QGraphicsTaskTreeNode(QGraphicsItem):
    def __init__(self, name):
        super().__init__()
        
        self.name = name
        
        self.xRadius = 10.0
        self.yRadius = 10.0
        
        self.paddingX = 20.0
        self.paddingY = 20.0
        
        self.normalColor = Qt.red
        
        fontSize = 12
        self.font = QFont("Times", fontSize)
        
        font_metrics = QFontMetrics(self.font)
        self.textWidth = font_metrics.width(self.name)
        self.textHeight = font_metrics.height()
        
        self.width = self.textWidth + (2.0 * self.paddingX)
        self.height = self.textHeight + (2.0 * self.paddingY)
        
        # https://stackoverflow.com/questions/10950820/qt-qgraphicsitem-drag-and-drop
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
    
    def boundingRect(self):
        return QRectF(0, 0, self.width, self.height)
        
    def paint(self, painter, option, widget):
        textX = (self.width / 2.0) - (self.textWidth / 2.0)
        textY = (self.height / 2.0) - (self.textHeight / 2.0)
        
        path = QPainterPath()
        path.addRoundedRect(QRectF(0, 0, self.width, self.height), self.xRadius, self.yRadius)
        painter.fillPath(path, self.normalColor)
        
        painter.setFont(self.font)
        painter.drawText(QPointF(textX, textY), self.name)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit( app.exec_() )
