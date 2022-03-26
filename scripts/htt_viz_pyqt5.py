#!/usr/bin/env python3

import sys
import yaml

#import rospy as rp
#from htt_viz.srv import Update

from htt_viz_py.QGraphicsTaskTreeNode import QGraphicsTaskTreeNode
from htt_viz_py.tree import Tree

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
		
		#This is a placeholder for now as the actual tree isn't used however once it is it should be good
		#to go just by uncommenting it and the ros functionality will go back to normal
		#if make_service == True :
			#self.server = rp.Service('update_htt', Update, self.taskTree.UpdateCallback)


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
		self.taskTree = Tree()
		self.taskTreeDisplayWidget = HTTDisplayWidget(self.taskTree, self)
		self.subWindow2.layout().addWidget(self.taskTreeDisplayWidget)
		
		self.nodeList = QListWidget()
		self.subWindow1.layout().addWidget(self.nodeList)
		
		for nodeType in self.taskTree.author.node_master_list[1:]:
			self.nodeList.addItem(nodeType.name)
		
		self.nodeList.itemSelectionChanged.connect(self.handleNodeSelectionChange)
		self.nodeList.item(0).setSelected(True)
		self.subWindow1.setMaximumWidth(100)
		
		self.setCentralWidget(self.subsplitter)
		
		self.filePath = None
		
	def handleNodeSelectionChange(self):
		selectedItem = self.nodeList.selectedItems()[0]

		# We only use 1 column. 
		# index is incremented by 1 as root is removed from QListWidget
		self.taskTreeDisplayWidget.selectedNodeIndex = self.nodeList.row(selectedItem) + 1

	def openCall(self):
		options = QFileDialog.Options()
		options |= QFileDialog.DontUseNativeDialog
		filePath, _ = QFileDialog.getOpenFileName(self, "Open", "","YAML Files (*.yaml)", options=options)
		if filePath:
			self.filePath = filePath
			
			with open(filePath, "r") as f:
				data = yaml.load(f, Loader=yaml.Loader)
				
				self.taskTreeDisplayWidget.clearTaskTree()
				
				for node in data["NodeList"]:
					#name, x=0, y=0, nParent = None
					x = data["Nodes"][node]["x"]
					y = data["Nodes"][node]["y"]
					parent = data["Nodes"][node]["parent"]

					node_type = data["Nodes"][node]["mask"]["type"]
					
					self.taskTreeDisplayWidget.addChildNode(parent, node_type, x, y)

	def newCall(self):
		self.taskTreeDisplayWidget.clearTaskTree()
		self.filePath = None
	
	# https://pythonspot.com/pyqt5-file-dialog/	
	def saveCall(self):
		filePath = self.filePath
		
		if not filePath:
			options = QFileDialog.Options()
			options |= QFileDialog.DontUseNativeDialog
			filePath, _ = QFileDialog.getSaveFileName(self,"Save","","YAML Files (*.yaml)", options=options)
			if not filePath:
				return
			self.filePath = filePath
			
		data = self.taskTree.toYamlDict()
		with open(filePath, "w") as f:
			yaml.dump(data, f)
			
		self.setWindowTitle("HTT-Viz")
		
	def saveAsCall(self):
		options = QFileDialog.Options()
		options |= QFileDialog.DontUseNativeDialog
		filePath, _ = QFileDialog.getSaveFileName(self, "Save As","","YAML Files (*.yaml)", options=options)
		if filePath:
			self.filePath = filePath
			
			data = self.taskTree.toYamlDict()
			with open(filePath, "w") as f:
				yaml.dump(data, f)
				
		self.setWindowTitle("HTT-Viz")

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

	def closeEvent(self, event):
		if self.windowTitle() == "HTT-Viz":
			event.accept()
		else:
			event.ignore()
			self.w = UnsavedContentWindow(self)
			self.w.show()

class SubWindow(QWidget):
	def __init__(self):
		super(SubWindow, self).__init__()

		self.main_layout = QVBoxLayout()
		self.setLayout(self.main_layout)


class HTTDisplayWidget(QGraphicsView):
	def __init__(self, taskTree, mainWin):
		super().__init__()
		self.mainWin = mainWin
		
		self.taskTree = taskTree
		self.scene = QGraphicsScene()
		self.scene.setBackgroundBrush(QColor(0x3D, 0x3D, 0x3D))
		self.setScene(self.scene)
		self.taskTree.registerScene(self.scene)
		
		# TODO: Scrolling makes a constant cursor, and feels really janky to use
		# Maybe we should implment our own?
		self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
		
		self.show()
		
		self.selectedNodeIndex = 0
		
	def addChildNode(self, parentName, selectedNodeIndex, x=None, y=None):
		parentNode = self.taskTree.findNodeByName(parentName)
		if parentNode is not None:
			self.taskTree.AddNode([parentNode, selectedNodeIndex, True])
			
		self.mainWin.setWindowTitle("*HTT-Viz")
				
	def removeChildNode(self, name):
		self.taskTree.RemoveNode([name, True])
		
		self.mainWin.setWindowTitle("*HTT-Viz")
				
	def clearTaskTree(self):
		for child in self.taskTree.root_node.children:
			self.removeChildNode(child.name)
		
	def contextMenuEvent(self, event):
		eventPos = event.pos()
		maybeNode = self.itemAt(eventPos)
		if not isinstance(maybeNode, QGraphicsTaskTreeNode):
			maybeNode = None
	
		# https://stackoverflow.com/questions/67591464/create-event-clicking-on-context-menu-with-python-pyqt5
		menu = QMenu(self)
		# TODO: Only show items if a node was selected, else early exit or show different menu
		addChildNode = menu.addAction("Add Child Node")
		editNode = menu.addAction("Edit Node")
		removeNode = menu.addAction("Remove Node")
		close = menu.addAction("Close")
		action = menu.exec_(self.mapToGlobal(eventPos))
		
		if action == addChildNode:
			if maybeNode is not None:
				self.addChildNode(maybeNode.name, self.selectedNodeIndex)
		elif action == editNode:
			if maybeNode is not None:
				#print('Edit Node')
				self.w = EditWindow(maybeNode, self.mainWin)
				self.w.show()
		elif action == removeNode:
			if maybeNode is not None:
				self.removeChildNode(maybeNode.name)
		elif action == close:
			if self.windowTitle() != "HTT-Viz":
				self.w = UnsavedContentWindow(self.mainWin)
				self.w.show()
			else:   
				app.quit()
		else:
			print('Unknown Action:')
			print(action)
			
class EditWindow(QWidget):
	def __init__(self, node, mainWin):
		super().__init__()
		self.node = node
		self.mainWin = mainWin
		
		textBoxList = QFormLayout()
		
		p = self.palette()
		p.setColor(self.backgroundRole(), QColor(0x3D, 0x3D, 0x3D))
		self.setPalette(p)
		
		self.setWindowTitle("Edit Node")
		self.setMinimumSize(QSize(350, 275)) 
		self.resize(350, 275)
		
		self.titleBox = QLineEdit()
		self.titleBox.resize(200,40)
		self.titleBox.setText(node.name) #CHANGE TO NODE TITLE
		textBoxList.addRow("Title", self.titleBox)
		
		self.paramBox = QPlainTextEdit()
		self.paramBox.resize(200,120)
		self.paramBox.setPlainText("Node Params") #NEEDS BACKEND
		textBoxList.addRow("Parameters", self.paramBox)
		
		#CHANGE TO FOREACH FOR PARAMETER ARRAY BASED ON TYPE
		
		#self.paramList = []
		#for x in node.params:
		#	param = None
		#	
		#	match x.type:
		#		case "bool":
		#			param = QCheckBox()
		#		case "float":
		#			param = QLineEdit()
		#		case "int":
		#			param = QLineEdit()
		#		case "string":
		#			param = QLineEdit()
		#			
		#	paramList.append(param)
		#	textBoxList.addRow(x.name, param)
		
		saveButton = QPushButton('Save', self)
		saveButton.clicked.connect(self.saveClick)
		
		closeButton = QPushButton('Close', self)
		closeButton.clicked.connect(self.closeClick)
		
		textBoxList.addRow(saveButton, closeButton)
		
		self.setLayout(textBoxList)
		
		self.titleBox.textChanged.connect(self.onChange)
		self.paramBox.textChanged.connect(self.onChange)

	def onChange(self):
		self.setWindowTitle("*Edit Node")
		
	def saveClick(self):
		#save parameter values
		self.node.name = self.titleBox.text() #CHANGE TO NODE TITLE
		self.mainWin.setWindowTitle("*HTT-Viz")
		self.setWindowTitle("Edit Node")
		
	def closeClick(self):
		winTitle = "Edit Node"
		if self.windowTitle() != winTitle:
			self.w = UnsavedContentWindow(self)
			self.w.show()

			#msg = QMessageBox()
			#msg.setIcon(QMessageBox.Information)
			#msg.setText("Unsaved Changes")
			#msg.setWindowTitle("Exit Error")
			#msg.exec()
		else:	
			self.close()

class UnsavedContentWindow(QWidget):
	def __init__(self, window):
		super().__init__()
		self.window = window

		p = self.palette()
		p.setColor(self.backgroundRole(), QColor(0x3D, 0x3D, 0x3D))
		self.setPalette(p)
		
		self.setWindowTitle("ERROR")
		self.setMinimumSize(QSize(225, 130))
		self.setMaximumSize(QSize(225, 130)) 
		self.resize(225, 130)

		uC = QLabel()
		uC.setText("Unsaved Content")
		uC.setStyleSheet("color: white")
		uC.setAlignment(Qt.AlignCenter)

		vbox = QVBoxLayout()
		vbox.addWidget(uC)

		saveButton = QPushButton('Save Content', self)
		#saveButton.move(5, 80)
		saveButton.clicked.connect(self.saveContentClick)
		vbox.addWidget(saveButton)
		
		closeButton = QPushButton('Close Anyway', self)
		#closeButton.move(120, 80)
		closeButton.clicked.connect(self.closeAnywayClick)
		vbox.addWidget(closeButton)

		
		
		self.setLayout(vbox)

	def saveContentClick(self):
		#save parameter values
		if self.window.windowTitle() == "*HTT-Viz":
			self.window.saveCall()
			self.close()
		elif self.window.windowTitle() == "*Edit Node":
			self.window.saveClick()
			self.close()
		
	def closeAnywayClick(self):
			#self.window.close()
		if self.window.windowTitle() == "*HTT-Viz":
			app.quit()
		elif self.window.windowTitle() == "*Edit Node":
			self.window.close()
			self.close()

if __name__ == "__main__":
	app = QtWidgets.QApplication(sys.argv)
	mainWin = MainWindow()
	mainWin.show()
	sys.exit( app.exec_() )
