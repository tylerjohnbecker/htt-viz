#!/usr/bin/env python3

import sys
#import rospy as rp
#from htt_viz.srv import Update
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QMainWindow, QWidget, QPushButton, QAction, QFrame
from PyQt5.QtCore import QSize, Qt    
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic
import yaml

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
        self.taskTree = TaskTree()
        self.taskTreeDisplayWidget = HTTDisplayWidget(self.taskTree, "THEN", self)
        self.subWindow2.layout().addWidget(self.taskTreeDisplayWidget)
        
        self.nodeList = QListWidget()
        self.subWindow1.layout().addWidget(self.nodeList)
        self.nodeList.addItem("THEN")
        self.nodeList.addItem("AND")
        self.nodeList.addItem("OR")
        self.nodeList.addItem("MOVE")
        self.nodeList.addItem("GRAB")
        self.nodeList.itemSelectionChanged.connect(self.handleNodeSelectionChange)
        self.nodeList.item(0).setSelected(True)
        self.subWindow1.setMaximumWidth(100)
        
        self.setCentralWidget(self.subsplitter)
        
        self.filePath = None
        
    def handleNodeSelectionChange(self):
        selectedItem = self.nodeList.selectedItems()[0]
        self.taskTreeDisplayWidget.selectedNode = selectedItem.text()

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
                    self.taskTreeDisplayWidget.addChildNode(parent, node, x, y)

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
            
        data = serialize_tree(self.taskTree)
        with open(filePath, "w") as f:
            yaml.dump(data, f)

        self.setWindowTitle("HTT-Viz")

        
        
    def saveAsCall(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        filePath, _ = QFileDialog.getSaveFileName(self, "Save As","","YAML Files (*.yaml)", options=options)
        if filePath:
            self.filePath = filePath
            
            data = serialize_tree(self.taskTree)
            with open(filePath, "w") as f:
                yaml.dump(data, f)

            self.setWindowTitle("HTT-Viz")

    def exitCall(self):
        winTitle = "HTT-Viz"
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
    def __init__(self, taskTree, selectedNode, mainWin):
        super().__init__()
        self.mainWin = mainWin

        self.taskTree = taskTree
        
        self.scene = QGraphicsScene()
        #self.scene.addText("Welcome to HTT-Viz!")
        
        self.scene.addItem(self.taskTree.rootNode.qGraphics)
        
        self.scene.setBackgroundBrush(QColor(0x3D, 0x3D, 0x3D))
        
        self.setScene(self.scene)
        
        # TODO: Scrolling makes a constant cursor, and feels really janky to use
        # Maybe we should implment our own?
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        
        self.show()
        
        self.selectedNode = selectedNode
        
        self.numCreatedNodes = 0
        
    def addChildNode(self, parentName, name, x=None, y=None):
        parentNode = self.taskTree.getNodeByName(parentName)
        if parentNode is not None:
            child = self.taskTree.addChild(parentNode, TaskTreeNode(name))
            if child is not None:
                child.setX(x or parentNode.getX())
                child.setY(y or (parentNode.getY() + parentNode.height() + 20.0))
                
                self.scene.addItem(child.qGraphics)
                self.scene.addItem(child.qGraphics.lines[-1].line)
                
                self.numCreatedNodes += 1

                self.mainWin.setWindowTitle("*HTT-Viz")
                
    def removeChildNode(self, name):
        node = self.taskTree.getNodeByName(name)
        if node is not None:
            if self.taskTree.removeNode(node):
                def cleanNode(scene, node, depth=0):
                    scene.removeItem(node.qGraphics)
                    for line in node.qGraphics.lines:
                        if line.child:
                            scene.removeItem(line.line)
                        elif line.parent:
                            if depth == 0:
                                maybeLine = next(filter(lambda parentLine: parentLine.child.name == line.parent.name, iter(line.parent.qGraphics.lines)), None)
                                if maybeLine is not None:
                                    line.parent.qGraphics.lines.remove(maybeLine)
                                    scene.removeItem(maybeLine.line)
                        else:
                            print('Neither child not parent opposing node')
                    for child in node.children:
                        cleanNode(scene, child, depth+1)
                        
                cleanNode(self.scene, node)

                self.mainWin.setWindowTitle("*HTT-Viz")
                
    def clearTaskTree(self):
        for child in self.taskTree.rootNode.children:
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
                self.addChildNode(maybeNode.name, f'{self.selectedNode}_4_0_00{self.numCreatedNodes}')
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
            #app.quit()
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
        #    param = None
        #    
        #    match x.type:
        #        case "bool":
        #            param = QCheckBox()
        #        case "float":
        #            param = QLineEdit()
        #        case "int":
        #            param = QLineEdit()
        #        case "string":
        #            param = QLineEdit()
        #            
        #    paramList.append(param)
        #    textBoxList.addRow(x.name, param)
        
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
            app.quit()


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
        
        
        
class TaskTree:
    def __init__(self):
        ROOT_NAME = "ROOT_4_0_000"
        
        self.rootNode = TaskTreeNode(ROOT_NAME)
        self.nodes = [self.rootNode]
        
    def getNodeByName(self, name):
        for node in self.nodes:
            if node.name == name:
                return node
        return None
        
    def getParentOfNode(self, inputNode):
        for node in self.nodes:
            for child in node.children:
                if child.name == inputNode.name:
                    return node
        return None
        
    # Add a child from a given node
    # Returns the added node if successful
    def addChild(self, parent, node):    
        if self.getNodeByName(node.name) is not None:
            return None
            
        self.nodes.append(node)
        parent.addChild(node)
            
        return node
        
    def removeNode(self, node):
        if node == self.rootNode:
            return False
        
        self.nodes.remove(node)
        
        for child in node.children:
            self.removeNode(child)
            
        return True
        
class TaskTreeNode:
    def __init__(self, name, x=0, y=0, scene=None):
        self.name = name
        
        self.children = []
        
        # Display State
        self.qGraphics = QGraphicsTaskTreeNode(self.name)
        self.qGraphics.setX(x)
        self.qGraphics.setY(y)
        
    # Returns `true` if this is a root. 
    # Roots are nodes that lack a parent. There should only be one.
    def isRoot(self):
        return self.parent is not None
        
    # Get the width of this node
    def width(self):
        return self.qGraphics.width
        
    # Get the height of this node
    def height(self):
        return self.qGraphics.height
        
    # Get the X value of this node
    def getX(self):
        return self.qGraphics.x()
        
     # Get the Y value of this node
    def getY(self):
        return self.qGraphics.y()
        
    # Set the X value of this node
    def setX(self, x):
        return self.qGraphics.setX(x)
        
    # Set the Y value of this node
    def setY(self, y):
        return self.qGraphics.setY(y)
        
    # Add a child node relationship
    def addChild(self, node):
        self.children.append(node)
        
        line = QGraphicsLineItem(self.getX() + (self.width() / 2.0), self.getY() + self.height(), node.getX() + (node.width() / 2.0), node.getY())
        node.qGraphics.lines.append(QGraphicsTaskTreeNodeLine(line=line, parent=self))
        self.qGraphics.lines.append(QGraphicsTaskTreeNodeLine(line=line, child=self))
        
class QGraphicsTaskTreeNode(QGraphicsItem):
    def __init__(self, name):
        super().__init__()
        
        self.name = name
        
        self.xRadius = 10.0
        self.yRadius = 10.0
        
        self.paddingX = 20.0
        self.paddingY = 20.0
        
        self.normalColor = QColor(121, 218, 255)
        self.borderColor = QColor(4, 180, 245)
        
        fontSize = 12
        self.font = QFont("Times", fontSize)
        
        font_metrics = QFontMetrics(self.font)
        self.textWidth = font_metrics.width(self.name)
        self.textHeight = font_metrics.height()
        
        # TODO: These aren't centering vertically correctly and I don't know why.
        self.width = self.textWidth + (2.0 * self.paddingX)
        self.height = self.textHeight + (2.0 * self.paddingY)
        
        # https://stackoverflow.com/questions/10950820/qt-qgraphicsitem-drag-and-drop
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        
        self.lines = []
    
    def boundingRect(self):
        return QRectF(0, 0, self.width, self.height)
        
    def paint(self, painter, option, widget):
        textX = (self.width / 2.0) - (self.textWidth / 2.0)
        textY = (self.height / 2.0) - (self.textHeight / 2.0)
        
        path2 = QPainterPath()
        path2.addRoundedRect(QRectF(0, 0, self.width, self.height), self.xRadius, self.yRadius)
        painter.fillPath(path2, self.borderColor)
        
        path = QPainterPath()
        path.addRoundedRect(QRectF(5, 5, self.width-10, self.height-10), self.xRadius-10, self.yRadius-10)
        painter.fillPath(path, self.normalColor)
        
        
        
        
        painter.setFont(self.font)
        painter.drawText(QPointF(textX, textY), self.name)
        
    def itemChange(self, change, value):
        ret = super().itemChange(change, value)
        
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange:
            for line in self.lines:
                innerLine = line.line.line()
                if line.parent:
                    innerLine.setP2(QPointF(value.x() + (self.width / 2.0), value.y()))
                elif line.child:
                    innerLine.setP1(QPointF(value.x() + (self.width / 2.0), value.y() + self.height))
                line.line.setLine(innerLine)
                        
        
        return ret
        
class QGraphicsTaskTreeNodeLine:
    def __init__(self, line=None, parent=None, child=None):
        self.parent = parent
        self.child = child
        self.line = line
        
def serialize_tree(tree):
    # initialize overarching vars
    tree_dict = {}
    tree_dict["Nodes"] = {}
    tree_dict["NodeList"] = []

    # make sure that the nodelist is in the correct order (parents before children)
    def populateNodeList(node, list):
        list.append(node.name)
        for child in node.children:
            populateNodeList(child, list)
        
    populateNodeList(tree.rootNode, tree_dict["NodeList"])

	# iterate through each child now (could be any order, right now in alphabetical)
    for treeNode in tree.nodes:
        nodeName = treeNode.name
        
        # initialize the entry for the child
        tree_dict["Nodes"][nodeName] = {}	
        
        # now the mask
        tree_dict["Nodes"][nodeName]["mask"] = {}

        # each mask is saved in the Name of the child like so (Name_type_robot_node)
        type = int(nodeName[-7:-6])
        robot = int(nodeName[-5:-4])
        node  = int(nodeName[-3:])

        tree_dict["Nodes"][nodeName]["mask"]["type"] = type
        tree_dict["Nodes"][nodeName]["mask"]["robot"] = robot
        tree_dict["Nodes"][nodeName]["mask"]["node"] = node

        # Now save the parent (if its none we make sure its all caps in the file)
        maybeParent = tree.getParentOfNode(treeNode)
        if maybeParent is not None:
            tree_dict["Nodes"][nodeName]["parent"] = maybeParent.name
        else:
            tree_dict["Nodes"][nodeName]["parent"] = 'NONE'

        # initialize the list of children and save them as is
        tree_dict["Nodes"][nodeName]["children"] = []
        for c_child in treeNode.children:
            tree_dict["Nodes"][nodeName]["children"].append(str(c_child.name))

        # if we have no children make sure to do all caps NONE again
        if len(tree_dict["Nodes"][nodeName]["children"]) == 0:
            tree_dict["Nodes"][nodeName]["children"].append("NONE")

		# Not a base function of HTT's so I'll leave this blank for now until we have a more dynamic way of doing this
        tree_dict["Nodes"][nodeName]["peers"] = ['NONE']

		# Make sure to save their coords as decided by the user so that when we load the tree it always looks the same
        tree_dict["Nodes"][nodeName]["x"] = treeNode.getX()
        tree_dict["Nodes"][nodeName]["y"] = treeNode.getY()
        
    return tree_dict

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit( app.exec_() )
