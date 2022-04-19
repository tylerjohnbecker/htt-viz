import os

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

def makeImg(root, fart):
	filename = "graph.gv"
	
	saveImg(filename, root, fart)
	
def createGVFile(filename):
	f = open(filename, "w")
	f.write("digraph D {\n\n")
	f.write("node [fontcolor=white color=dodgerblue2 shape=box style=filled fillcolor=deepskyblue]\n")
	f.close()
		
def pushNodeToGv(filename, node):
	arrowhead = "arrowhead=none"
	f = open(filename, "a")
	if node.isRoot():
		if len(node.children) == 0:
			f.write(node.name + "[" + arrowhead + "]\n")
	else:
		f.write(node.parent.name + " -> " + node.name + "[" + arrowhead + "]\n")
		
	for child in node.children:
		pushNodeToGv(filename, child)
		
	f.close()

def endGVFile(filename):
	f = open(filename, "a")
	f.write("}")	
	f.close()
	
def saveImg(filename, root, fart):
	dialog = QFileDialog()
	dialog.setFilter(dialog.filter() | QtCore.QDir.Hidden)
	dialog.setDefaultSuffix('png')
	dialog.setAcceptMode(QFileDialog.AcceptSave)
	dialog.setNameFilters(['PNG (*.png)'])
	
	if dialog.exec_() == QDialog.Accepted:
		filepath = dialog.selectedFiles()[0]
		
		path = filepath
		for x in range(len(filepath) - 1, 0, -1):
			if filepath[x] == '/':
				path = filepath[0:x+1]
				break
				
		path = path + filename
		
		createGVFile(path)
		pushNodeToGv(path, root)
		endGVFile(path)
		os.system("dot -Tpng " + path + " -o " + filepath)
		
		os.remove(path)
		
