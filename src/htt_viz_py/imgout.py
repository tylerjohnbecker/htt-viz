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

def makeImg(root):
	filename = "graphViz/graph.gv"
	
	createGVFile(filename)
	pushNodeToGv(filename, root)
	endGVFile(filename)
	saveImg(filename)
	
def createGVFile(filename):
	f = open(filename, "w")
	f.write("digraph D {\n\n")
	f.close()
		
def pushNodeToGv(filename, node):
	shape = "shape = box"
	arrowhead = "arrowhead = none"
	f = open(filename, "a")
	if not node.isRoot():
		f.write(node.parent.name + " -> " + node.name + "[" + shape + ", " + arrowhead + "]\n")
		
	for child in node.children:
		pushNodeToGv(filename, child)
		
	f.close()

def endGVFile(filename):
	f = open(filename, "a")
	f.write("}")	
	f.close()
	
def saveImg(filename):
	os.system("dot -Tpng " + filename + " -o graphViz/file.png")
