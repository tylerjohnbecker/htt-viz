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
	pushNodeToGv(root)
	for child in root.children:
		pushNodeToGv(child)
		
	saveImg()
		
def pushNodeToGv(node):
	print(node.name)
	
def saveImg():
	dialog = QFileDialog()
	dialog.setFilter(dialog.filter() | QtCore.QDir.Hidden)
	dialog.setDefaultSuffix('png')
	dialog.setAcceptMode(QFileDialog.AcceptSave)
	dialog.setNameFilters(['PNG (*.png)'])
	if dialog.exec_() != QDialog.Accepted:
		return
