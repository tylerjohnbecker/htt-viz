from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class QGraphicsTaskTreeNode(QGraphicsItem):
	def __init__(self, name, title):
		super().__init__()
		
		self.name = name

		# Visual display title
		self.title = title
		
		self.xRadius = 10.0
		self.yRadius = 10.0
		
		self.paddingX = 20.0
		self.paddingY = 20.0
		
		self.normalColor = QColor(121, 218, 255)
		self.borderColor = QColor(4, 180, 245)
		
		fontSize = 12
		self.font = QFont("Times", fontSize)
		
		font_metrics = QFontMetrics(self.font)
		self.textWidth = font_metrics.width(self.title)
		self.textHeight = font_metrics.height()
		
		# TODO: These aren't centering vertically correctly and I don't know why.
		self.width = self.textWidth + (2.0 * self.paddingX)
		self.height = self.textHeight + (2.0 * self.paddingY)
		
		# https://stackoverflow.com/questions/10950820/qt-qgraphicsitem-drag-and-drop
		self.setFlag(QGraphicsItem.ItemIsSelectable, True)
		self.setFlag(QGraphicsItem.ItemIsMovable, True)
		self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
		self.setFlag(QGraphicsItem.ItemSendsScenePositionChanges, True)
		
		self.positionChangeHandlers = []
	
	def boundingRect(self):
		return QRectF(0, 0, self.width, self.height)
		
	def paint(self, painter, option, widget):
		font_metrics = QFontMetrics(self.font)
		self.textWidth = font_metrics.width(self.title)
		self.textHeight = font_metrics.height()
		self.width = self.textWidth + (2.0 * self.paddingX)
		self.height = self.textHeight + (2.0 * self.paddingY)

		textX = (self.width / 2.0) - (self.textWidth / 2.0)
		textY = (self.height / 2.0) - (self.textHeight / 2.0)
		
		path2 = QPainterPath()
		path2.addRoundedRect(QRectF(0, 0, self.width, self.height), self.xRadius, self.yRadius)
		painter.fillPath(path2, self.borderColor)
		
		path = QPainterPath()
		path.addRoundedRect(QRectF(5, 5, self.width - 10, self.height - 10), self.xRadius, self.yRadius)
		painter.fillPath(path, self.normalColor)
		
		painter.setFont(self.font)
		painter.drawText(QPointF(textX, textY), self.title)
		
	def addPositionChangeHandler(self, handler):
		self.positionChangeHandlers.append(handler)
		
	def itemChange(self, change, value):
		ret = super().itemChange(change, value)
		
		#suffice it to say that QGraphicsItem.GraphicsItemChange.ItemPosition = 0
		if change == 0:#Hardcoded the value oof the Cpp enum to hopefully fix an error#QGraphicsItem.GraphicsItemChange.ItemPositionChange
			for handler in self.positionChangeHandlers:
				handler(value.x(), value.y())
				
		return ret

	def showActiveColor(self):
		self.borderColor = QColor(0, 0, 0)
		self.normalColor = QColor(255, 255, 255)

	def showInactiveColor(self):
		self.normalColor = QColor(121, 218, 255)
		self.borderColor = QColor(4, 180, 245)
		
	# Get the graphics width
	def getWidth(self):
		return self.width
		
	# Get the graphics height
	def getHeight(self):
		return self.height