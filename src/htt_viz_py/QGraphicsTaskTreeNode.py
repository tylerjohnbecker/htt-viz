from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

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