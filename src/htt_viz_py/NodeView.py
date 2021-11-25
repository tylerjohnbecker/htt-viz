import wx
import rospy as rp
from htt_viz.srv import Update

NODE_WIDTH = 50
NODE_HEIGHT = 25
NODE_RADIUS = 10

class Node:
	parent
	children
	color

	def __init__(self, name, x=0, y=0, nParent = None):
		self.name = name
		self.x = x
		self.y = y
		
		self.parent = nParent
		self.children = []
		
	def addChild(self, name, x, y):
		self.children.append(Node(name, x, y))
		
	def getHitNode(self, x, y):
		nodeWidth = NODE_WIDTH
		nodeHeight = NODE_HEIGHT
		
		if x > self.x and x < self.x + nodeWidth and y > self.y and y < self.y + nodeHeight:
			return self
		
		for child in self.children:
			maybeNode = child.getHitNode(x, y)
			if maybeNode is not None:
				return maybeNode
		
		return None
		
	def draw(self, dc):
		nodeWidth = NODE_WIDTH
		nodeHeight = NODE_HEIGHT
		
		dc.DrawRoundedRectangle(self.x, self.y, nodeWidth, nodeHeight, NODE_RADIUS)
		textWidth, textHeight = dc.GetTextExtent(self.name)
		dc.DrawText(self.name, (nodeWidth / 2) - (textWidth / 2) + self.x, (nodeHeight / 2) - (textHeight / 2) + self.y)
		
		for child in self.children:
			child.draw(dc)
			dc.DrawLine(self.x + (nodeWidth / 2), self.y + nodeHeight, child.x + (nodeWidth / 2), child.y)

class Tree:
	node_dict = {"root" : Node("root", 0, 0)}
	root_node = node_dict["root"]

	#I dunno what we want to pass in here yet but we can now construct trees from scratch
	def __init__():
		pass 

	def AddNode(parent_name, node):
		node_dict[node.name] = node
		node_dict[parent_name].addChild(node)

	def RemoveNode(node_name):
		#base case of a leaf
		if node_dict[node_name].children.len() == 0:
			del node_dict[node_name]
			return

		#if its not a leaf we need to delete the children first
		for child in node_dict[node_name].children:
			RemoveNode(child.name)

		#then we need to find it in its parent's list and delete it there
		for child in node_dict[node_name].parent.children:
			if child.name == node_name:
				del child
				break
		#finally we need to make sure its gone from the node_dict
		del node_dict[node_name]
		


class NodeView(wx.Panel):
	def __init__(self, parent, id=wx.ID_ANY, pos=wx.DefaultPosition, size=wx.DefaultSize):
		wx.Panel.__init__(self, parent, id, pos, size)

		#initializing a rosservice to update the nodes when the colors need to change
		self.server = rp.Service('update_htt', Update, self.UpdateCallback)

		# The base node. Temporarily use filler data.
		self.node = Node("test", 0, 0)
		self.node.addChild("child 1", 100, 100)
		self.node.addChild("child 2", 100, 200)

		self.SetBackgroundColour("dark grey")
		
		# The offset for when a user begins a drag.
		self.dragOffsetX = 0
		self.dragOffsetY = 0
		
		# A reference to the dragging node
		self.draggingNode = None
		
		# Bind paint handler
		self.Bind(wx.EVT_PAINT, self.OnPaint)
		
		# Bind mouse handlers
		self.Bind(wx.EVT_LEFT_DOWN, self.OnMouseLeftDown)
		self.Bind(wx.EVT_LEFT_UP, self.OnMouseLeftUp)
		self.Bind(wx.EVT_MOTION, self.OnMouseMotion)
		
	def OnPaint(self, event):
		dc = wx.BufferedPaintDC(self)
		self.Draw(dc)
		
	def OnEraseBackground(self, event):
		pass
		
	def Draw(self, dc):
		width, height = self.GetClientSize()
		if not width or not height:
			return
		
		backColour = self.GetBackgroundColour()
		backBrush = wx.Brush(backColour, wx.SOLID)
		dc.SetBackground(backBrush)
		dc.Clear()
		
		# Testing node rendering
		self.node.draw(dc)
		
	def OnMouseLeftDown(self, event):
		eventX = event.GetX()
		eventY = event.GetY()

		maybeNode = self.node.getHitNode(eventX, eventY)
		
		self.draggingNode = maybeNode

		if maybeNode is not None:
			self.dragOffsetX = eventX - maybeNode.x
			self.dragOffsetY = eventY - maybeNode.y
		
		# Skip to allow wx to set up focus correctly
		event.Skip()
		
	def OnMouseLeftUp(self, event):
		pass

	def UpdateCallback(self, req):
		#message received to update, can handle color changes of nodes here
		#for now it will just print hello
		print("hello world!")
		
	def OnMouseMotion(self, event):
		if event.Dragging():
			if event.LeftIsDown() and self.draggingNode is not None:
				self.draggingNode.x = event.GetX() - self.dragOffsetX
				self.draggingNode.y = event.GetY() - self.dragOffsetY
				
				self.Refresh(False)