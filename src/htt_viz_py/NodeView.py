import wx
import rospy as rp
import yaml
from yaml import Loader, Dumper
from htt_viz.srv import Update
from htt_viz.srv import UpdateResponse

NODE_WIDTH = 120
NODE_HEIGHT = 25
NODE_RADIUS = 10

class Node:

	def __init__(self, name, x=0, y=0, nParent = None):
		self.name = name
		self.x = x
		self.y = y
		self.activation_potential = 0.0
		self.color = "red"

		self.parent = nParent
		self.children = []
		
	def addChild(self, nNode):
		self.children.append(nNode)
		
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
		
		dc.SetBrush(wx.Brush(self.color))
		dc.DrawRoundedRectangle(self.x, self.y, nodeWidth, nodeHeight, NODE_RADIUS)
		textWidth, textHeight = dc.GetTextExtent(self.name)
		dc.DrawText(self.name, (nodeWidth / 2) - (textWidth / 2) + self.x, (nodeHeight / 2) - (textHeight / 2) + self.y)
		
		for child in self.children:
			child.draw(dc)
			dc.DrawLine(self.x + (nodeWidth / 2), self.y + nodeHeight, child.x + (nodeWidth / 2), child.y)

class Tree:
	

	#I dunno what we want to pass in here yet but we can now construct trees from scratch
	def __init__(self):
		self.node_dict = {"ROOT_4_0_000" : Node("ROOT_4_0_000", 50, 10)}
		self.root_node = self.node_dict["ROOT_4_0_000"] 

	def populateNodeList(self, list, cur_ptr):
		list.append(str(cur_ptr.name))
		for child in cur_ptr.children:
			self.populateNodeList(list, child)

	def toYamlDict(self):
		tree_dict = {}
		tree_dict["Nodes"] = {}
		tree_dict["NodeList"] = []

		self.populateNodeList(tree_dict["NodeList"], self.root_node)

		for child in self.node_dict:
			#tree_dict["NodeList"].append(str(child))
			tree_dict["Nodes"][str(child)] = {}
			tree_dict["Nodes"][str(child)]["mask"] = {}

			type = int(child[-7:-6])
			robot = int(child[-5:-4])
			node  = int(child[-3:])

			tree_dict["Nodes"][str(child)]["mask"]["type"] = type
			tree_dict["Nodes"][str(child)]["mask"]["robot"] = robot
			tree_dict["Nodes"][str(child)]["mask"]["node"] = node

			if not self.node_dict[str(child)].parent == None:
				tree_dict["Nodes"][str(child)]["parent"] = str(self.node_dict[child].parent)
			else:
				tree_dict["Nodes"][str(child)]["parent"] = 'NONE'

			tree_dict["Nodes"][str(child)]["children"] = []
			for c_child in self.node_dict[str(child)].children:
				tree_dict["Nodes"][str(child)]["children"].append(str(c_child.name))

			if len(tree_dict["Nodes"][str(child)]["children"]) == 0:
				tree_dict["Nodes"][str(child)]["children"].append("NONE")

			#Not a base function of HTT's so I'll leave this blank for now until we have a more dynamic way of doing this
			tree_dict["Nodes"][str(child)]["peers"] = ['NONE']
			tree_dict["Nodes"][str(child)]["x"] = self.node_dict[child].x
			tree_dict["Nodes"][str(child)]["y"] = self.node_dict[child].y



		return tree_dict

	def AddNode(self, parent_name, node):
		self.node_dict[node.name] = node
		node.parent = parent_name
		if not parent_name == "NONE":
			self.node_dict[parent_name].addChild(node)

		if self.root_node == None:
			self.root_node = self.node_dict[node.name]

	
	def PrintNodes(self, node_name):
		
		print(node_name)
		if len(self.node_dict[node_name].children) == 0:
			return

		for child in self.node_dict[node_name].children:
			self.PrintNodes(child.name)


	def PrintTree(self):
		self.PrintNodes(self.root_node.name)

	def DestroyTree(self):
		self.RemoveNode(self.root_node.name)
		self.root_node = None

	#should be private (won't work unless called like how its called in RemoveNode())
	def RemoveNodeRec(self, node_name):
		num_children = len(self.node_dict[node_name].children)

		#if its not a leaf we need to delete the children first
		for i in range(num_children - 1, -1, -1):
			self.RemoveNode(self.node_dict[node_name].children[i].name)

		self.node_dict[node_name].children = []
		
		self.node_dict.pop(node_name)

	def RemoveNode(self, node_name):
		
		#first delete ourselves in the parent's list of children
		if not self.node_dict[node_name].parent == None and not self.node_dict[node_name].parent == "NONE":
			for child in self.node_dict[self.node_dict[node_name].parent].children:
				if child.name == node_name:
					self.node_dict[self.node_dict[node_name].parent].children.remove(child)
					break

		#then delete ourself and children recursively
		self.RemoveNodeRec(node_name)

	#function to draw the whole tree (wraps the recursive function)
	def draw(self, dc):
		self.drawTreeRec(dc, self.root_node.name)

	#recursive function to draw the tree
	def drawTreeRec(self, dc, node_name):
		self.node_dict[node_name].draw(dc)
		for child in self.node_dict[node_name].children:
			self.drawTreeRec(dc, child.name)

# Custom widget design based on https://wiki.wxpython.org/CreatingCustomControls
class NodeView(wx.Panel):
	def __init__(self, parent, id=wx.ID_ANY, pos=wx.DefaultPosition, size=wx.DefaultSize, make_service=True):
		wx.Panel.__init__(self, parent, id, pos, size)

		if make_service == True :
			#initializing a rosservice to update the nodes when the colors need to change
			self.server = rp.Service('update_htt', Update, self.UpdateCallback)

		# The base node. Temporarily use filler data.
		self.tree = Tree()

		#n1 = Node("THEN_0_0_001", 150, 50)
		#n2 = Node("BEHAVIOR_3_0_002", 100, 100)

		#self.tree.AddNode("ROOT_4_0_000", n1)
		#self.tree.AddNode("THEN_0_0_001", n2)

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
		self.Bind(wx.EVT_RIGHT_DOWN, self.OnMouseRightDown)
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
		self.tree.draw(dc)
		
	def OnMouseLeftDown(self, event):
		eventX = event.GetX()
		eventY = event.GetY()

		maybeNode = self.tree.root_node.getHitNode(eventX, eventY)
		
		self.draggingNode = maybeNode

		if maybeNode is not None:
			self.dragOffsetX = eventX - maybeNode.x
			self.dragOffsetY = eventY - maybeNode.y
		
		# Skip to allow wx to set up focus correctly
		event.Skip()
		
	def OnMouseRightDown(self, event):
		# XXX HACK XXX
		# RCMenu really should be controlled here. 
		# Instead, we will try to get the click location and store it for when the rcmenu needs it.
		self.lastRightClickX = event.GetX()
		self.lastRightClickY = event.GetY()
		
	def OnMouseLeftUp(self, event):
		pass

	def UpdateCallback(self, req):
		#message received to update, can handle color changes of nodes here
		#for now it will just print hello
		self.tree.node_dict[req.owner].activation_potential = req.activation_potential
		if req.active == True:
			self.tree.node_dict[req.owner].color = "green"
		else:
			self.tree.node_dict[req.owner].color = "red"

		self.Refresh(False)
		return UpdateResponse(True)
		
	def OnMouseMotion(self, event):
		if event.Dragging():
			if event.LeftIsDown() and self.draggingNode is not None:
				self.draggingNode.x = event.GetX() - self.dragOffsetX
				self.draggingNode.y = event.GetY() - self.dragOffsetY
				
				self.Refresh(False)

	def saveTree(self):
		yaml_dict = 0
		return yaml_dict

	def loadTree(self, file):
		data = yaml.load(file, Loader=Loader)

		#prompt to save changes if there are any? Here or before the open dialogue

		#so a note for myself the yaml loads as a dictionary of dictionaries all the way down
		#destroy the tree
		self.tree.DestroyTree()
		
		#iterate through all the nodes we need to make
		for node in data["NodeList"]:
			#name, x=0, y=0, nParent = None
			x = data["Nodes"][node]["x"]
			y = data["Nodes"][node]["y"]
			parent = data["Nodes"][node]["parent"]
			new_node = Node(node, x, y)
			self.tree.AddNode(parent, new_node)

		self.Refresh(False)
    
	def saveTree(self, file):
		yaml.dump(self.tree.toYamlDict(), file)
