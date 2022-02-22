import wx
import rospy as rp
import yaml
from yaml import Loader, Dumper
from htt_viz.srv import Update
from htt_viz.srv import UpdateResponse
from htt_viz_py.Tree import Tree, Node
from htt_viz_py.Stack import ActionNode, FunctionCall

# Custom widget design based on https://wiki.wxpython.org/CreatingCustomControls
class NodeView(wx.Panel):
	def __init__(self, parent, id=wx.ID_ANY, pos=wx.DefaultPosition, size=wx.DefaultSize, make_service=True):
		wx.Panel.__init__(self, parent, id, pos, size)

		if make_service == True :
			#initializing a rosservice to update the nodes when the colors need to change
			self.server = rp.Service('update_htt', Update, self.UpdateCallback)

		# The base node. Temporarily use filler data.
		self.tree = Tree()

		# Coords to be used for undoing movement by measuring start and end coords of a node
		self.start_x = 0.0
		self.start_y = 0.0
		self.lifted = True

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
			
			if self.lifted:
					self.start_x = self.draggingNode.x
					self.start_y = self.draggingNode.y

					self.lifted = False


		
		# Skip to allow wx to set up focus correctly
		event.Skip()
		
	def OnMouseRightDown(self, event):
		# XXX HACK XXX
		# RCMenu really should be controlled here. 
		# Instead, we will try to get the click location and store it for when the rcmenu needs it.
		self.lastRightClickX = event.GetX()
		self.lastRightClickY = event.GetY()
		
	def OnMouseLeftUp(self, event):

		if self.draggingNode is not None:
			undo = FunctionCall(self.tree.MoveNode, [self.draggingNode, self.start_x, self.start_y])
			redo = FunctionCall(self.tree.MoveNode, [self.draggingNode, event.GetX(), event.GetY()])
			self.tree.undo_stack.push( ActionNode( True, [ undo ], [ redo ] ) )
			self.tree.redo_stack.clear()


			self.draggingnode = None
			self.lifted = True

	def UpdateCallback(self, req):
		ptr = self.tree.findNodeByName(req.owner)
		ptr.activation_potential = req.activation_potential
		if req.active == True:
			ptr.color = "green"
		else:
			ptr.color = "red"

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
			if node == "ROOT_4_0_000":
				continue
			#name, x=0, y=0, nParent = None
			x = data["Nodes"][node]["x"]
			y = data["Nodes"][node]["y"]
			parent = self.tree.findNodeByName(data["Nodes"][node]["parent"])
			new_node = Node(node, x, y)
			self.tree.AddNode([ parent, new_node, False ])

		self.Refresh(False)

	def saveTree(self, file):
		yaml.dump(self.tree.toYamlDict(), file)
