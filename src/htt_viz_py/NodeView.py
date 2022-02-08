import wx
#import rospy as rp
import yaml
from yaml import Loader, Dumper
#from htt_viz.srv import Update
#from htt_viz.srv import UpdateResponse
from Stack import Stack, ActionNode, FunctionCall

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

		self.type = self.name[-7:-6]

		self.parent = nParent
		self.children = []
		self.depth = 0
		
	def addChild(self, nNode):
		self.children.append(nNode)
		
	def isLeaf(self):
		return len(self.children) < 1;

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
	

	# No params, simply initialize a Root Node for the initial tree viewing, and as well
	def __init__(self):
		self.node_dict = {"ROOT_4_0_000" : Node("ROOT_4_0_000", 50, 10)}
		self.root_node = self.node_dict["ROOT_4_0_000"] 
		self.undo_stack = Stack("UNDO")
		self.redo_stack = Stack("REDO")

	# desc.: a helper function for toYamlDict to make sure the parents come before the children in NodeList
	def populateNodeList(self, list, cur_ptr):
		list.append(str(cur_ptr.name))
		for child in cur_ptr.children:
			self.populateNodeList(list, child)

	# desc.: create a dictionary which is ready to be saved to a yaml file for use with the trees
	def toYamlDict(self):

		# initialize overarching vars
		tree_dict = {}
		tree_dict["Nodes"] = {}
		tree_dict["NodeList"] = []

		# make sure that the nodelist is in the correct order (parents before children)
		self.populateNodeList(tree_dict["NodeList"], self.root_node)

		# iterate through each child now (could be any order, right now in alphabetical)
		for child in self.node_dict:

			# initialize the entry for the child
			tree_dict["Nodes"][str(child)] = {}

			# now the mask
			tree_dict["Nodes"][str(child)]["mask"] = {}

			# each mask is saved in the Name of the child like so (Name_type_robot_node)
			type = int(child[-7:-6])
			robot = int(child[-5:-4])
			node  = int(child[-3:])

			tree_dict["Nodes"][str(child)]["mask"]["type"] = type
			tree_dict["Nodes"][str(child)]["mask"]["robot"] = robot
			tree_dict["Nodes"][str(child)]["mask"]["node"] = node

			# Now save the parent (if its none we make sure its all caps in the file)
			if not self.node_dict[str(child)].parent is None:
				tree_dict["Nodes"][str(child)]["parent"] = str(self.node_dict[child].parent)
			else:
				tree_dict["Nodes"][str(child)]["parent"] = 'NONE'

			# initialize the list of children and save them as is
			tree_dict["Nodes"][str(child)]["children"] = []
			for c_child in self.node_dict[str(child)].children:
				tree_dict["Nodes"][str(child)]["children"].append(str(c_child.name))

			# if we have no children make sure to do all caps NONE again
			if len(tree_dict["Nodes"][str(child)]["children"]) == 0:
				tree_dict["Nodes"][str(child)]["children"].append("NONE")

			# Not a base function of HTT's so I'll leave this blank for now until we have a more dynamic way of doing this
			tree_dict["Nodes"][str(child)]["peers"] = ['NONE']

			# Make sure to save their coords as decided by the user so that when we load the tree it always looks the same
			tree_dict["Nodes"][str(child)]["x"] = self.node_dict[child].x
			tree_dict["Nodes"][str(child)]["y"] = self.node_dict[child].y



		return tree_dict

	#Params:
	#	args[0]:	name of the parent node to add to
	#	args[1]:	object created for the new node to add
	#	args[2]:	boolean representing whether or not this call needs to be added to the undo_stack
	def AddNode(self, args):

		# start by adding the new node to the dictionary
		self.node_dict[args[1].name] = args[1]

		# set the parent of the new node to the passed parent (its a reference so it will change all instances)
		args[1].parent = args[0]

		p = self.node_dict[args[0]]
		depth = 1
		if args[0] != 'NONE':
			while p.parent is not None:
				p = self.node_dict[p.parent]
				depth = depth + 1

		args[1].depth = depth
		# Not sure if this is necessary so I'm leaving it in (Tyler)
		if not args[0] == "NONE":
			self.node_dict[args[0]].addChild(args[1])

		# Case for the tree being empty
		if self.root_node == None:
			self.root_node = self.node_dict[args[1].name]
		
		# For the initial call push to undo_stack otherwise we are using it from undo/redo
		if args[2]:
			undo = FunctionCall(self.RemoveNode, [args[1].name, False, [], 0])
			redo = FunctionCall(self.AddNode, [args[0], args[1], False])

			action = ActionNode(True, [ undo ], [ redo ])

			self.undo_stack.push(action)
			self.redo_stack.clear()

	
	# desc.: recursively print the nodes in the subtree starting at node_name
	def PrintNodes(self, node_name):
		
		dashes = ''

		for i in range(self.node_dict[node_name].depth):
			dashes = dashes + '- '

		print(dashes + node_name)

		for child in self.node_dict[node_name].children:
			self.PrintNodes(child.name)

	# desc.: print the nodes starting from the root
	def PrintTree(self):
		self.PrintNodes(self.root_node.name)

	# desc.: essentially just a call to remove node the root thereby completely destroying the tree
	def DestroyTree(self):
		self.RemoveNode([ self.root_node.name, False, [], 0 ] )
		self.root_node = None

	# NOTE: do not call from outside the class (Things will break)
	# Params:
	#	node_name:	name of the node currently being removed
	#	list:		list to which we are appending all of the undo functions for each node removed
	#	index:		argument used to preserve the child's place in its parent's list of children for undo
	def RemoveNodeRec(self, node_name, list, index):
		
		# get the num of children early
		num_children = len(self.node_dict[node_name].children)

		# get the params for the list insert now (to push onto the undo stack)
		parent = self.node_dict[node_name].parent
		node = self.node_dict[node_name]

		# if its not a leaf we need to delete the children first
		# children are deleted back to front so that their ordering does not mess up the loop
		# delete 0 go to 1 (there is now an element at 0 which we have skipped)
		for i in range(num_children - 1, -1, -1):
			# make sure to put the name to delete in a list
			self.RemoveNode([ self.node_dict[node_name].children[i].name, False, list , i])

		# make extra sure the children are gone
		self.node_dict[node_name].children = []
		
		# now pop the node_name from the dictionary as it should be fully gone
		self.node_dict.pop(node_name)

		# insert this action into the list for undo
		list.insert(index, FunctionCall(self.AddNode, [parent, node, False]))

	# Params:
	#	args[0]:	name of the node being removed
	#	args[1]:	boolean representing whether we need to make a new entry to the undo_stack(starts as true and should always be false after)
	#	args[2]:	list of all nodes removed as undo functions (redo is only 1 function call)
	#	args[3]:	index to pass to children so that they remain in the same order
	def RemoveNode(self, args):
		
		# first delete ourselves in the parent's list of children
		if not self.node_dict[args[0]].parent == None and not self.node_dict[args[0]].parent == "NONE":
			for child in self.node_dict[self.node_dict[args[0]].parent].children:
				if child.name == args[0]:
					self.node_dict[self.node_dict[args[0]].parent].children.remove(child)
					break
		
		# Recursive function to delete all children
		self.RemoveNodeRec(args[0], args[2], args[3])

		# if this is the initial call to remove a node
		if args[1]:
			# The redo is easy because we just have a single node to remove
			redo = FunctionCall(self.RemoveNode, [ args[0], False , [], 0 ])

			# The undo is already done for us because we passed a list through the recursive calls
			action = ActionNode(True, args[2], [ redo ])

			# Push it to the undo_stack now
			self.undo_stack.push(action)
			# Clear the redo stack because we did a new action
			self.redo_stack.clear()

	# Params:
	#	args[0]: name of the node being moved
	#	args[1]: x value to move to
	#	args[2]: y value to move to
	def MoveNode(self, args):
		self.node_dict[args[0]].x = args[1]
		self.node_dict[args[0]].y = args[2]

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
			undo = FunctionCall(self.tree.MoveNode, [self.draggingNode.name, self.start_x, self.start_y])
			redo = FunctionCall(self.tree.MoveNode, [self.draggingNode.name, event.GetX(), event.GetY()])
			self.tree.undo_stack.push( ActionNode( True, [ undo ], [ redo ] ) )
			self.tree.redo_stack.clear()


			self.draggingnode = None
			self.lifted = True

	def UpdateCallback(self, req):
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
			self.tree.AddNode([ parent, new_node, False ])

		self.Refresh(False)

	def saveTree(self, file):
		yaml.dump(self.tree.toYamlDict(), file)
