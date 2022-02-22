#!/usr/bin/env python

from htt_viz_py.Stack import Stack, ActionNode, FunctionCall
import wx

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
		self.num_children = 0

	# Quick way to define if two nodes are equal. Essentially just check all of their attributes against each other
	# This is a local definition of equivalence two nodes at different points in their trees due to different grandparents
	# will return a false positive here so that should be handled in the tree class
	def equals(self, comp_node):
		
		# If their children lists aren't the same size they aren't equal
		if not (len(self.children) == len(comp_node.children)):
		    return False
		
		# For this node it is ok to make the equivalence non recursive as the tree with handle the recursion
		# Therefore simply check the names of all the children against each other
		children = True
		for i in range(len(self.children)):
			children = children and (self.children[i].name == comp_node.children[i].name)
		
		parent_same = True
		if self.parent is not None:
			parent_same = (self.parent.name == comp_node.parent.name)
		
		# Make sure to return children first so that it short circuits and otherwise just check the attributes
		# all of these need to be true for the nodes to be equivalent
		return children \
			and (self.x == comp_node.x) \
			and (self.y == comp_node.y) \
			and (self.name == comp_node.name) \
			and parent_same 
		
	def addChild(self, nNode, index = -1):
		if index > -1 and index < len(self.children):
			self.children.insert(index, nNode)
		else:
			self.children.append(nNode)

		self.num_children = self.num_children + 1 + nNode.num_children
		cur_ptr = self.parent
		while cur_ptr is not None:
			cur_ptr.num_children = cur_ptr.num_children + 1 + nNode.num_children
			cur_ptr = cur_ptr.parent
		
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
		self.root_node = Node("ROOT_4_0_000", 50, 10) 
		self.undo_stack = Stack("UNDO")
		self.redo_stack = Stack("REDO")
		self.num_nodes = 1
		self.free_nums = [False] * 1000
		self.free_nums[0] = True			

	#recursive function for below
	def rec_equals(self, cur_ptr, cur_comp_ptr):
		# always check the two passed in first
		if (not cur_ptr.equals(cur_comp_ptr)):
			return False

		ret = True

		# This works great when they are equal but terribly when they are not
		for i in range(len(cur_ptr.children)):
			ret = ret and self.rec_equals(cur_ptr.children[i], cur_comp_ptr.children[i])
			
			# We can short-circuit if we ever get false to save some time
			if not ret:
				break

		return ret

	def getNextNum(self):
		for i in range(1000):
			if not self.free_nums[i]:
				return i
		
		return -1

	def equals (self, comp_tree):
		# this should utilize recursion, so i'll go inorder and just check all the nodes and then return the
		# answering returns all and'd together
		return self.rec_equals(self.root_node, comp_tree.root_node)
	
	def rec_copy(self, tree, cur_ptr):

		if not cur_ptr is self.root_node:
			cp = Node(cur_ptr.name, cur_ptr.x, cur_ptr.y, None)

			parent = tree.findNodeByName(cur_ptr.parent.name)

			#[parent object false]
			tree.AddNode([parent, cp, False])

		for i in cur_ptr.children:
			self.rec_copy(tree, i)

	# Creates a deep copy of the tree basically like a copy constructor would
	def copy (self):
		ret = Tree()
		
		self.rec_copy(ret, self.root_node)

		return ret

	# desc.: a helper function for toYamlDict to make sure the parents come before the children in NodeList
	def populateNodeList(self, list, cur_ptr):
		list.append(str(cur_ptr.name))
		for child in cur_ptr.children:
			self.populateNodeList(list, child)

	def recAddToList (self, dict, cur_ptr):
		
		# initialize the entry for the child
		dict["Nodes"][cur_ptr.name] = {}

		# now the mask
		dict["Nodes"][cur_ptr.name]["mask"] = {}

		# each mask is saved in the Name of the child like so (Name_type_robot_node)
		type = int(cur_ptr.name[-7:-6])
		robot = int(cur_ptr.name[-5:-4])
		node  = int(cur_ptr.name[-3:])

		dict["Nodes"][cur_ptr.name]["mask"]["type"] = type
		dict["Nodes"][cur_ptr.name]["mask"]["robot"] = robot
		dict["Nodes"][cur_ptr.name]["mask"]["node"] = node

		# Now save the parent (if its none we make sure its all caps in the file)
		if cur_ptr.parent is not None:
			dict["Nodes"][cur_ptr.name]["parent"] = cur_ptr.parent.name
		else:
			dict["Nodes"][cur_ptr.name]["parent"] = 'NONE'

		# initialize the list of children and save them as is
		dict["Nodes"][cur_ptr.name]["children"] = []
		for c_child in cur_ptr.children:
			dict["Nodes"][cur_ptr.name]["children"].append(str(c_child.name))

		# if we have no children make sure to do all caps NONE again
		if len(dict["Nodes"][cur_ptr.name]["children"]) == 0:
			dict["Nodes"][cur_ptr.name]["children"].append("NONE")

		# Not a base function of HTT's so I'll leave this blank for now until we have a more dynamic way of doing this
		dict["Nodes"][cur_ptr.name]["peers"] = ['NONE']

		# Make sure to save their coords as decided by the user so that when we load the tree it always looks the same
		dict["Nodes"][cur_ptr.name]["x"] = cur_ptr.x
		dict["Nodes"][cur_ptr.name]["y"] = cur_ptr.y

		for i in cur_ptr.children:
			self.recAddToList(dict, i)

	# desc.: create a dictionary which is ready to be saved to a yaml file for use with the trees
	def toYamlDict(self):

		# initialize overarching vars
		tree_dict = {}
		tree_dict["Nodes"] = {}
		tree_dict["NodeList"] = []

		# make sure that the nodelist is in the correct order (parents before children)
		self.populateNodeList(tree_dict["NodeList"], self.root_node)

		# iterate through each child now (could be any order, right now in alphabetical)
		self.recAddToList(tree_dict, self.root_node)

		return tree_dict

	def recFindNodeByName(self, name, cur_ptr):
		if (cur_ptr.name == name):
			return cur_ptr

		for i in cur_ptr.children:
			cp = self.recFindNodeByName(name, i)
			if cp is not None:
				return cp

		return None

	def findNodeByName(self, name):
		return self.recFindNodeByName(name, self.root_node)

	def rec_num_maintainer(self, cur_ptr, bool):
		num = int(cur_ptr.name[-3:])

		self.free_nums[num] = bool
		
		for child in cur_ptr.children:
			self.rec_num_maintainer(child, bool)

	#Params:
	#	args[0]:	name of the parent node to add to
	#	args[1]:	object created for the new node to add
	#	args[2]:	boolean representing whether or not this call needs to be added to the undo_stack
	#	args[3]:	insert index for the list of children (optional)
	def AddNode(self, args):

		#If we are just normally adding a node to the tree
		if args[2]:
			nNum = self.getNextNum()

			preceeding_0s = ''

			if ( nNum / 10 ) < 1:
				preceeding_0s = '00'
			elif ( nNum / 100) < 1:
				preceeding_0s = '0'

			args[1].name = args[1].name + "_" + preceeding_0s + str(nNum)
			self.free_nums[nNum] = True
		else:#Otherwise we might have children along with the node we are adding, and the node will already have a number
			self.rec_num_maintainer(args[1], True)

		# set the parent of the new node to the passed parent (its a reference so it will change all instances)
		args[1].parent = args[0]

		p = args[0]
		depth = 1
		if p is not None:
			while p.parent is not None:
				p = p.parent
				depth = depth + 1

		args[1].depth = depth
		# Not sure if this is necessary so I'm leaving it in (Tyler)
		if args[0] is not None:
			if len(args) == 4:
				args[0].addChild(args[1], args[3])
			else:
				args[0].addChild(args[1])

		# Case for the tree being empty
		if self.root_node == None:
			self.root_node = args[1]

		self.num_nodes = self.root_node.num_children + 1
		
		# For the initial call push to undo_stack otherwise we are using it from undo/redo
		if args[2]:
			undo = FunctionCall(self.RemoveNode, [args[1].name, False])
			redo = FunctionCall(self.AddNode, [args[0], args[1], False])

			action = ActionNode(True, [ undo ], [ redo ])

			self.undo_stack.push(action)
			self.redo_stack.clear()

	
	# desc.: recursively print the nodes in the subtree starting at node_name
	def PrintNodes(self, cur_ptr):
		
		dashes = ''

		for i in range(cur_ptr.depth):
			dashes = dashes + '- '

		print(dashes + cur_ptr.name)

		for child in cur_ptr.children:
			self.PrintNodes(child)

	# desc.: print the nodes starting from the root
	def PrintTree(self):
		self.PrintNodes(self.root_node)

	# desc.: essentially just a call to remove node the root thereby completely destroying the tree
	def DestroyTree(self):
		if len(self.root_node.children) > 0:
			self.RemoveNode([ self.root_node.children[0].name, False] )

	# Params:
	#	args[0]:	name of the node being removed
	#	args[1]:	boolean representing whether we need to make a new entry 
	#				to the undo_stack(starts as true and should always be false after)
	def RemoveNode(self, args):
		if (args[0] == "ROOT_4_0_000"):
			return

		to_remove = self.findNodeByName(args[0])
		parent = to_remove.parent

		i = -1
		for i in range(len(parent.children)):
			if parent.children[i].name == to_remove.name:
				parent.children.remove(parent.children[i])
				break

		#update the number of nodes for each ancestor of this node
		cur_ptr = to_remove.parent
		while cur_ptr is not None:
			cur_ptr.num_children = cur_ptr.num_children - (to_remove.num_children + 1)
			cur_ptr = cur_ptr.parent

		to_remove.parent = None
		
		self.num_nodes = self.root_node.num_children + 1
		
		self.rec_num_maintainer(to_remove, False)

		# if this is the initial call to remove a node
		if args[1]:
			# The redo is easy because we just have a single node to remove
			redo = FunctionCall(self.RemoveNode, [ args[0], False ])
			# The undo is just about adding the entire subtree we just removed to the parent
			undo = FunctionCall(self.AddNode, [ parent, to_remove, False, i ])

			# The undo is already done for us because we passed a list through the recursive calls
			action = ActionNode(True, [ undo ], [ redo ])

			# Push it to the undo_stack now
			self.undo_stack.push(action)
			# Clear the redo stack because we did a new action
			self.redo_stack.clear()

	# Params:
	#	args[0]: name of the node being moved
	#	args[1]: x value to move to
	#	args[2]: y value to move to  
	#	args[3]: boolean representing whether it should be added to the undo_stack
	def MoveNode(self, args):
		prev_x = args[0].x
		prev_y = args[0].y

		args[0].x = args[1]
		args[0].y = args[2]

		if len(args) >= 4 and args[3]:
			undo = FunctionCall(self.MoveNode, [args[0], prev_x, prev_y, False])
			redo = FunctionCall(self.MoveNode, [args[0], args[1], args[2], False])

			action = ActionNode(True, [undo], [redo])

			self.undo_stack.push(action)
			self.redo_stack.clear()

	#function to draw the whole tree (wraps the recursive function)
	def draw(self, dc):
		self.drawTreeRec(dc, self.root_node)

	#recursive function to draw the tree
	def drawTreeRec(self, dc, cur_ptr):
		cur_ptr.draw(dc)
		for child in cur_ptr.children:
			self.drawTreeRec(dc, child)
