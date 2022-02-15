#!/usr/bin/env python

from Stack import Stack, ActionNode, FunctionCall

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
			children = (children and (self.children[i].name == comp_node.children[i].name))

		# Make sure to return children first so that it short circuits and otherwise just check the attributes
		# all of these need to be true for the nodes to be equivalent
		return children and (self.x == comp_node.x) and (self.y == comp_node.y) and (self.parent == comp_node.parent) and (self.name == comp_node.name)
		
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

	#recursive function for below
	def rec_equals(self, cur_ptr, cur_comp_ptr):
		# always check the two passed in first
		if (not cur_ptr.equals(cur_comp_ptr)):
			return False

		ret = True

		# This works great when they are equal but terribly when they are not
		for i in range(len(cur_ptr.children)):
			ret = ret and (self.rec_equals(cur_ptr.children[i], cur_comp_ptr.children[i]))
			
			# We can short-circuit if we ever get false to save some time
			if not ret:
				break

		return ret

	def equals (self, comp_tree):
		# this should utilize recursion, so i'll go inorder and just check all the nodes and then return the
		# answering returns all and'd together
		return self.rec_equals(self.root_node, comp_tree.root_node)
	
	def rec_copy(self, tree, cur_ptr):

		if not cur_ptr is self.root_node:
			cp = Node(cur_ptr.name, cur_ptr.x, cur_ptr.y, cur_ptr.parent)

			#[parent object false]
			tree.AddNode([cp.parent, cp, False])

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
