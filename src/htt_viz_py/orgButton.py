#!/usr/bin/env python

class organizeTreeButton():
	def __init__(self):
		super().__init__()
	
	def organizeTree(self, node):
		self.preliminarySort(self, node)
		self.finalSort(self, node)
		self.finalTouches(self, node)
		
	#Makes the first sort over the tree
	#Sorts nodes in reference to siblings and parent
	#Does not account for overlapping with cousins
	def preliminarySort(self, node):
			
		offset = 0
		length = len(node.children)
		iterator = 0
		maxWidth = 0
		
		if length % 2 == 0:
			offset = (node.getWidth() * 1.5)/2
		
		for child in node.children:
		
			child.setX(node.getX() + ((iterator - int(length/2)) * ((child.getWidth()) * 1.5)) + offset)
			if node.isRoot():
				pass
			child.setY(node.getY() + 80)
			self.preliminarySort(self, child)
			iterator+=1

	#Calls function to fix each level of the tree
	#For all levels excluding the first two
	#Since these two levels will be in perfect condition
	#always, following the preliminary sort	
	def finalSort(self, node):
		sorted = False
		height = self.treeHeight(self, node)

		if height > 2:
			for depth in range(height, 2, -1):
				self.repositionNodesByLevel(self, node, depth)

	#Finds the height of the tree
	def treeHeight(self, node):
		height = 0
		for child in node.children:
			height = max(height, self.treeHeight(self, child))
			
		return height + 1
		
	#Checks each level of the tree and spaces
	#Overlapping nodes as well as their parents
	def repositionNodesByLevel(self, node, depth):
		nodeList = []
		self.appendNodesByLevel(self, node, depth - 1, nodeList)
		parentList = []
		self.appendNodesByLevel(self, node, depth - 2, parentList)
		superParentList = []
		
		if node.isRoot():
			root = node
		else:
			root = self.getRoot(self, node)
			
		self.appendNodesByLevel(self, root, 1, superParentList)
			
		if len(nodeList) > 1:
			for x in range(1, len(nodeList)):
				difference = nodeList[x].getX() - nodeList[x -1].getX()
				if difference < (node.getWidth() * 1.5):
					relative = self.getFurthestNonRootParent(nodeList[x-1])
					relative2 = self.getFurthestNonRootParent(nodeList[x])
					if relative2 != relative:
						self.moveSubTree(self, relative, node.getWidth() * 1.5 - difference)
						self.moveAdjacentSubTrees(self, superParentList, relative, node.getWidth() * 1.5 - difference)
					else:
						self.moveSubTree(self, nodeList[x-1].parent, node.getWidth() * 1.5 - difference)
						self.moveAdjacentSubTrees(self, parentList, nodeList[x-1].parent, node.getWidth() * 1.5 - difference)
					
		nodeList.clear()
		parentList.clear()
		superParentList.clear()

	#Appends all nodes at a given depth to the given node list
	def appendNodesByLevel(self, node, depth, nodeList):
		if depth == 0:
			nodeList.append(node)
		else:
			if depth > 0:
				for child in node.children:
					self.appendNodesByLevel(self, child, depth - 1, nodeList)
			
		return depth + 1
		
	def moveSubTree(self, node, difference):
		node.setX(node.getX() - difference)
		for child in node.children:
			self.moveSubTree(self, child, difference)

	def moveAdjacentSubTrees(self, nodeList, lastNode, difference):
		last = 0
		while nodeList[last] != lastNode:
			last+=1
		
		if last >= 0:
			for x in range(last):
				self.moveSubTree(self, nodeList[x], difference)
			
		
	def getFurthestNonRootParent(node):
		while not node.parent.isRoot():
			node = node.parent
			
		return node
		
	def getRoot(self, node):
		return self.getFurthestNonRootParent(node).parent
		
	def finalTouches(self, node):
		if len(node.children) == 1:
			for child in node.children:
				self.finalTouches(self, child)
		else:
			minX = float('inf')
			maxX = float('-inf')
			for child in node.children:
				if child.getX() > maxX:
					maxX = child.getX()
				if child.getX() < minX:
					minX = child.getX()
			
			width = maxX - minX
			center = node.getX()
			difference = (minX + width/2) - center
		
			
			for child in node.children:
				self.moveSubTree(self, child, difference)
