#!/usr/bin/env python3

import os
import glob

from htt_viz_py.NodeType import NodeType

class ProgramAuthor(object):
	def __init__(self):
		self.node_master_list = []

		Root = NodeType("")
		Then = NodeType("")
		And = NodeType("")
		Or = NodeType("")

		Root.canHaveParents = False
		Root.canHaveChildren = True
		Root.name = "Root"

		Then.canHaveChildren = True
		Then.name = "THEN"
		And.canHaveChildren = True
		And.name = "AND"
		Or.canHaveChildren = True
		Or.name = "OR"

		self.node_master_list.append(Root)
		self.node_master_list.append(Then)
		self.node_master_list.append(Or)
		self.node_master_list.append(And)

	def maintainIndices(self):
		for i in range(len(self.node_master_list)):
			self.node_master_list[i].index = i

	#clears everything but the task nodes. Useful when loading from a file
	def clearTypes(self):
		n_master_list = []

		for i in range(4):
			n_master_list.append(self.node_master_list[i])

		temp = self.node_master_list

		self.node_master_list = n_master_list

		del temp

	#loads by default the behaviors included in htt_viz
	def loadDefault(self):
		#returns the path of this file which is stored in the global __file__ by default
		my_path = os.path.dirname(os.path.abspath(__file__))

		#essentially this removes /src/htt_viz_py and replaces it with /include/behavior
		include_path = my_path[0:(len(my_path) - 15)] + "/include/behavior"
		self.node_folder_list = [include_path]

		behavior_types = self.readNodesFromFolder(include_path)

		for x in behavior_types:
			self.node_master_list.append(x)

		self.maintainIndices()

	def readNodesFromFolder(self, folder_path):
		return_types = []
		os.chdir(folder_path)

		for file in glob.glob("*.yaml"):
			n_type = NodeType(file)

			return_types.append(n_type)

		return return_types 

	def addNodeFromFile(self, file_name):
		n_node = NodeType(file_name)

		self.node_master_list.append(n_node)
		self.maintainIndices()

	def getPathsToSave (self):
		list_to_save = []

		for i in range (4, len(self.node_master_list)):
			list_to_save.append(self.node_master_list[i].path + "/" + self.node_master_list[i].file)

		return list_to_save

	def addNodesFromFolder(self, folder_path):
		if folder_path in self.node_folder_list:
			return

		n_nodes = self.readNodesFromFolder(folder_path)

		self.node_folder_list.append(folder_path)

		for x in n_nodes:
			self.node_master_list.append(n_nodes)
		self.maintainIndices()

	def removeNodeByName(self, name):
		for i in range(len(self.node_master_list)):
			if name == self.node_master_list[i].name:
				del self.node_master_list[i]
				self.maintainIndices()
				return True

		return False

	def getNodeTypeByName(self, name):
		for x in self.node_master_list:
			if x.name == name:
				return x

		return None

	def getNodeTypeByIndex(self, index):
		return self.node_master_list[index]

	#I don't know how this is going to work so I'm going to leave this blank for now
	def loadMasterList(self, tree_list):
		pass

	#This is the last thing that I'll do for the project before we start working with the robot
	def updateLoadTree(self):
		pass

	def toString(self):
		to_print = "Author " + "\n"

		for x in self.node_master_list:
			to_print = to_print + "\t" + x.toString() + "\n"

		return to_print

if __name__ == "__main__":
	author = ProgramAuthor()

	print(author.to_string())