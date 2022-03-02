#!usr/bin/python

from NodeType import NodeType
import os
import glob

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
		Then.name = "Then"
		And.canHaveChildren = True
		And.name = "And"
		Or.canHaveChildren = True
		Or.name = "Or"

		self.node_master_list.append(Root)
		self.node_master_list.append(Then)
		self.node_master_list.append(And)
		self.node_master_list.append(Or)

		#returns the path of this file which is stored in the global __file__ by default
		my_path = os.path.dirname(os.path.abspath(__file__))

		#essentially this removes /src/htt_viz_py and replaces it with /include/behavior
		include_path = my_path[0:(len(my_path) - 15)] + "/include/behavior"

		behavior_types = self.read_nodes_from_folder(include_path)

		for x in behavior_types:
			self.node_master_list.append(x)

	def read_nodes_from_folder(self, folder_name):
		return_types = []
		os.chdir(folder_name)

		for file in glob.glob("*.yaml"):
			n_type = NodeType(file)

			return_types.append(n_type)

		return return_types 

	def to_string(self):
		to_print = "Author " + "\n"

		for x in self.node_master_list:
			to_print = to_print + "\t" + x.to_string() + "\n"

		return to_print

if __name__ == "__main__":
	author = ProgramAuthor()

	print(author.to_string())