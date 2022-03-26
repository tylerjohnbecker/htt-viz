#!/usr/bin/env python3

from htt_viz_py.Param import Param, ParamInt, ParamBool, ParamFloat, ParamString

import glob, os
import yaml
from yaml import Loader, Dumper

class NodeType(object): 

	def __init__(self, file_name):
		self.params = []
		self.name = ""
		self.source = ""
		self.index = 0

		self.file = ""
		self.canHaveParents = True
		self.canHaveChildren = False

		if not file_name == "":
			self.read_params_from_file(file_name)


	def copy_params_list(self):
		n_list = []

		for i in self.params:
			n_param = None
			if i.type == "float":
				n_param = ParamFloat()
			elif i.type == "string":
				n_param = ParamString()
			elif i.type == "int":
				n_param = ParamInt()
			elif i.type == "bool":
				n_param = ParamBool()
			else:
				n_param = Param()

			n_param.copy(i)
			n_list.append(n_param)

		return n_list

	def to_string(self):
		to_print = "NodeType " + self.name + ": " + self.source + ", " + self.file + "\n"

		for x in self.params:
			to_print = to_print + "\t" + x.to_string() + "\n"

		return to_print

	def read_params_from_file (self, file_name):

		self.params = []
		self.file = file_name

		with open(file_name, "r") as file: 
			data = yaml.load(file, Loader=Loader)
			self.source = data["node"]["source"]
			self.name = data["node"]["name"]

			for par_name in data["node"]["params"]["param_names"]:
				par_type = data["node"]["params"][par_name]

				if par_type == "float":
					new_par = ParamFloat()
					new_par.type = par_type
					new_par.name = par_name
					self.params.append(new_par)
				elif par_type == "string":
					new_par = ParamString()
					new_par.type = par_type
					new_par.name = par_name
					self.params.append(new_par)
				elif par_type == "int":
					new_par = ParamInt()
					new_par.type = par_type
					new_par.name = par_name
					self.params.append(new_par)
				elif par_type == "bool":
					new_par = ParamBool()
					new_par.type = par_type
					new_par.name = par_name
					self.params.append(new_par)
				else:
					print("Error in " + file_name + "...")
					print("Parameter [" + par_name + "] has unhandled type [" + par_type + "]")
					print("Creating a parameter with no type instead")
					new_par = Param()
					new_par.type = par_type
					new_par.name = par_name
					self.params.append(new_par)

	def write_params_to_file (self, file_name):
		yamlDict = {}

		yamlDict["node"] = {}

		yamlDict["node"]["source"] = "move_behavior.h" 

		yamlDict["node"]["params"] = {}
		yamlDict["node"]["params"]["param_names"] = []

		for x in self.params:
			yamlDict["node"]["params"]["param_names"].append(x.name)
			yamlDict["node"]["params"][x.name] = x.type

		with open(file_name, "w") as file:
			yaml.dump(yamlDict, file)

def read_nodes_from_folder(folder_name):
	return_types = []
	os.chdir(folder_name)

	for file in glob.glob("*.yaml"):
		n_type = NodeType(file)

		return_types.append(n_type)

	return return_types 




if __name__ == "__main__":

	#returns the path of this file which is stored in the global __file__ by default
	my_path = os.path.dirname(os.path.abspath(__file__))

	#essentially this removes /src/htt_viz_py and replaces it with /include/behavior
	include_path = my_path[0:(len(my_path) - 15)] + "/include/behavior"

	types = read_nodes_from_folder(include_path)

	for x in types: 
		print(x.to_string())