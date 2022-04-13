#!/usr/bin/python

import yaml
from yaml import Loader, Dumper

class Param(object):

    def __init__(self):
        self.name = ""
        self.type = "none"
        self.value = None

    def copy(self, copy_param):
        self.name = copy_param.name
        self.type = copy_param.type
        self.value = copy_param.value

    def equals(self, comp_param):
        return  self.name == comp_param.name \
            and self.type == comp_param.type \
            and self.value == comp_param.value

    def setValue(self, n_value):
        self.value = n_value

    def toString(self):
        return "Param "+ self.name + ": " + self.type + ", " + str(self.value)

class ParamFloat (Param):

    def __init__(self):
        super(Param, self).__init__()

        self.type = "float"
        self.value = 0.0

    # All of these assume a string as input and save it in a different way
    def setValue(self, n_value):
        self.value = float(n_value)


class ParamString (Param):
    
    def __init__(self):
        super(Param, self).__init__()

        self.type = "string"
        self.value = ""


class ParamInt (Param):
    
    def __init__(self):
        super(Param, self).__init__()

        self.type = "int"
        self.value = 0
    
    def setValue(self, n_value):
        self.value = int(n_value)

class ParamBool (Param):
    
    def __init__(self):
        super(Param, self).__init__()

        self.type = "bool"
        self.value = False


def read            (file_name):
    ret = []
    
    with open(file_name, "r") as file: 
        data = yaml.load(file, Loader=Loader)
        for par_name in data["node"]["params"]["param_names"]:
            par_type = data["node"]["params"][par_name]

            if par_type == "float":
                new_par = ParamFloat()
                new_par.type = par_type
                new_par.name = par_name
                ret.append(new_par)
            elif par_type == "string":
                new_par = ParamString()
                new_par.type = par_type
                new_par.name = par_name
                ret.append(new_par)
            elif par_type == "int":
                new_par = ParamInt()
                new_par.type = par_type
                new_par.name = par_name
                ret.append(new_par)
            elif par_type == "bool":
                new_par = ParamBool()
                new_par.type = par_type
                new_par.name = par_name
                ret.append(new_par)
            else:
                print("Error in file_name...")
                print("Parameter [" + par_name + "] has unhandled type [" + par_type + "]")
                print("Creating a parameter with no type instead")
                new_par = Param()
                new_par.type = par_type
                new_par.name = par_name
                ret.append(new_par)
    
    return ret

def test_read       (file_name):
    param_list = read(file_name)

    with open(file_name, "r") as file: 
        data = yaml.load(file, Loader=Loader)
        print(data["node"]["source"] + ": ")
        print("Expected values from file: ")
        for x in data["node"]["params"]["param_names"]:
            print("[" + x + "]: [" + data["node"]["params"][x] + "]")
        
        print("Values from param_list: ")
        for x in param_list:
            print("[" + x.name + "]: [" + x.type + "]")

#wrapper object for this will need to hold the src file as a member of the class
def write           (params, file_name):
    
    yamlDict = {}

    yamlDict["node"] = {}

    yamlDict["node"]["source"] = "move_behavior.h" 

    yamlDict["node"]["params"] = {}
    yamlDict["node"]["params"]["param_names"] = []

    for x in params:
        yamlDict["node"]["params"]["param_names"].append(x.name)
        yamlDict["node"]["params"][x.name] = x.type

    with open(file_name, "w") as file:
        yaml.dump(yamlDict, file)


def test_write      (file_name):
    params = read(file_name)

    write(params, "/home/tylerbecker/sp_ws/src/htt-viz/include/behavior/test.yaml")

    params2 = read("/home/tylerbecker/sp_ws/src/htt-viz/include/behavior/test.yaml")

    passed = True

    if not len(params) == len(params2):
        passed = False
        print("Test failed, after a write and read the number of params are not the same!")

    for i in range(len(params)):
        passed = passed and (params[i].equals(params2[i]))

        if not passed:
            print("Test failed, after a write and a read the params are no longer equivalent!")

            break

    if passed:
        print("Test succeeded after a read-write-read the same data was loaded into the list!")

if __name__ == "__main__":
    #test the class

    # more accurately test the init function of the class's wrapper, by essentially 
    # creating a list here from a file
    test_write("/home/tylerbecker/sp_ws/src/htt-viz/include/behavior/move_behavior.yaml")