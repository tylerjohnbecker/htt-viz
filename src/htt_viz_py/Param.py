#!/usr/bin/python

import yaml
from yaml import Loader

class Param:

    def __init__(self):
        self.name = ""
        self.type = "none"

class ParamFloat (Param):

    def __init__(self):
        super().__init__()

        self.type = "float"

    #etcs

class ParamString (Param):
    
    def __init__(self):
        super().__init__()

        self.type = "string"


class ParamInt (Param):
    
    def __init__(self):
        super().__init__()

        self.type = "int"

class ParamBool (Param):
    
    def __init__(self):
        super().__init__()

        self.type = "bool"


def read            (file_name):
    ret = []
    
    with open(file_name, "r") as file: 
        data = yaml.load(file, Loader=Loader)
        print(data["node"]["source"] + ": ")
        print("Expected values from file: ")
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

def write           (file_name):
    pass

def test_write      (file_name):
    pass

if __name__ == "__main__":
    #test the class

    # more accurately test the init function of the class's wrapper, by essentially 
    # creating a list here from a file
    test_read("C:/opt/workspace/catkin_ws/src/htt_viz/include/behavior/move_behavior.yaml")