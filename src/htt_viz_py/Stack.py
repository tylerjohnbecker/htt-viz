#!/usr/bin/python

# This is a file for basic Stack Stuff so that we don't have to clutter up the other files
# This stack is meant for the undo/redo functionality

class FunctionCall:
    def __init__(self, func, args):
        self.func = func
        self.args = args

    def run (self):
        # wrapper function so that the function is not automatically passed (this StackNode) as self
        self.runFunction(self.func)
        
    # should only be used inside of this class by the above run function
    def runFunction (self, func):
        # run the function with the args

        # it is implied that (with the correct self) this call is:
        #  self.func([list of args])
        # because python
        func(self.args)

# The StackNode Class: A class containing a tuple of an undo function and a redo function for an action.
class ActionNode:
    # Params:
    #   switch:     True is undo, False is redo
    #   undo_list:  List of functions that will totally undo a single action
    #   redo_list:  List of functions that will totally redo a single action
    def __init__(self, switch, undo_list, redo_list):
        #obviously save everything
        self.undo_list = undo_list
        self.redo_list = redo_list
        self.next = None
        self.switch = switch

    def run(self):
        if self.switch:
            for function in self.undo_list:
                function.run()
        else:
            for function in self.redo_list:
                function.run()

# basic stack functionality for a stack of anything
# This is meant for both the undo stack and the redo stack of the NodeView 
class Stack:

    # we don't need any params for init
    def __init__(self, id):

        # id for printing
        self.id = id
        # simply declare head as None
        self.head = None

    # Basic print method for debugging
    def print(self):
        current = self.head

        print(self.id + ":")

        while not current is None:
            print(str(current))
            current = current.next
        print("\n")

    # Basic push method for stacks
    def push(self, new_node):

        # if we don't have anything just make it the head and return
        if self.head is None:
            self.head = new_node
            return None

        new_node.next = None 
        
        # add to the front of the stack
        tmp = self.head
        self.head = new_node
        new_node.next = tmp

    # Basic pop method for stacks
    def pop(self):

        # can't pop if the stack is empty
        if self.head == None:
            return None

        # create a tmp ptr for head, move the head to its next one, and then return the tmp
        tmp = self.head
        self.head = tmp.next
        
        return tmp

    # Stack isEmpty method
    def isEmpty(self):
        # the stack is empty is the head is None
        return self.head is None