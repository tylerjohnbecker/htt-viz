#!/usr/bin/python

# This is a file for basic Stack Stuff so that we don't have to clutter up the other files
# This stack is meant for the undo/redo functionality

# The StackNode Class: A class containing a tuple of an undo function and a redo function for an action.
class ActionNode:
    # Params:
    #   switch:     True is undo, False is redo
    #   undo_func:  function to undo action
    #   redo_func:  function to redo action
    #   args_u:     list of parameters for undo action
    #   args_r:     list of parameters for redo action
    def __init__(self, switch, undo_func, redo_func, args_u = None, args_r = None):
        #obviously save everything
        self.undo_func = undo_func
        self.redo_func = redo_func
        self.next = None
        self.switch = switch

        if not args_u is None:
            # the first argument is always self because of how python works with objects, so we pop it
            self.obj_u = args_u[0]
            self.args_u = args_u
            self.args_u.pop(0)
        else:
            # this message will only ever be seen in error
            # functions passed to this stack should always have parameters
            print("No arguments passed for Undo function")

        if not args_r is None:
            # the first argument is always self because of how python works with objects, so we pop it
            self.obj_r = args_r[0]
            self.args_r = args_r
            self.args_r.pop(0)
        else:
            # this message will only ever be seen in error
            # functions passed to this stack should always have parameters
            print("No arguments passed for function")

    def run (self):
        if self.switch:
            # wrapper function so that the function is not automatically passed (this StackNode) as self
            self.runFunc(self.undo_func)
        else:
            self.runFunc(self.redo_func)

    # should only be used inside of this class by the above run function
    def runFunc (self, func):
        # this is for the case that we wants args to be nothing
        # if we have args
        if self.switch:
            # run the function with the args

            # it is implied that (with the correct self) this call is:
            #  self.func([list of args])
            # because python
            func(self.args_u)
        else:#if we need to redo instead
            func(self.args_r)

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
            self.print()
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