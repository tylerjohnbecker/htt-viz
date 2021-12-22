#!/usr/bin/python

# This is a file for basic Stack Stuff so that we don't have to clutter up the other files
# This stack is meant for the undo/redo functionality

class StackNode:
    # self is obviously this node, function is a tuple of the method and object (self.RemoveNode),
    # and args is a list of the parameters for the function
    def __init__(self, func, args = None):
        #obviously save everything
        self.func = func
        self.next = None
        if not args is None:
            # the first argument is always self because of how python works with objects, so we pop it
            self.obj = args[0]
            self.args = args
            self.args.pop(0)
        else:
            # this message will only ever be seen in error
            # functions passed to this stack should always have parameters
            print("No arguments passed for function")

    def run (self):
        # wrapper function so that the function is not automatically passed (this StackNode) as self
        self.runFunc(self.func)

    # should only be used inside of this class by the above run function
    def runFunc (self, func):
        # this is for the case that we wants args to be nothing
        # if we have args
        if not self.args is None:
            # run the function with the args

            # it is implied that (with the correct self) this call is:
            #  self.func([list of args])
            # because python
            func(self.args)
        else:#if we do not have args
            self.func()

# basic stack functionality for a stack of lambdas
# This is meant for both the undo stack and the redo stack of the NodeView 
class Stack:

    # we don't need any params for init
    def __init__(self):

        # simply declare head as None
        self.head = None

    # Basic print method for debugging
    def print(self):
        current = self.head

        while not current is None:
            print(str(current))
            current = current.next

    # Basic push method for stacks
    def push(self, new_node):

        # if we don't have anything just make it the head and return
        if self.head == None:
            self.head = new_node
            return None
        
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
        self.head = self.head.next
        return tmp

    # Stack isEmpty method
    def isEmpty(self):
        # the stack is empty is the head is None
        return self.head is None