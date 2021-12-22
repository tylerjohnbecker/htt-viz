#!/usr/bin/python

class StackNode:
    def __init__(self, func, args = None):
        self.func = func
        self.next = None
        if not args is None:
            self.obj = args[0]
            self.args = args
            self.args.pop(0)
        else:
            print("No arguments passed for function")

    def run (self):
        self.runFunc(self.func)

    def runFunc (self, func):#I think it's actually doing self.self.RemoveNode(self, self.args)
        if not self.args is None:
            print(str(func))
            func(self.args)
        else:
            self.func()

#obviously all stacks in python are templated but this one is meant to hold lambda for the undo/redo functionality
class Stack:
    def __init__(self):
        self.head = None

    def push(self, new_node):

        if self.head == None:
            self.head = new_node
            return
        
        current = self.head

        while not current.next == None:
            current = current.next
            
        current.next = new_node

    def pop(self):
        if self.head == None:
            return

        tmp = self.head
        self.head = self.head.next
        return tmp

    def isEmpty(self):
        return self.head == None