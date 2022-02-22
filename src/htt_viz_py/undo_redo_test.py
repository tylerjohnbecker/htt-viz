#!/usr/bin/python

from Tree import Tree, Node
import random as r

def random_move     ( tree ):
    pass

def random_rem      ( tree ):
    r.seed()
    cur_ptr = tree.root_node.children[0]

    #should just iterate to a random spot in the tree
    for i in range(r.randrange(1, 5)):
        if cur_ptr is tree.root_node:
            cur_ptr = cur_ptr.children[0]
        elif r.randrange(1, 10) < 2:
            break
        elif len(cur_ptr.children) == 0:
            break
        else:
            cur_ptr = cur_ptr.children[r.randrange(len(cur_ptr.children))]

    
    print("Removing " + cur_ptr.name)
    tree.RemoveNode( [ cur_ptr.name, True ] )

def random_place    ( tree ):
    #so Here I want a random seed based on the time
    r.seed()
    cur_ptr = tree.root_node;
    for i in range(r.randrange(1, 15)):
        if not cur_ptr.isLeaf():
            if cur_ptr is tree.root_node:
                cur_ptr = cur_ptr.children[0]
            elif r.randrange(1, 10) < 5:
                break
            else:
                cur_ptr = cur_ptr.children[r.randrange(len(cur_ptr.children))]
                #if its an action we can't add a child to it
                if cur_ptr.type == '3':
                    cur_ptr = cur_ptr.parent
                    i = i - 1
        else:
            if cur_ptr.type == '3':
                cur_ptr = cur_ptr.parent
                i = i - 1
            else:
                break

    num = r.randrange(4)

    if cur_ptr is tree.root_node and num == 3:
        num = r.randrange(3)

    robot = 0
    node_num = tree.num_nodes

    preceeding_0s = ''

    if ( node_num / 10 ) < 1:
        preceeding_0s = '00'
    elif ( node_num / 100) < 1:
        preceeding_0s = '0'
        '_' + str(num) + '_' + str(robot) + '_' + preceeding_0s + str(node_num)
    name = ''
    if num == 3:
        name = name + 'MOVE'
    elif num == 2:
        name = name + 'AND'
    elif num == 1:
        name = name + 'OR'
    else:
        name = name + 'THEN'

    name = name + '_' + str(num) + '_' + str(robot) + '_' + preceeding_0s + str(node_num)

    nNode = Node(name, 0, 0, cur_ptr)
    tree.AddNode([cur_ptr, nNode, True])

if __name__ == "__main__":
    
    t = Tree()

    print("TEST 1: Adding undo/redo")

    print("Adding a bunch of Random nodes to the Tree")
    for i in range(100):
        random_place(t)

    print("making a copy of the full tree")
    t_copy = t.copy()

    print("making an empty tree")
    empty_copy = Tree()

    print("==Running undo until the stack is empty!")
    u_stack = t.undo_stack
    r_stack = t.redo_stack

    i = 0

    while not u_stack.isEmpty():
        func = u_stack.pop()

        func.run()
        func.switch = False
        func.next = None
        r_stack.push(func)
        i = i + 1

    print("Checking against an empty tree")
    if t.equals(empty_copy):
        print("\tUndo test [[passed]] the tree is now empty")
    else:
        print("\tTest [[Failed]]")
        t.PrintTree()

    print("==Running redo until the stack is empty")
    while not r_stack.isEmpty():
        func = r_stack.pop()

        func.run()

    print("Checking against the copy from before the undo's")
    if t.equals(t_copy):
        print("\tRedo test [[passed]] the tree is back to its original state")
    else:
        print("\tTest [[Failed]]")

    print("\n\n")

    print("Test 2: removing undo/redo")

    print("Using the same built tree from test 1")
    
    t2 = t_copy.copy()
    i = 0

    for i in range(5):
        random_rem(t2)
        if len(t2.root_node.children) == 0:
            break

    print("Removed [" + str(i) + "] nodes")

    print("Making a copy of the new tree state")
    rem_copy = t2.copy()

    print("==Undoing until the stack is empty")
    u_stack = t2.undo_stack
    r_stack = t2.redo_stack
    
    i = 0

    while not u_stack.isEmpty():
        func = u_stack.pop()
        func.run()
        func.switch = False
        func.next = None
        r_stack.push(func)
        i = i + 1

    print("Checking against the original tree")
    if t2.equals(t_copy):
        print("\tUndo test [[passed]] the tree is back to how it was")
    else:
        print("\tTest [[Failed]]")
        t2.PrintTree()
        print("Checked Against: ")
        t_copy.PrintTree()

    print("==Running redo until the stack is empty")
    while not r_stack.isEmpty():
        func = r_stack.pop()

        func.run()

    print("Checking against the copy from before the undo's")
    if t2.equals(rem_copy):
        print("\tRedo test [[passed]] the tree is back to its original state")
    else:
        print("\tTest [[Failed]]")

    print("\n\n")

    print("Test 3: moving undo/redo")

    print("Once again using the initial tree")
    t3 = t_copy.copy()

    print("Moving a random number of nodes around")
    j = 0
    for i in range(r.randrange(100)):

        random_move(t3)

        j = j + 1

    print("Moved a total of " + str(j) + " Nodes around")
    print("Copying new tree")
    t3_copy = t3.copy()

    print("==Undoing until the stack is empty")
    u_stack = t3.undo_stack
    r_stack = t3.redo_stack
    
    i = 0

    while not u_stack.isEmpty():
        func = u_stack.pop()
        func.run()
        func.switch = False
        func.next = None
        r_stack.push(func)
        i = i + 1

    print("Checking against the original tree")
    if t3.equals(t_copy):
        print("\tUndo test [[passed]] the tree is back to how it was")
    else:
        print("\tTest [[Failed]]")

    print("== Running redo until the stack is empty")
    while not r_stack.isEmpty():
        func = r_stack.pop()

        func.run()

    print("Checking against the copy from before the undo's")
    if t3.equals(t3_copy):
        print("\tRedo test [[passed]] the tree is back to its original state")
    else:
        print("\tTest [[Failed]]")