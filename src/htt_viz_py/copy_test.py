#!/usr/bin/python

from Tree import Tree, Node
import random as r

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
                    cur_ptr = tree.node_dict[cur_ptr.parent]
                    i = i - 1
        else:
            if cur_ptr.type == '3':
                cur_ptr = tree.node_dict[cur_ptr.parent]
                i = i - 1
            else:
                break

    num = r.randrange(4)

    if cur_ptr is tree.root_node and num == 3:
        num = r.randrange(3)

    robot = 0
    node_num = len(tree.node_dict)

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
    tree.AddNode([cur_ptr.name, nNode, False])


if __name__ == "__main__":
    print("Testing copy constructor of NodeView.Tree")

    print("Testing a tree with 10 Nodes...")
    
    t = Tree()
    for i in range(10):
        random_place(t)

    t_copy = t.copy()

    if t.equals(t_copy):
        print("\tTest passed moving on to the next one...")
        print("Tree 1: ")
        t.PrintTree()
        print("Tree 1 CP: ")
        t_copy.PrintTree()
    else:
        print ("\tTest FAILED")

    print("Running a 2nd test with 951 nodes in the tree...")
    
    t = Tree()
    for i in range (950):
        random_place(t)

    t_copy = t.copy()
    
    if t.equals(t_copy):
        print("\tTest passed now Exiting...")
    else:
        print ("\tTest FAILED")