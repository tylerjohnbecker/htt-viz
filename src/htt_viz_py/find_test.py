#!/usr/bin/python

from Tree import Tree, Node
import random as r

names = []

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

    names.append(name)

    nNode = Node(name, 0, 0, cur_ptr)
    tree.AddNode([cur_ptr, nNode, False])

if __name__ == "__main__":
    
    print("Creating a random tree of size 900")
    
    t = Tree()

    for i in range(900):
        random_place(t)
    succeeded = True

    print("Now successively searching for each node out of " + str(len(names)) + " added to the tree...")
    cp = None
    for i in names:
        
        a = r.randrange(300)
        nefarious = False
        if a < 50:
            cp = t.findNodeByName(i + "I am not a real name")
            nefarious = True
        else:
            cp = t.findNodeByName(i)

        if cp is None and not nefarious:
            print("\tTest Failed because node " + i + " could not be found!")
            t.PrintTree()
            succeeded = False
            break

    if succeeded:
        print("\tTest passed")