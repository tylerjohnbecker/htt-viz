#!/usr/bin/python

from tree import Tree, Node
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
                if cur_ptr.m_type.index > 3:
                    cur_ptr = cur_ptr.parent
                    i = i - 1
        else:
            if cur_ptr.m_type.index > 3:
                cur_ptr = cur_ptr.parent
                i = i - 1
            else:
                break

    num = r.randrange(3) + 1

    if cur_ptr is tree.root_node and num == 3:
        num = r.randrange(2) + 1

    tree.AddNode([cur_ptr, num, True])


if __name__ == "__main__":
    print("Testing copy constructor of NodeView.Tree")

    print("Testing a tree with 10 Nodes...")
    
    t = Tree()
    for i in range(10):
        random_place(t)

    t_copy = t.copy()

    if t.equals(t_copy):
        print("\tTest passed moving on to the next one...")
    else:
        print ("\tTest FAILED")
        print("Tree 1: ")
        t.PrintTree()
        print("Tree 1 CP: ")
        t_copy.PrintTree()

    print("Running a 2nd test with 951 nodes in the tree...")
    
    t = Tree()
    for i in range (950):
        random_place(t)

    t_copy = t.copy()
    
    if t.equals(t_copy):
        print("\tTest passed now Exiting...")
    else:
        print ("\tTest FAILED")