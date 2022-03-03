#!/usr/bin/python

# This is a file meant to test the save functionality of htt_viz
# This is not meant to be shipped with htt_viz and is not meant to be run through ros
from tree import Tree, Node
import yaml
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

    robot = 0

    tree.AddNode([cur_ptr, num, True])

        

if __name__ == "__main__":
    #Ok so the game plan here is to make a very large tree
    #I feel like I should make a random place algo. and then abuse it for testing
    t = Tree()

    for i in range(950):
        random_place(t)

    #t.PrintTree()
    #print("length of t's children: " + str(len(t.root_node.children)))
    with open("../../trees/saveTreeTest.yaml", "w") as outfile:
        yaml.dump(t.toYamlDict(), outfile)