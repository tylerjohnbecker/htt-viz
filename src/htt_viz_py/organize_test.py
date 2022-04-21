#!/usr/bin/python

import pytest 
from pytest.qt_compat import qt_api

from orgButton import organizeTreeButton
from tree import Tree, Node
import random as r

def test(testCount, treeSize)
	for x in range(testCount):
		root = genRandomTree(treeSize)
		organizeTreeButton.organizeTree(organizeTreeButton, root)
		success = checkTree(root)

def genRandomTree(treeSize)
	r.seed()
	cur_ptr = tree.root_node;
	for i in range(r.randrange(1, 25)
		       if not cur_ptr.isLeaf():
		       if
def checkTree(node)
	 r.seed()
    cur_ptr = tree.root_node;
    for i in range(r.randrange(1, 10)):
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

    tree.AddNode([cur_ptr, num, True]
		
	
if __name__ == "__main__":
		 
	t = Tree()

	x = 100 #how many tests will be ran
	n = 100 #how many nodes there will be
	test(x,n)
