#!/usr/bin/python

import pytest 
from pytest.qt_compat import qt_api

from orgButton import organizeTreeButton
from tree import Tree, Node
import random as r

@pytest.fixture
def test(testCount, treeSize)
	for x in range(testCount):
		root = genRandomTree(treeSize)
		organizeTreeButton.organizeTree(organizeTreeButton, root)
		success = checkTree(root)

def genRandomTree(treeSize, qtbot)
	r.seed()
	cur_ptr = tree.root_node;
	for i in range(r.randrange(1, 10))
		qtbot.treeHeight(self, node)
		organizeTreeButton.organizeTree(organizeTreeButton, root)
		success = checkTree(root)
		
def checkTree(node)
	 r.seed()
    cur_ptr = tree.root_node;
	organizeTreeButton.organizeTree(organizeTreeButton, root)
    	for i in range(r.randrange(1, 10)):
        if not cur_ptr.isLeaf():
            if cur_ptr is tree.root_node:
                cur_ptr = cur_ptr.children[0]
            elif r.randrange(1, 10) < 5:
                break
            else:
                cur_ptr = cur_ptr.children[r.randrange(len(cur_ptr.children))]
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
		 
	t = qt_api.Tree(self)

	x = 100 #how many tests will be ran
	n = 100 #how many nodes there will be
	test(x,n)
		 print ("Organization Button works!!" + str(i))
		 if not tree_placed:
		 print("Test has failed")
		 
		 print ("Exiting")
		 break
	print("Test complete program exiting")
		 
