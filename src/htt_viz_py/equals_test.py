#!/usr/bin/python

# This is a file meant to test the save functionality of htt_viz
# This is not meant to be shipped with htt_viz and is not meant to be run through ros
from NodeView import Tree, Node
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


# Note this function won't make any sense outside of when both trees are already equivalent
def random_place_both    ( tree1 , tree2):
    #so Here I want a random seed based on the time
    r.seed()
    cur_ptr = tree1.root_node;
    for i in range(r.randrange(1, 15)):
        if not cur_ptr.isLeaf():
            if cur_ptr is tree1.root_node:
                cur_ptr = cur_ptr.children[0]
            elif r.randrange(1, 10) < 5:
                break
            else:
                cur_ptr = cur_ptr.children[r.randrange(len(cur_ptr.children))]
                #if its an action we can't add a child to it
                if cur_ptr.type == '3':
                    cur_ptr = tree1.node_dict[cur_ptr.parent]
                    i = i - 1
        else:
            if cur_ptr.type == '3':
                cur_ptr = tree1.node_dict[cur_ptr.parent]
                i = i - 1
            else:
                break

    num = r.randrange(4)

    if cur_ptr is tree1.root_node and num == 3:
        num = r.randrange(3)

    robot = 0
    node_num = len(tree1.node_dict)

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
    tree1.AddNode([cur_ptr.name, nNode, False])
    tree2.AddNode([cur_ptr.name, nNode, False])


        

if __name__ == "__main__":
    #Ok so the game plan here is to make a very large tree
    #I feel like I should make a random place algo. and then abuse it for testing
    t = Tree()
    t2 = Tree()

    for i in range(950):
        
        #hardcore ish test for equivalence
        print("RUNNING TEST " + str(i) + "...")
        
        before = t.equals(t2)

        print("\tBefore place both trees are equal: " + str(before))

        random_place_both(t, t2)

        if (1 == r.randrange(100)):
            print("\tPLACING NEFARIOUS EXTRA NODE")
            random_place(t)

        after = t.equals(t2)
        
        if after:
            print("\tafter place both trees are equal!\n")
        else:
            print("\tTest " + str(i) + " failed...")

            print("\tSaving both trees to tree1.yaml, and tree2.yaml before exiting...")

            
            with open("trees/failTree1.yaml", "w") as outfile:
                yaml.dump(t.toYamlDict(), outfile)

            with open("trees/failTree2.yaml", "w") as outfile:
                yaml.dump(t2.toYamlDict(), outfile)

            print("\tExiting!!!")

            break

    print("Test complete program exiting")
