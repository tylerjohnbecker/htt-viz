# This is a file meant to test if loading file will remain the same as the file saved      
import pytest

from htt_viz_py.tree import Tree, Node
import yaml
import htt_viz
import random as r



@pytest.fixture
def test_filedialog(qtbot, window):
	def handle_dialog():
		while window.
	QTimer.singleShot(500, handle_dialog)
	qt.mouseClick(tree.browseButton, QtCore.Qt.LeftButton, delay=1)

def random_sub    ( tree ):
  r.seed()
  cur_ptr = tree.root_node.childern[0]
  
  for i in range(r.randrange(1, 15)):
        if cur_ptr is tree.root_node:
            cur_ptr = cur_ptr.children[0]
        elif r.randrange(1, 10) < 5:
          break
        elif len(cur_ptr.childern) == 0:
          break
        else:
          cur_ptr = cur_ptr.childern[r.randrange(len(cur_ptr.childern))]
          
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
  
    t = Tree()

    print("Adding nodes to tree")
    for i in range(150):
        random_place(t)
        
     with open("../../trees/saveTreeTest.yaml", "w") as outfile:
        yaml.dump(t.toYamlDict(), outfile)
        
   print("saving node tree")
    data = self.taskTree.toYamlDict()
		  with open(filePath, "w") as f:
			  yaml.dump(data, f)
        
        with open(filePath, "r") as f:
				data = yaml.load(f, Loader=yaml.Loader)
				
				self.taskTreeDisplayWidget.clearTaskTree()

				self.taskTree.author.clearTypes()

				for file_name in data["NodeFileIncludes"]:
					self.taskTree.author.addNodeFromFile(file_name)
        
        random_place_both(t, t2)
        node_placed = False

        if (1 == r.randrange(100)):
            print("\tPLACING NEFARIOUS EXTRA NODE")
            node_placed = True
            random_place(t)


        if after and not node_placed:
            print("\tafter place both trees are equal!\n")
        else:
            print("\tTrees are not equal for iteration: " + str(i))

            if not node_placed:
                print("Test Failed!")
                print("\tSaving both trees to tree1.yaml, and tree2.yaml before exiting...")

                with open("trees/failTree1.yaml", "w") as outfile:
                    yaml.dump(t.toYamlDict(), outfile)

                with open("trees/failTree2.yaml", "w") as outfile:
                    yaml.dump(t2.toYamlDict(), outfile)

            print("\tExiting!!!")

            break

    print("Test complete program exiting")
