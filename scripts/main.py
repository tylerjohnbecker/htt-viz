#!/usr/bin/python

import wx
import wx.stc
import wx.richtext
import rospy
import threading
import random
from htt_viz_py.NodeView import NodeView
from htt_viz_py.NodeView import Tree
from htt_viz_py.NodeView import Node
from rosgraph_msgs.msg import Log
	
class frameMain ( wx.Frame ):
	def __init__( self, parent , make_service):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"HTT-VIZ", pos = wx.DefaultPosition, size = wx.Size( 1000,550 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )

		self.SetSizeHints( 500, 500 )
		
		self.title = "HTT-VIZ"
		self.star = False
		self.pathname = None

		## Split window into panels
		splitter = wx.SplitterWindow(self)
		self.left = NodePanel(splitter)
		self.right = TreePanel(splitter, make_service)
		splitter.SplitVertically(self.left, self.right)
		splitter.SetMinimumPaneSize(200)
		
		splitter.Bind(wx.EVT_RIGHT_DOWN, self.OnRightDown)
		
		# Menu Buttons
		self.Layout()
		self.menubarMain = wx.MenuBar( 0 )
		self.menuFile = wx.Menu()

		# New
		self.menuItemFileNew = wx.MenuItem( self.menuFile, wx.ID_ANY, u"New"+ u"\t" + u"Ctrl+N", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileNew )
		
		# Open
		self.menuItemFileOpen = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Open"+ u"\t" + u"Ctrl+O", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileOpen  )
		
		# Save
		self.menuItemFileSave = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Save"+ u"\t" + u"Ctrl+S", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileSave )
		
		# Save As
		self.menuItemFileSaveAs = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Save As"+ u"\t" + u"Ctrl-Shift+S", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileSaveAs  )
		
		self.menuFile.AppendSeparator()
		
		# Exit
		self.menuItemFileExit = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Exit"+ u"\t" + u"Alt-F4", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileExit  )
		
		self.menubarMain.Append( self.menuFile, u"File" ) 
		# End File Tab

		self.menuEdit = wx.Menu()
		
		# Undo
		self.menuItemEditUndo = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Undo"+ u"\t" + u"Ctrl+Z", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditUndo )

		# Redo
		self.menuItemEditRedo = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Redo"+ u"\t" + u"Ctrl+Y", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditRedo  )

		self.menuEdit.AppendSeparator()

		# Cut
		self.menuItemEditCut = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Cut"+ u"\t" + u"Ctrl+X", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditCut  )

		# Copy
		self.menuItemEditCopy = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Copy"+ u"\t" + u"Ctrl+C", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditCopy  )

		# Paste
		self.menuItemEditPaste = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Paste"+ u"\t" + u"Ctrl+V", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditPaste  )

		self.menubarMain.Append( self.menuEdit, u"Edit" ) 
		# End Edit Tab


		self.menuView = wx.Menu()

		self.menuItemViewDebug = wx.MenuItem( self.menuView, wx.ID_ANY, u"Debug Console"+ u"\t" + u"Ctrl+Shift+Y", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuView.Append( self.menuItemViewDebug  )

		self.menubarMain.Append( self.menuView, u"View" ) 
		
		self.menuHelp = wx.Menu()

		# Help
		self.menuItemHelpAbout = wx.MenuItem( self.menuHelp, wx.ID_ANY, u"About..."+ u"\t" + u"", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuHelp.Append( self.menuItemHelpAbout  )

		self.menubarMain.Append( self.menuHelp, u"Help" ) 
		# End Help Tab
		
		self.SetMenuBar( self.menubarMain )
		
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.Bind( wx.EVT_MENU, self.menuItemFileNewOnMenuSelection, id = self.menuItemFileNew.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemFileOpenOnMenuSelection, id = self.menuItemFileOpen.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemFileSaveOnMenuSelection, id = self.menuItemFileSave.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemFileSaveAsOnMenuSelection, id = self.menuItemFileSaveAs.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemFileExitOnMenuSelection, id = self.menuItemFileExit.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemViewConsoleOnMenuSelection, id = self.menuItemViewDebug.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemHelpAboutOnMenuSelection, id = self.menuItemHelpAbout.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemEditUndoOnMenuSelection, id = self.menuItemEditUndo.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemEditRedoOnMenuSelection, id = self.menuItemEditRedo.GetId() )

		self.Bind(wx.EVT_CLOSE, self.exitEvent)
	
	def __del__( self ):
		pass

	def AddStar(self):
		if self.star == False:
			self.star = True
			self.SetTitle(self.title + "*")

	def ResetStar(self):
		self.star = False
		self.SetTitle(self.title)

	# Handles the creation of new frames
	def new_frame(self, event):
		frame = frameNew(None, make_service = False)
	
	# Virtual event handlers, overide them in your derived class
	def menuItemFileNewOnMenuSelection( self, event ):
		self.new_frame(event)
	
	# Open File Event
	# Referenced from: https://wxpython.org/Phoenix/docs/html/wx.FileDialog.html
	def menuItemFileOpenOnMenuSelection( self, event ):
		#if self.contentNotSaved:
		#	if wx.MessageBox("Current content has not been saved! Proceed?", "Please confirm", wx.ICON_QUESTION | wx.YES_NO, self) == wx.NO:
		#		return
		with wx.FileDialog(self, "Open file", wildcard="File Types (*.yaml)|*.yaml", style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST) as fileDialog:
			if fileDialog.ShowModal() == wx.ID_CANCEL:
				return

			self.pathname = fileDialog.GetPath()
	
			try:
				with open(self.pathname, 'r') as file:
					self.right.treeEditor.loadTree(file)
					self.ResetStar()
					self.right.treeEditor.tree.undo_stack.clear()
					self.right.treeEditor.tree.redo_stack.clear()
			except IOError:
				wx.LogError("Cannot open file.")

	# Save Event
	def menuItemFileSaveOnMenuSelection( self, event ):
		#with wx.FileDialog(self, "Save file", wildcard="File Types (*.yaml)|*.yaml", style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT) as fileDialog:
			
		#if fileDialog.ShowModal() == wx.ID_CANCEL:
		#	return
			
		if self.pathname == None:
			self.menuItemFileSaveAsOnMenuSelection(event)
			return

		#self.status("Saving content")
		try:
			with open(self.pathname, 'w') as file:
				self.right.treeEditor.saveTree(file)
				self.ResetStar()
		except IOError:
			wx.LogError("Cannot save current data '%s'." % self.pathname)
		event.Skip()
	
	# Save As Event
	# Referenced from: https://wxpython.org/Phoenix/docs/html/wx.FileDialog.html	
	def menuItemFileSaveAsOnMenuSelection( self, event ):
		with wx.FileDialog(self, "Save file as", wildcard="File Types (*.yaml)|*.yaml", style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT) as fileDialog:

			if fileDialog.ShowModal() == wx.ID_CANCEL:
				return     

			self.pathname = fileDialog.GetPath()

			try:
				with open(self.pathname, 'w') as file:
					self.right.treeEditor.saveTree(file)
					self.ResetStar()
			except IOError:
				wx.LogError("Cannot save current data in file '%s'." % pathname) 
		
	# Exit Event
	def menuItemFileExitOnMenuSelection( self, event ):
		wx.Exit()

	def menuItemViewConsoleOnMenuSelection(self, event):
		console = consoleWindow()
		console.Show()

	def menuItemHelpAboutOnMenuSelection( self, event ):
		about = aboutWindow()
		about.Show()
	
	def menuItemEditUndoOnMenuSelection( self, event ):
		obj = self.right.treeEditor.tree.undo_stack.pop()

		if not obj is None:
		   obj.run()
		   obj.switch = False
		   obj.next = None
		   self.right.treeEditor.tree.redo_stack.push(obj)

		self.right.treeEditor.Refresh(False)

	def menuItemEditRedoOnMenuSelection( self, event ):
		obj = self.right.treeEditor.tree.redo_stack.pop()

		if not obj is None:
		   obj.run()
		   obj.switch = True
		   obj.next = None 
		   self.right.treeEditor.tree.undo_stack.push(obj)

		self.right.treeEditor.Refresh(False)

	# Function to display Right Click Menu
	def OnRightDown(self, e):
		self.PopupMenu(RCMenu(self))

	def exitEvent(self, event):
		rospy.signal_shutdown("")
		self.Destroy()

# Right-Click Menu Class
class RCMenu(wx.Menu):
	def __init__(self, parent):
		super(RCMenu, self).__init__()

		self.parent = parent
		
		# Menu Options and links to behaviors
		addmenu = wx.MenuItem(self, wx.ID_ANY, 'Add Child Node')
		self.Append(addmenu )
		self.Bind(wx.EVT_MENU, self.OnAddChildNode, addmenu)

		editmenu = wx.MenuItem(self, wx.ID_ANY, 'Edit Node')
		self.Append(editmenu )
		self.Bind(wx.EVT_MENU, self.OnEditNode, editmenu)

		removemenu = wx.MenuItem(self, wx.ID_ANY, 'Remove Node')
		self.Append(removemenu )
		self.Bind(wx.EVT_MENU, self.RemoveNode, removemenu)

		closemenu = wx.MenuItem(self, wx.ID_ANY, 'Close')
		self.Append(closemenu )
		self.Bind(wx.EVT_MENU, self.OnClose, closemenu)
	
	# Behaviors for menu options
	def OnAddChildNode(self, e):
		# XXX HACK XXX
		# RCMenu should be moved to NodeView instead of doing this chain
		nodeView = self.parent.right.treeEditor
		
		# XXX HACK XXX
		# Try to get the data from the selection widget in a better way, 
		# maybe through events or shared state
		nodeList = self.parent.left.nodeList
		selectionIndex = nodeList.GetSelection()
		selection = nodeList.GetString(selectionIndex)
		
		maybeNode = nodeView.tree.root_node.getHitNode(nodeView.lastRightClickX, nodeView.lastRightClickY)
		
		if maybeNode is not None:
			# TODO: Make a function to do all these steps on nodeview
			num = 3

			if selection == 'AND':
				num = 2
			elif selection == 'OR':
				num = 1
			elif selection == 'THEN':
				num = 0

			robot = 0
			node_num = len(nodeView.tree.node_dict)

			preceeding_0s = ''

			if ( node_num / 10 ) < 1:
				preceeding_0s = '00'
			elif ( node_num / 100) < 1:
				preceeding_0s = '0'

			# XXX HACK XXX
			# Node names must be unique
			node = Node(selection + '_' + str(num) + '_' + str(robot) + '_' + preceeding_0s + str(node_num), maybeNode.x, maybeNode.y + 100)
			nodeView.tree.AddNode([maybeNode.name, node, True])
			nodeView.Refresh(False)
			self.parent.AddStar()
		
		# Call 'AddNode' with selected node as parameter
		# self.AddNode(self, parent_name, node)

	def OnEditNode(self, e):
		# Edit Node
		self.parent.Iconize() # Temp behavior

	def RemoveNode(self, e):
		# XXX HACK XXX
		# RCMenu should be moved to NodeView instead of doing this chain
		nodeView = self.parent.right.treeEditor
		
		maybeNode = nodeView.tree.root_node.getHitNode(nodeView.lastRightClickX, nodeView.lastRightClickY)
		
		if maybeNode is not None:
			# TODO: Make a function to do all these steps on nodeview
			nodeView.tree.RemoveNode( [ maybeNode.name, True, [], 0 ] )
			nodeView.Refresh(False)
			self.parent.AddStar()

	def OnClose(self, e):
		self.parent.Close()

class NodePanel(wx.Panel):
	def __init__(self, parent):
		#wx.Panel.__init__(self, parent = parent)
		#wx.Button(self, -1, "New Node")
		#self.SetBackgroundColour("grey")
		wx.Panel.__init__(self, parent = parent)
		self.parent = parent
		button = wx.Button(self, -1, "New Node")
		self.SetBackgroundColour("grey")
		List = ['THEN','AND','OR','MOVE','GRAB']
		self.nodeList=wx.ListBox(self, -1, pos = (3,30), size = (194, 460), choices = List, style = wx.LB_SINGLE)

		#self.nodeList.Bind(wx.EVT_RIGHT_DOWN, self.OnRightDown)
		button.Bind(wx.EVT_RIGHT_DOWN, self.OnRightDown)
		self.Bind(wx.EVT_RIGHT_DOWN, self.OnRightDown)

	def OnRightDown(self, event):
		wx.PostEvent(self.parent, event)
		event.Skip()

class TreePanel(wx.Panel):
	def __init__(self, parent, make_service):
		wx.Panel.__init__(self, parent = parent)
		self.parent = parent
		
		#sizer to put them one above the other with no horizontal constraints
		sizer = wx.BoxSizer(wx.VERTICAL)

		#no functionality just a pretty button for now
		runButton = wx.Button(self, -1, "Run Tree")
		
		self.treeEditor = NodeView(self, wx.ID_ANY, wx.Point(15, 15), size=wx.Size(400, 400), make_service=make_service)
		
		#add them to the sizer in the correct order
		sizer.Add(runButton)
		sizer.Add(self.treeEditor, 1, wx.EXPAND)
		
		self.SetBackgroundColour("dark grey")
		self.SetSizer(sizer)
		runButton.Bind(wx.EVT_RIGHT_DOWN, self.OnRightDown)
		self.treeEditor.Bind(wx.EVT_RIGHT_DOWN, self.OnRightDown)
		self.Bind(wx.EVT_RIGHT_DOWN, self.OnRightDown)

	def OnRightDown(self, event):
		wx.PostEvent(self.parent, event)
		event.Skip()

class aboutWindow(wx.Frame):
	def __init__(self, parent=None):
		wx.Frame.__init__ (self, parent=parent, title = "About...")

		st = wx.StaticText(self, label = "Welcome to HTT-VIZ")
		st = wx.StaticText(self, label = "A graphical interface which allows the user to edit and save Hierarchical Task Trees before deploying them to the Root")
		font = st.GetFont()
		font.PointSize += 5
		font = font.Bold()
		st.SetFont(font)

		self.Show()
		font = st.GetFont()
		font.PointSize += 5
		font = font.Bold()
		st.SetFont(font)

		self.Show()

class consoleWindow(wx.Frame):
	def __init__(self, parent=None, tree_name="/task_tree_node"):
		wx.Frame.__init__(self, parent=parent, title = "ROS Console output")

		self.tn = tree_name
		self.rt = wx.richtext.RichTextCtrl(self, style=wx.VSCROLL|wx.TE_READONLY|wx.NO_BORDER)
		#font = self.st.GetFont()
		#font.PointSize += 2
		#self.st.SetFont(font)
		#self.st.StyleSetSpec(wx.stc.STC_STYLE_DEFAULT, "fore:BLACK, back:BLACK")
		#self.st.StyleSetBackground(wx.stc.STC_STYLE_DEFAULT, wx.BLACK)
		#self.st.StyleSetForeground(wx.stc.STC_STYLE_DEFAULT, wx.GREEN)
		

		self.consoleSubscriber = rospy.Subscriber("rosout", Log, self.consoleCallback)
		
		self.Show()

	def consoleCallback(self, msg):
		if msg.name == self.tn and not msg.msg == None:
			wx.CallAfter(self.rt.BeginTextColour, (0, 0, 0))
			wx.CallAfter(self.rt.AppendText, msg.msg + "\n")
			wx.CallAfter(self.rt.EndTextColour)

#class used for creating new windows with new files
class frameNew (frameMain):
	def __init__(self, title, parent=None, make_service=True) :
		frameMain.__init__(self, parent = None, make_service=make_service)
		self.Show()

class MainApp(wx.App):
    def OnInit(self):
        mainFrame = frameMain(None, True)
        mainFrame.Show(True)
        return True

class AsyncSpinner(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
	#overriden behavior of the thread
    def run(self):
		#we are only using this thread to collect the messages 
		##that are sent to this program asynchronously
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('htt_viz')
    #spinner = AsyncSpinner()
    #spinner.start()
    app = MainApp()
    app.MainLoop()
    rospy.spin()
    #rospy.signal_shutdown("")
    #spinner.join()
