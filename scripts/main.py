import wx
import wx.stc
import wx.richtext
import rospy
import threading
from htt_viz_py.NodeView import NodeView
from htt_viz_py.NodeView import Tree
from htt_viz_py.NodeView import Node
from rosgraph_msgs.msg import Log


class frameMain ( wx.Frame ):
	def __init__( self, parent ):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"HTT-VIZ", pos = wx.DefaultPosition, size = wx.Size( 1000,550 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
		
		self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )
		
		## Split window into panels
		splitter = wx.SplitterWindow(self)
		left = NodePanel(splitter)
		self.right = TreePanel(splitter)
		splitter.SplitVertically(left, self.right)
		splitter.SetMinimumPaneSize(200)
		
		
		# Menu Buttons
		self.Layout()
		self.menubarMain = wx.MenuBar( 0 )
		self.menuFile = wx.Menu()

		# New
		self.menuItemFileNew = wx.MenuItem( self.menuFile, wx.ID_ANY, u"New"+ u"\t" + u"Ctrl+N", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileNew )
		
		# Open
		self.menuItemFileOpen = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Open"+ u"\t" + u"Ctrl+O", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileOpen )
		
		# Save
		self.menuItemFileSave = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Save"+ u"\t" + u"Ctrl+S", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileSave )
		
		# Save As
		self.menuItemFileSaveAs = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Save As"+ u"\t" + u"Ctrl-Shift+S", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileSaveAs )
		
		self.menuFile.AppendSeparator()
		
		# Exit
		self.menuItemFileExit = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Exit"+ u"\t" + u"Alt-F4", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.Append( self.menuItemFileExit )
		
		self.menubarMain.Append( self.menuFile, u"File" ) 
		# End File Tab

		self.menuEdit = wx.Menu()
		
		# Undo
		self.menuItemEditUndo = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Undo"+ u"\t" + u"Ctrl+Z", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditUndo )

		# Redo
		self.menuItemEditRedo = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Redo"+ u"\t" + u"Ctrl+Y", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditRedo )

		self.menuEdit.AppendSeparator()

		# Cut
		self.menuItemEditCut = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Cut"+ u"\t" + u"Ctrl+X", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditCut )

		# Copy
		self.menuItemEditCopy = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Copy"+ u"\t" + u"Ctrl+C", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditCopy )

		# Paste
		self.menuItemEditPaste = wx.MenuItem( self.menuEdit, wx.ID_ANY, u"Paste"+ u"\t" + u"Ctrl+V", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuEdit.Append( self.menuItemEditPaste )

		self.menubarMain.Append( self.menuEdit, u"Edit" ) 
		# End Edit Tab


		self.menuView = wx.Menu()

		self.menuItemViewDebug = wx.MenuItem( self.menuView, wx.ID_ANY, u"Debug Console"+ u"\t" + u"Ctrl+Shift+Y", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuView.Append( self.menuItemViewDebug )

		self.menubarMain.Append( self.menuView, u"View" ) 
		
		self.menuHelp = wx.Menu()

		# Help
		self.menuItemHelpAbout = wx.MenuItem( self.menuHelp, wx.ID_ANY, u"About..."+ u"\t" + u"", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuHelp.Append( self.menuItemHelpAbout )

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
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def menuItemFileNewOnMenuSelection( self, event ):
		event.Skip()
	
	# Open File Event
	def menuItemFileOpenOnMenuSelection( self, event ):
		#if self.contentNotSaved:
		#	if wx.MessageBox("Current content has not been saved! Proceed?", "Please confirm", wx.ICON_QUESTION | wx.YES_NO, self) == wx.NO:
		#		return
		with wx.FileDialog(self, "Open file", wildcard="File Types (*.xyz)|*.xyz", style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST) as fileDialog:
			if fileDialog.ShowModal() == wx.ID_CANCEL:
				return
			pathname = fileDialog.GetPath()
			try:
				with open(pathname, 'r') as file:
					self.right.treeEditor.loadTree(file)
			except IOError:
				wx.LogError("Cannot open file.")

	
	def menuItemFileSaveOnMenuSelection( self, event ):
		event.Skip()
	
	# Save As Event
	def menuItemFileSaveAsOnMenuSelection( self, event ):
		with wx.FileDialog(self, "Save file", wildcard="File Types (*.xyz)|*.xyz", style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT) as fileDialog:

			if fileDialog.ShowModal() == wx.ID_CANCEL:
				return     

			pathname = fileDialog.GetPath()
			try:
				with open(pathname, 'w') as file:
					self.right.treeEditor.saveTree(file)
			except IOError:
				wx.LogError("Cannot save current data in file '%s'." % pathname)

	def Save_Template_As(self, e):
		openFileDialog = wx.FileDialog(self, 'Save Template As', self.save_into_directory, '',
		                                   'Content files (*.yaml; *.json)|*.yaml;*.json|All files (*.*)|*.*',
		                                   wx.FD_SAVE | wx.wx.FD_OVERWRITE_PROMPT)
										   
		if openFileDialog.ShowModal() == wx.ID_CANCEL:
			return
        
		json_ext = '.json'

		filename = openFileDialog.GetPath()
		self.status('Saving Template content...')
		h = open(filename, 'w')
        
		if filename[-len(json_ext):] == json_ext:
			h.write(self.report.template_dump_json().encode('utf-8'))
		else:
			h.write(self.report.template_dump_yaml().encode('utf-8'))
			h.close()
			self.status('Template content saved') 

		save(window, window.getGlobalSettings().getCurrentFileName())
		return True
	
	# Exit Event
	def menuItemFileExitOnMenuSelection( self, event ):
		wx.Exit()

	def menuItemViewConsoleOnMenuSelection(self, event):
		console = consoleWindow()
		console.Show()

	def menuItemHelpAboutOnMenuSelection( self, event ):
		about = aboutWindow()
		about.Show()

class NodePanel(wx.Panel):
	def __init__(self, parent):
		#wx.Panel.__init__(self, parent = parent)
		#wx.Button(self, -1, "New Node")
		#self.SetBackgroundColour("grey")
		wx.Panel.__init__(self, parent = parent)
		wx.Button(self, -1, "New Node")
		self.SetBackgroundColour("grey")
		List = ['Node A', 'Node B', 'Node C', 'Node D', 'Node E', 'Node F', 'Node G']
		NodeList=wx.ListBox(parent, -1, pos = (3,30), size = (194, 110), choices = List, style = wx.LB_SINGLE)

class TreePanel(wx.Panel):
	def __init__(self, parent):
		wx.Panel.__init__(self, parent = parent)
		
		#sizer to put them one above the other with no horizontal constraints
		sizer = wx.BoxSizer(wx.VERTICAL)

		#no functionality just a pretty button for now
		runButton = wx.Button(self, -1, "Run Tree")
		
		self.treeEditor = NodeView(self, wx.ID_ANY, wx.Point(15, 15), size=wx.Size(400, 400))
		
		#add them to the sizer in the correct order
		sizer.Add(runButton)
		sizer.Add(self.treeEditor)
		
		self.SetBackgroundColour("dark grey")
		self.SetSizer(sizer)
		#List = ['Node A', 'Node B', 'Node C', 'Node D', 'Node E', 'Node F', 'Node G']
		#NodeList=wx.ListBox(parent, -1, pos = (3,30), size = (194, 110), choices = List, style = wx.LB_SINGLE)

class aboutWindow(wx.Frame):
	def __init__(self, parent=None):
		wx.Frame.__init__ (self, parent=parent, title = "About...")

		st = wx.StaticText(self, label = "Welcome to HTT-VIZ")
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
		if msg.name == self.tn:
			self.rt.BeginTextColour((0, 122, 0))
			self.rt.WriteText(msg.msg + "\n")
			self.rt.EndTextColour()


class MainApp(wx.App):
    def OnInit(self):
        mainFrame = frameMain(None)
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
    spinner = AsyncSpinner()
    spinner.start()
    app = MainApp()
    app.MainLoop()
    rospy.signal_shutdown("")
    spinner.join()
