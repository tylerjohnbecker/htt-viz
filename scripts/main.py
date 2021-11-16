import wx

class frameMain ( wx.Frame ):
	
	def __init__( self, parent ):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"HTT-VIZ", pos = wx.DefaultPosition, size = wx.Size( 1000,550 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
		
		self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
		
		## Split window into panels
		splitter = wx.SplitterWindow(self)
		left = NodePanel(splitter)
		right = TreePanel(splitter)
		splitter.SplitVertically(left, right)
		splitter.SetMinimumPaneSize(200)
		
		self.Layout()
		self.menubarMain = wx.MenuBar( 0 )
		self.menuFile = wx.Menu()
		self.menuItemFileNew = wx.MenuItem( self.menuFile, wx.ID_ANY, u"New"+ u"\t" + u"Ctrl+N", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.AppendItem( self.menuItemFileNew )
		
		self.menuItemFileOpen = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Open"+ u"\t" + u"Ctrl+O", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.AppendItem( self.menuItemFileOpen )
		
		self.menuItemFileSave = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Save"+ u"\t" + u"Ctrl+S", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.AppendItem( self.menuItemFileSave )
		
		self.menuItemFileSaveAs = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Save As"+ u"\t" + u"Ctrl-Shift+S", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.AppendItem( self.menuItemFileSaveAs )
		
		self.menuFile.AppendSeparator()
		
		self.menuItemFileExit = wx.MenuItem( self.menuFile, wx.ID_ANY, u"Exit"+ u"\t" + u"Alt-F4", wx.EmptyString, wx.ITEM_NORMAL )
		self.menuFile.AppendItem( self.menuItemFileExit )
		
		self.menubarMain.Append( self.menuFile, u"File" ) 
		
		self.menuEdit = wx.Menu()
		self.menubarMain.Append( self.menuEdit, u"Edit" ) 
		
		self.menuView = wx.Menu()
		self.menubarMain.Append( self.menuView, u"View" ) 
		
		self.m_menuHelp = wx.Menu()
		self.menubarMain.Append( self.m_menuHelp, u"Help" ) 
		
		self.SetMenuBar( self.menubarMain )
		
		
		self.Centre( wx.BOTH )
		
		# Connect Events
		self.Bind( wx.EVT_MENU, self.menuItemFileNewOnMenuSelection, id = self.menuItemFileNew.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemFileOpenOnMenuSelection, id = self.menuItemFileOpen.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemFileSaveOnMenuSelection, id = self.menuItemFileSave.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemFileSaveAsOnMenuSelection, id = self.menuItemFileSaveAs.GetId() )
		self.Bind( wx.EVT_MENU, self.menuItemFileExitOnMenuSelection, id = self.menuItemFileExit.GetId() )
	
	def __del__( self ):
		pass
	
	
	# Virtual event handlers, overide them in your derived class
	def menuItemFileNewOnMenuSelection( self, event ):
		event.Skip()
	
	def menuItemFileOpenOnMenuSelection( self, event ):
		event.Skip()
	
	def menuItemFileSaveOnMenuSelection( self, event ):
		event.Skip()
	
	def menuItemFileSaveAsOnMenuSelection( self, event ):
		event.Skip()
	
	def menuItemFileExitOnMenuSelection( self, event ):
		event.Skip()

class NodePanel(wx.Panel):
	def __init__(self, parent):
		wx.Panel.__init__(self, parent = parent)
		wx.Button(self, -1, "New Node")
		self.SetBackgroundColour("grey")

class TreePanel(wx.Panel):
	def __init__(self, parent):
		wx.Panel.__init__(self, parent = parent)
		wx.Button(self, -1, "Run Tree")
		self.SetBackgroundColour("dark grey")
	
class MainApp(wx.App):
    def OnInit(self):
        mainFrame = frameMain(None)
        mainFrame.Show(True)
        return True

if __name__ == '__main__':
    app = MainApp()
    app.MainLoop()

