import wx
import graphviz



class FrameMain ( wx.Frame ):
	def __init__( self, parent ):
		wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"wxMain", pos = wx.DefaultPosition, size = wx.Size( 969,543 ), style = wx.DEFAULT_FRAME_STYLE|wx.TAB_TRAVERSAL )
		
		self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
		
		bSizerFrameMain = wx.BoxSizer( wx.VERTICAL )
		
		bSizerMainFrame = wx.BoxSizer( wx.VERTICAL )
		
		self.m_panelMain = wx.Panel( self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
		bSizerMainFrame.Add( self.m_panelMain, 1, wx.EXPAND |wx.ALL, 0 )
		
		
		bSizerFrameMain.Add( bSizerMainFrame, 1, wx.ALL|wx.EXPAND, 0 )
		
		
		self.SetSizer( bSizerFrameMain )
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
	
class MainApp(wx.App):
    def OnInit(self):
        mainFrame = FrameMain(None)
        mainFrame.Show(True)
        return True

if __name__ == '__main__':
    app = MainApp()
    app.MainLoop()