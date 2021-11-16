import wx

class NodeView(wx.Panel):
	def __init__(self, parent, id=wx.ID_ANY, pos=wx.DefaultPosition, size=wx.DefaultSize):
		wx.Panel.__init__(self, parent, id, pos, size)
		
		self.Bind(wx.EVT_PAINT, self.OnPaint)
		self.Bind(wx.EVT_LEFT_DOWN, self.OnMouseClick)
		
	def OnPaint(self, event):
		dc = wx.BufferedPaintDC(self)
		self.Draw(dc)
		
	def Draw(self, dc):
		width, height = self.GetClientSize()
		if not width or not height:
			return
		
		backColour = self.GetBackgroundColour()
		backBrush = wx.Brush(backColour, wx.SOLID)
		dc.SetBackground(backBrush)
		dc.Clear()
		
		dc.DrawRectangle(0, 0, 100, 50)
		testString = "test"
		testStringW, testStringH = dc.GetTextExtent(testString)
		dc.DrawText(testString, 100 / 2 - (testStringW / 2), 50 / 2 - (testStringH / 2))
		
	def OnMouseClick(self, event):
		pass