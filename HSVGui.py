from Tkinter import *
from threading import Thread
import numpy as np

class HSVGui(object):
    def __init__(self, setColorCallback):
        self.setColorCallback = setColorCallback
        
        self.minimum = np.array([0, 0, 0],np.uint8)
        self.maximum = np.array([0, 0, 0],np.uint8)

        self.root = Tk()
        self.root.wm_title("HSV Value-Chooser")

        self.fromHvar = IntVar()
        self.fromSvar = IntVar()
        self.fromVvar = IntVar()

        self.toHvar = IntVar()
        self.toSvar = IntVar()
        self.toVvar = IntVar()

    # Called if scale changes
    def onChange(self, value):
        self.minimum[0] = self.fromHvar.get()
        self.minimum[1] = self.fromSvar.get()
        self.minimum[2] = self.fromVvar.get()
        self.maximum[0] = self.toHvar.get()
        self.maximum[1] = self.toSvar.get()
        self.maximum[2] = self.toVvar.get()
        #print(self.minimum, self.maximum)
        self.setColorCallback(self.minimum, self.maximum)
    
    def createLabelFrame(self, text):
        labelframe = LabelFrame(self.root, text = text)
        labelframe.pack(fill = "both", expand = "yes")
        
        return labelframe
    
    def createScale(self, labelFrame, label, variable, from_, to):
        scale = Scale(
            labelFrame, 
            label = label, 
            variable = variable, 
            from_ = from_, 
            to = to, 
            command = self.onChange, 
            orient = HORIZONTAL, 
            length = 255
        )
        scale.pack(anchor = CENTER)
        
        return scale
    
    # Blocking mainloop
    def mainloop(self):
        self.root.mainloop()
