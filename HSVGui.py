from Tkinter import *
from threading import Thread
import numpy as np
import json

class InputWindow():
    def __init__(self, parent):
        top = self.top = Toplevel(parent)
        top.title = "Tupel benennen"
        self.myLabel = Label(top, text = "Namen fuer Tupel eingeben")
        self.myLabel.pack()
        self.myEntryBox = Entry(top)
        self.myEntryBox.focus_set()
        self.myEntryBox.pack()
        self.mySubmitButton = Button(top, text='OK', command = self.callbackOkButton)
        self.mySubmitButton.pack()
    
    def callbackOkButton(self):
        self.callback(self.myEntryBox.get())
        self.top.destroy()
    
    def setValueCallback(self, a_func):
        self.callback = a_func

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

        self.scales = [];
        # {"list": [], "last": {"max": [8, 255, 255], "min": [4, 133, 91]}}

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
        self.scales.append(scale)
        return scale

    def callbackInputWindowValue(self, astr = ''):
        self.listData["list"].append({'name' : astr, 'min' : self.minimum.tolist(), 'max' : self.maximum.tolist()})
        self.listbox.insert(END, astr)

    def callbackSaveButton(self):
        inputWindow = InputWindow(None)
        inputWindow.setValueCallback(self.callbackInputWindowValue)
        print("Save")

    def callbackListboxOnSelect(self, evt):
        w = evt.widget
        index = int(w.curselection()[0])
        self.scales[0].set(self.listData['list'][index]['min'][0])
        self.scales[1].set(self.listData['list'][index]['min'][1])
        self.scales[2].set(self.listData['list'][index]['min'][2])
        
        self.scales[3].set(self.listData['list'][index]['max'][0])
        self.scales[4].set(self.listData['list'][index]['max'][1])
        self.scales[5].set(self.listData['list'][index]['max'][2])

    def callbackListboxDelete(self, evt):
        w = evt.widget
        index = int(w.curselection()[0])

        self.listData['list'].remove(self.listData['list'][index])
        self.listbox.delete(index, index)
        #print(self.listData['list'][index])

    def initList(self):
        scrollbar = Scrollbar(self.root)
        scrollbar.pack(side=RIGHT, fill=Y)
        
        self.listbox = Listbox(self.root, yscrollcommand=scrollbar.set)
        self.listbox.pack()
        
        scrollbar.config(command=self.listbox.yview)
        
        # callbacks on list
        self.listbox.bind('<<ListboxSelect>>', self.callbackListboxOnSelect)
        self.listbox.bind('<Delete>', self.callbackListboxDelete)
        
        self.btnSave = Button(self.root, text = "Speichern", command = self.callbackSaveButton )
        self.btnSave.pack()

    def loadList(self):
        with open('HSVGui.json', 'r') as infile:
            self.listData = json.load(infile)
        infile.close()
        #print("HSVGui.loadList", self.listData)

        self.scales[0].set(self.listData["last"]["min"][0])
        self.scales[1].set(self.listData["last"]["min"][1])
        self.scales[2].set(self.listData["last"]["min"][2])

        self.scales[3].set(self.listData["last"]["max"][0])
        self.scales[4].set(self.listData["last"]["max"][1])
        self.scales[5].set(self.listData["last"]["max"][2])
        
        for tupel in self.listData['list']:
            self.listbox.insert(END, tupel['name'])

    def saveList(self):
        print("HSVGui.saveList")
        self.listData["last"]["min"] = self.minimum.tolist()
        self.listData["last"]["max"] = self.maximum.tolist()
        
        with open('HSVGui.json', 'w') as outfile:
            json.dump(self.listData, outfile)
        outfile.close();

    # Blocking mainloop
    def mainloop(self):
        self.root.mainloop()
