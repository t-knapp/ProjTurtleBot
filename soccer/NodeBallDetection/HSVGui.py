from Tkinter import *
from threading import Thread
from FilterOption import *
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
    def __init__(self, setColorCallback, setFilterShapeCallback, setFilterBlurCallback, json='HSVGui.json'):
        self.setColorCallback = setColorCallback
        self.setFilterShapeCallback = setFilterShapeCallback
        self.setFilterBlurCallback = setFilterBlurCallback
        
        # JSON File to store/load data
        self.json = json
        
        self.minimum = np.array([0, 0, 0],np.uint8)
        self.maximum = np.array([0, 0, 0],np.uint8)
        
        ''' Shape-Filter '''
        circularityOpt = FilterOption("circularity")
        inertiaOpt = FilterOption("inertia")
        convexityOpt = FilterOption("convexity")
        self.filterShape = [circularityOpt, inertiaOpt, convexityOpt]
        ''' Blur-Filter '''        
        self.filterBlur = 1       

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
        
        ''' Filter Shape '''
        self.cbCircularityVar = IntVar()
        self.scCircularityMinVar = DoubleVar()
        self.scCircularityMaxVar = DoubleVar()
        
        self.cbInertiaVar = IntVar()
        self.scInertiaMinVar = DoubleVar()
        self.scInertiaMaxVar = DoubleVar()
        
        self.cbConvexityVar = IntVar()
        self.scConvexityMinVar = DoubleVar()
        self.scConvexityMaxVar = DoubleVar()
        
        self.filterShapeScales = []
        self.filterShapeCheckboxes = []
        
        ''' Filter Blur '''        
        self.rbBlurVar = IntVar()        

        
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
    
    def createLabelFrame(self, text, column, row):
        labelframe = LabelFrame(self.root, text = text)
        labelframe.grid(column = column, row = row, sticky = "NESW")
        #labelframe.pack(fill = "both", expand = "yes")
        
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
        self.listData["list"].append({'name' : astr, 'min' : self.minimum.tolist(), 'max' : self.maximum.tolist(), 'filterBlur' : self.filterBlur,
				      'filtersShape' : [
					  {'name' : self.filterShape[0].name, 'activated' : self.filterShape[0].activated, 'min' : self.filterShape[0].minimum, 'max' : self.filterShape[0].maximum},
					  {'name' : self.filterShape[1].name, 'activated' : self.filterShape[1].activated, 'min' : self.filterShape[1].minimum, 'max' : self.filterShape[1].maximum},
					  {'name' : self.filterShape[2].name, 'activated' : self.filterShape[2].activated, 'min' : self.filterShape[2].minimum, 'max' : self.filterShape[2].maximum}
					]})
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
        
        
        ''' Filter Circularity '''
        self.cbCircularityVar.set(self.listData['list'][index]['filtersShape'][0]['activated'])
        self.filterShapeScales[0].set(self.listData['list'][index]['filtersShape'][0]['min'])
        self.filterShapeScales[1].set(self.listData['list'][index]['filtersShape'][0]['max'])
        
        ''' Filter Inertia '''
        self.cbInertiaVar.set(self.listData['list'][index]['filtersShape'][1]['activated'])
        self.filterShapeScales[2].set(self.listData['list'][index]['filtersShape'][1]['min'])
        self.filterShapeScales[3].set(self.listData['list'][index]['filtersShape'][1]['max'])
        
        ''' Filter Convexity '''
        self.cbConvexityVar.set(self.listData['list'][index]['filtersShape'][2]['activated'])
        self.filterShapeScales[4].set(self.listData['list'][index]['filtersShape'][2]['min'])
        self.filterShapeScales[5].set(self.listData['list'][index]['filtersShape'][2]['max'])
        
        ''' Filter Blur '''
        self.rbBlurVar.set(self.listData['list'][index]['filterBlur'])

    def callbackListboxDelete(self, evt):
        w = evt.widget
        index = int(w.curselection()[0])

        self.listData['list'].remove(self.listData['list'][index])
        self.listbox.delete(index, index)
        #print(self.listData['list'][index])

    def initList(self, text, column, row):
	labelFrame = LabelFrame(self.root, text = text)
        scrollbar = Scrollbar(labelFrame)
        scrollbar.pack(side=RIGHT, fill=Y)
        
        self.listbox = Listbox(labelFrame, yscrollcommand=scrollbar.set)
        self.listbox.pack()
        
        scrollbar.config(command=self.listbox.yview)
        
        # callbacks on list
        self.listbox.bind('<<ListboxSelect>>', self.callbackListboxOnSelect)
        self.listbox.bind('<Delete>', self.callbackListboxDelete)
        
        self.btnSave = Button(labelFrame, text = "Speichern", command = self.callbackSaveButton )
        self.btnSave.pack()
        labelFrame.grid(column = column, row = row, sticky = "N")

    def loadList(self):
        with open(self.json, 'r') as infile:
            self.listData = json.load(infile)
        infile.close()
        #print("HSVGui.loadList", self.listData)

        self.scales[0].set(self.listData["last"]["min"][0])
        self.scales[1].set(self.listData["last"]["min"][1])
        self.scales[2].set(self.listData["last"]["min"][2])

        self.scales[3].set(self.listData["last"]["max"][0])
        self.scales[4].set(self.listData["last"]["max"][1])
        self.scales[5].set(self.listData["last"]["max"][2])
        
        
        ''' Filter Circularity '''
        self.cbCircularityVar.set(self.listData['last']['filtersShape'][0]['activated'])
        self.filterShapeScales[0].set(self.listData['last']['filtersShape'][0]['min'])
        self.filterShapeScales[1].set(self.listData['last']['filtersShape'][0]['max'])
        
        ''' Filter Inertia '''
        self.cbInertiaVar.set(self.listData['last']['filtersShape'][1]['activated'])
        self.filterShapeScales[2].set(self.listData['last']['filtersShape'][1]['min'])
        self.filterShapeScales[3].set(self.listData['last']['filtersShape'][1]['max'])
        
        ''' Filter Convexity '''
        self.cbConvexityVar.set(self.listData['last']['filtersShape'][2]['activated'])
        self.filterShapeScales[4].set(self.listData['last']['filtersShape'][2]['min'])
        self.filterShapeScales[5].set(self.listData['last']['filtersShape'][2]['max'])
        
        ''' Filter Blur '''
        self.rbBlurVar.set(self.listData['last']['filterBlur'])
        
        for tupel in self.listData['list']:
            self.listbox.insert(END, tupel['name'])

    def saveList(self):
        print("HSVGui.saveList")
        self.listData["last"]["min"] = self.minimum.tolist()
        self.listData["last"]["max"] = self.maximum.tolist()
        
        ''' Filter Circularity '''
        self.listData['last']['filtersShape'][0]['name'] = self.filterShape[0].name
        self.listData['last']['filtersShape'][0]['activated'] = self.filterShape[0].activated
        self.listData['last']['filtersShape'][0]['min'] = self.filterShape[0].minimum
        self.listData['last']['filtersShape'][0]['max'] = self.filterShape[0].maximum
        
        ''' Filter Inertia '''
        self.listData['last']['filtersShape'][1]['name'] = self.filterShape[1].name
        self.listData['last']['filtersShape'][1]['activated'] = self.filterShape[1].activated
        self.listData['last']['filtersShape'][1]['min'] = self.filterShape[1].minimum
        self.listData['last']['filtersShape'][1]['max'] = self.filterShape[1].maximum
        
        ''' Filter Convexity '''
        self.listData['last']['filtersShape'][2]['name'] = self.filterShape[2].name
        self.listData['last']['filtersShape'][2]['activated'] = self.filterShape[2].activated
        self.listData['last']['filtersShape'][2]['min'] = self.filterShape[2].minimum
        self.listData['last']['filtersShape'][2]['max'] = self.filterShape[2].maximum
        
        ''' Filter Blur '''
        self.listData["last"]["filterBlur"] = self.filterBlur
        
        with open(self.json, 'w') as outfile:
            json.dump(self.listData, outfile, sort_keys=True, indent=4, separators=(',', ': '))
        outfile.close();

    # Blocking mainloop
    def mainloop(self):
        self.root.mainloop()

    
   
    def createScrollableLabelFrame(self, text, column, row):
	labelframe = LabelFrame(self.root, text = text, width = 50, height = 100, bd = 1)
	
	self.canvas = Canvas(labelframe, highlightthickness=0)
	frame = Frame(self.canvas)
	scrollbar = Scrollbar(labelframe, orient = "vertical", command = self.canvas.yview)
	self.canvas.configure(yscrollcommand = scrollbar.set)
	
	scrollbar.pack(side = "right", fill = "y")
	self.canvas.pack(side = "left")
	self.canvas.create_window((5,5), window = frame, anchor="nw")
	frame.bind("<Configure>", self.myfunction)
	
	labelframe.grid(column = column, row = row, sticky = "NESW")
	
	return frame	
    
    def myfunction(self, event):
      self.canvas.configure(scrollregion=self.canvas.bbox("all"),width=240,height=200)

    def createShapeFilterOption(self, labelFrame, cbText, cbVar, scMinText, scMinVar, scMinFrom, scMinTo, scMaxText, scMaxVar, scMaxFrom, scMaxTo):
	''' Panels '''
	pMain = PanedWindow(labelFrame)
	pSub = PanedWindow(pMain)
	
	''' Elements '''
	cbFilter = Checkbutton(pMain, text = cbText, variable = cbVar, command = lambda: self.onChangeCbShapeFilter(cbVar, pSub))	
	scMin = Scale(
	  pSub, 
	  label = scMinText,
	  variable = scMinVar,
	  from_ = scMinFrom,
	  to = scMinTo,
	  command = self.onChangeScShapeFilter,
	  orient = HORIZONTAL,
	  resolution = 0.01,
	  length = 225
	)		
	scMax = Scale(
	  pSub,
	  label = scMaxText,
	  variable = scMaxVar,
	  from_ = scMaxFrom,
	  to = scMaxTo,
	  command = self.onChangeScShapeFilter,
	  orient = HORIZONTAL,
	  resolution = 0.01,
	  length = 225
	)
	
	scMin.grid()
	scMax.grid()	
		
	cbFilter.grid(sticky = W)
	pSub.grid()
	#pSub.grid_remove()
	
	pMain.grid(sticky = W)
	
	self.filterShapeScales.append(scMin)
	self.filterShapeScales.append(scMax)
	self.filterShapeCheckboxes.append(cbFilter)
   
    def createSingleRadioBtn(self, labelFrame, rbText, rbVar, value):
	radio = Radiobutton(labelFrame, text = rbText, variable = rbVar, value = value, command = self.onChangeFilterBlur)
	radio.grid(sticky = W)
   
    def createBlurFilterOption(self, labelFrame, rbText, rbVar, value, scSigXText, scSigXVar, scSigXFrom, scSigXTo, scSigYText, scSigYVar, scSigYFrom, scSigYTo, scKSizeText, scKSizeVar, scKSizeFrom, scKSizeTo):
	''' Panels '''
	pMain = PanedWindow(labelFrame)
	pSub = PanedWindow(pMain)
	
	''' Elements '''
	radio = Radiobutton(labelFrame, text = rbText, variable = rbVar, value = value)
	scSigX = Scale(
	  pSub, 
	  label = scSigXText,
	  variable = scSigXVar,
	  from_ = scSigXFrom,
	  to = scSigXTo,
	  command = None,
	  orient = HORIZONTAL,
	  resolution = 1,
	  length = 225
	)
	scSigY = Scale(
	  pSub,
	  label = scSigYText,
	  variable = scSigYVar,
	  from_ = scSigYFrom,
	  to = scSigYTo,
	  command = None,
	  orient = HORIZONTAL,
	  resolution = 1,
	  length = 225
	)
	scKSize = Scale(
	  pSub,
	  label = scKSizeText,
	  variable = scKSizeVar,
	  from_ = scKSizeFrom,
	  to = scKSizeTo,
	  command = None,
	  orient = HORIZONTAL,
	  resolution = 1,
	  length = 225
	)
	
	scSigX.grid()
	scSigY.grid()
	scKSize.grid()
	
	radio.grid(sticky = W)
	pSub.grid()
	#pSub.grid_remove()
	
	pMain.grid(sticky = W)
   
    def updateShapeFilter(self):
	self.filterShape[0].activated = self.cbCircularityVar.get()
	self.filterShape[0].minimum = self.scCircularityMinVar.get()
	self.filterShape[0].maximum = self.scCircularityMaxVar.get()
	
	self.filterShape[1].activated = self.cbInertiaVar.get()
	self.filterShape[1].minimum = self.scInertiaMinVar.get()
	self.filterShape[1].maximum = self.scInertiaMaxVar.get()
	
	self.filterShape[2].activated = self.cbConvexityVar.get()
	self.filterShape[2].minimum = self.scConvexityMinVar.get()
	self.filterShape[2].maximum = self.scConvexityMaxVar.get()
	
	self.setFilterShapeCallback(self.filterShape)
		  
    def onChangeScShapeFilter(self, event):
	self.updateShapeFilter()
	
    def onChangeCbShapeFilter(self, cbVar, panel):
	''' expand panel '''
	'''if(cbVar.get() == 0):
	  panel.grid_remove()	  
	else:
	  panel.grid()	'''
	  
	self.updateShapeFilter()
	
    def onChangeFilterBlur(self):
	self.filterBlur = self.rbBlurVar.get()
	
	self.setFilterBlurCallback(self.filterBlur)
	
	
	
	
	

    
