from Tkinter import *
from io import *
import json
from threading import Thread
import cv2
import numpy as np

ORANGE_MIN = np.array([5, 150, 91],np.uint8)
ORANGE_MAX = np.array([15, 225, 225],np.uint8)

root = Tk()

fromHvar = IntVar()
fromSvar = IntVar()
fromVvar = IntVar()

toHvar = IntVar()
toSvar = IntVar()
toVvar = IntVar()

def onChange(value):
    ORANGE_MIN[0] = fromHvar.get()
    ORANGE_MIN[1] = fromSvar.get()
    ORANGE_MIN[2] = fromVvar.get()
    ORANGE_MAX[0] = toHvar.get()
    ORANGE_MAX[1] = toSvar.get()
    ORANGE_MAX[2] = toVvar.get()


# From HSV
labelframe1 = LabelFrame(root, text="HSV Min Color")
labelframe1.pack(fill="both", expand="yes")

scale1 = Scale( labelframe1, label='H', variable = fromHvar, from_=0, to=180, command=onChange, orient=HORIZONTAL, length=180 )
scale1.pack(anchor=CENTER)

scale2 = Scale( labelframe1, label='S', variable = fromSvar, from_=0, to=255, command=onChange, orient=HORIZONTAL, length=180 )
scale2.pack(anchor=CENTER)

scale3 = Scale( labelframe1, label='V', variable = fromVvar, from_=0, to=255, command=onChange, orient=HORIZONTAL, length=180 )
scale3.pack(anchor=CENTER)

# To HSV
labelframe2 = LabelFrame(root, text="HSV Max Color")
labelframe2.pack(fill="both", expand="yes")

scale4 = Scale( labelframe2, label='H', variable = toHvar, from_=0, to=180, command=onChange, orient=HORIZONTAL, length=180 )
scale4.pack(anchor=CENTER)

scale5 = Scale( labelframe2, label='S', variable = toSvar, from_=0, to=255, command=onChange, orient=HORIZONTAL, length=180 )
scale5.pack(anchor=CENTER)

scale6 = Scale( labelframe2, label='V', variable = toVvar, from_=0, to=255, command=onChange, orient=HORIZONTAL, length=180 )
scale6.pack(anchor=CENTER)


# Read last values from config
#f = open('hsv-adjust-gui.cfg', 'wb+')
#hst_data = json.load(f)
#print(hst_data)


cvThreadRunning = True
def cvThread(arg):
   print("Start thread for opencv")
   #ORANGE_MIN = np.array([5, 150, 91],np.uint8)
   #ORANGE_MAX = np.array([15, 225, 225],np.uint8)

   cap = cv2.VideoCapture(0)

   cv2.namedWindow('image', cv2.WINDOW_NORMAL)
   cv2.startWindowThread()

   while(cvThreadRunning):
      ret, img = cap.read()

      hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

      frame_threshed = cv2.inRange(hsv_img, ORANGE_MIN, ORANGE_MAX)
      #cv2.imwrite('cv-.jpg', frame_threshed)
      cv2.imshow('image', frame_threshed)

   cv2.destroyAllWindows()
  


if __name__ == "__main__":
    thread = Thread(target = cvThread, args = (10, ))
    thread.start()

root.mainloop()
cvThreadRunning = False
thread.join()


#json.dump([ORANGE_MIN.tolist(), ORANGE_MAX.tolist()], f)
#f.close();
