# Add root directory to path to enable imports ... Python sucks.
import sys
sys.path.append('../../')

import math
import argparse
from threading import Thread
import rospy
import numpy as np
import cv2
import array

import message_filters

from cv_bridge import CvBridge
from kobuki_msgs.msg import ButtonEvent
from sensor_msgs.msg import Image
from std_msgs.msg import String

from soccer.NodeBallDetection.detectBlob import DetectBlob
from soccer.NodeBallDetection.HSVGui import HSVGui
from soccer.messages.GoalDetectionMessage import GoalDetectionMessage


class NodeGoalDetection(object):
  
  def __init__(self, nthframe=1, normalize=False, name="NodeGoalDetection"):
    rospy.init_node(name, anonymous=False)
    rospy.loginfo("Stop goal detection by pressing CTRL + C")
    rospy.loginfo(name + " using normalization: " + str(normalize))
    rospy.loginfo(name + " using every n-th frame " + str(nthframe))

    self.name = name
    self.detectBlob = DetectBlob(name, (1278, 790))
    
    self.cv_bridge = CvBridge()

    # OpenCV windows
    self.cv_image = self.name + " :: image"
    cv2.namedWindow(self.cv_image, 1)
    cv2.moveWindow(self.cv_image, 1278, 490)
    
    cv2.startWindowThread()
    
    # Variables for CV Show Checkboxes
    self.cvWindows = dict()

    rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.processImages, queue_size=1)
    rospy.Subscriber("/camera/depth/image_raw", Image, self.processDepthImage, queue_size=1)

    # Publisher for BallDetection
    self.msgGoal = rospy.Publisher("/soccer/goalPosition", String, queue_size=1)

    # proceed every n-th frame from cmdline
    self.nthframe = nthframe
    self.counter = 1
    self.depthCounter = 1 
    
    self.normalize = normalize
    
    # Store messages and calculate mean
    self.msgArray = []
    self.msgCounter = 1
    self.msgMaxCount = 5
    
    # How many pixels are cropped in y axis from 0 (top)
    self.imageCrop = 215

  def processDepthImage(self, data):
    # only n-th image
    if(self.nthframe !=0 and (self.depthCounter % self.nthframe != 0)):
        self.depthCounter = self.depthCounter + 1
        return
    
    #print data.width, data.height
    
    # Draw image 'as it is'
    depthFull = self.cv_bridge.imgmsg_to_cv2(data, "passthrough")
    depth = depthFull[self.imageCrop:480, 0:640]
    #depth = self.cv_bridge.imgmsg_to_cv2(data, "32FC1")
    #depth_array = np.array(depth, dtype=np.float32)

    #cv2.imshow("depth", depth)
    
    self.depthCounter = 1

    #height, width = depth.shape[:2]
    #print (width, height)

    self.depthImage = depth

  def processImages(self, ros_img):
    # proceed only n-th image
    if(self.nthframe != 0):
        if(self.counter % self.nthframe != 0):
            self.counter = self.counter + 1
            return
    
    fullimg = self.cv_bridge.imgmsg_to_cv2(ros_img, "bgr8")
    img = fullimg[self.imageCrop:480, 0:640] # y oben start: y unten ende, x links start: x rechts ende
    
    # ToDo: Crop image with cv 'region of interest'
    keypoints = self.detectBlob.getBlobs(img)
   
    # Calculate center    
    centerX = 0;
    centerY = 0;
    centerR = 0;
    
    for item in keypoints :
    #if len(keypoints) == 1:
        #item = keypoints[0]
        # Coordinates and radius
        x = int(item.pt[0])
        y = int(item.pt[1])
        r = int(item.size)
        
        centerX = centerX + x
        centerY = centerY + y
        centerR = centerR + r
        

    if len(keypoints) > 0:
        centerX = centerX / len(keypoints)
        centerY = centerY / len(keypoints)
        centerR = centerR / len(keypoints)
        
        try:
            #depth = np.uint16( self.depthImage[int(item.pt[0]),int(item.pt[1])] )
            #depth in mm
            #TODO: Normalize: Average values in detected circle
            
            y_range = range(y - r/2, y + r/2)
            x_range = range(x - r/2, x + r/2)
            depthArray = self.depthImage[np.ix_(y_range,x_range)]
            depthSum = 0
            for i in range(len(depthArray)):
                for j in range(len(depthArray[i])):
                    depthSum = depthSum + depthArray[i][j]
            normalizedDepth = depthSum / (len(depthArray) * len(depthArray[0]))
            
            #
            # ATTENTION!
            # OpenCV uses height in 1st index pos and width on 2nd
            #
            
            #height, width = self.depthImage.shape[:2]
            #print width, height, x, y
            depth = normalizedDepth if self.normalize else self.depthImage[y, x]
        except IndexError:
            # Ignore
            print "Error"
            pass
                
        
        # 0,0 in OpenCV is left upper corner
        # Scale values in message between 0 - 100
        
        self.msgArray.append((centerX, depth))
        
        cv2.circle(img, (centerX,centerY), centerR, (0,0,255), 2)

    if self.msgCounter == self.msgMaxCount:
        # Publish GoalDetectionMessage
        msgGoalDetection = self.calculateMsg()
        self.msgGoal.publish(String(msgGoalDetection.toJSONString()))
        self.msgArray = []
        self.msgCounter = 1

    # Display the resulting frame
    if self.cvWindows[self.cv_image]:
        cv2.imshow(self.cv_image, img)
    
    self.msgCounter = self.msgCounter + 1
    self.counter = 1

  def calculateMsg(self):
    meanX = 0;
    meanDistance = 0;
    meanCnt = 0
    for msg in self.msgArray:
        if msg[0] != 0:
            meanX = meanX + msg[0]
            meanDistance = meanDistance + msg[1]
            meanCnt = meanCnt + 1
    
    msgGoalDetection = GoalDetectionMessage()
    
    if meanCnt != 0:
        centerX = meanX/meanCnt
        meanDistance = meanDistance/meanCnt
            
        if centerX < 640 / 3:
            msgGoalDetection.state = GoalDetectionMessage.STATELEFT
        elif centerX > 640 / 3 and centerX < (640 / 3) * 2:
            msgGoalDetection.state = GoalDetectionMessage.STATESTRAIGHT
        else:
            msgGoalDetection.state = GoalDetectionMessage.STATERIGHT
        
        msgGoalDetection.distance = meanDistance
        #msgGoalDetection.distance = depth.astype(int)
    return msgGoalDetection


  def cvCheckBoxCallback(self, varName, varValue):
      if self.cvWindows[varName]:
        self.cvWindows[varName] = varValue
        cv2.destroyWindow(varName)
      else:
        cv2.namedWindow(varName, 1)
        self.cvWindows[varName] = varValue
        if varName == self.cv_image:
            cv2.moveWindow(self.cv_image, 1278, 490)
        


def guiThread(colorCallback, filterShapeCallback, filterBlurCallback, ngdObject):
    # Create GUI
    gui = HSVGui(colorCallback, filterShapeCallback, filterBlurCallback, json='goals.json', title='NodeGoalDetection', position=(1800,0));

    # Group min
    groupMin = gui.createLabelFrame("HSV Min-Value", 0, 0);
    gui.createScale(groupMin, "H", gui.fromHvar, 0, 180)
    gui.createScale(groupMin, "S", gui.fromSvar, 0, 255)
    gui.createScale(groupMin, "V", gui.fromVvar, 0, 255)

    # Group max
    groupMax = gui.createLabelFrame("HSV Max-Value", 1, 0);
    gui.createScale(groupMax, "H", gui.toHvar, 0, 180)
    gui.createScale(groupMax, "S", gui.toSvar, 0, 255)
    gui.createScale(groupMax, "V", gui.toVvar, 0, 255)

    # Filter  
    groupShapeFilters = gui.createScrollableLabelFrame("Shape-Filters", 0, 1)
    gui.createShapeFilterOption(groupShapeFilters, "Activate Circularity", gui.cbCircularityVar, "Min.", gui.scCircularityMinVar, 0, 1, "Max.", gui.scCircularityMaxVar, 0, 1)
    gui.createShapeFilterOption(groupShapeFilters, "Activate Inertia", gui.cbInertiaVar, "Min.", gui.scInertiaMinVar, 0, 1, "Max.", gui.scInertiaMaxVar, 0, 1)
    gui.createShapeFilterOption(groupShapeFilters, "Activate Convexity", gui.cbConvexityVar, "Min.", gui.scConvexityMinVar, 0, 1, "Max.", gui.scConvexityMaxVar, 0, 1)
        
    groupBlur = gui.createLabelFrame("Blur-Filter", 1, 1);    
    gui.createSingleRadioBtn(groupBlur, "None", gui.rbBlurVar, 1)
    gui.createSingleRadioBtn(groupBlur, "Activate GaussianBlur", gui.rbBlurVar, 2)  
    gui.createSingleRadioBtn(groupBlur, "Activate MedianBlur", gui.rbBlurVar, 3)  

    # List
    gui.initList("Speichern/Laden", 2, 0)
    gui.loadList()

    # OpenCV windows checkboxes
    cbGroup = gui.createLabelFrame("OpenCV windows", 2,1)
    gui.createCVCheckbox(cbGroup, ngdObject.cv_image, ngdObject)
    gui.createCVCheckbox(cbGroup, ngdObject.detectBlob.cv_range, ngdObject.detectBlob)


    # Start mainloop (blocking!)
    gui.mainloop()

    # Write list as JSON
    gui.saveList()

    # Close all opencv windows
    cv2.destroyAllWindows()
    
    # Quit Node
    rospy.signal_shutdown("Correct end.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--normalize', help='Normalize depth values using points in detected ball\'s area (square based)', default=False, action='store_true')
    parser.add_argument('--nthframe', help='Use only n-th frame for detection', default=1, nargs='?', type=int)
    args = parser.parse_args()

    ngd = NodeGoalDetection(args.nthframe, args.normalize);
    
    # GUI in separate thread
    thread = Thread(target = guiThread, args = (ngd.detectBlob.setColors, ngd.detectBlob.setFilterShape, ngd.detectBlob.setFilterBlur, ngd))
    thread.start()
    
    rospy.spin()
