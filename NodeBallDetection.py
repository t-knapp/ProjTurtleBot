import rospy
import numpy as np
import cv2
import message_filters
from detectBlob import DetectBlob
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import sys

from kobuki_msgs.msg import ButtonEvent

from HSVGui import HSVGui
from threading import Thread

from messages.BallDetectionMessage import BallDetectionMessage
from std_msgs.msg import String

class NodeBallDetection(object):
  def __init__(self, name="NodeBallDetection"):
    rospy.init_node(name, anonymous=False)
    rospy.loginfo("Stop detection by pressing CTRL + C")

    self.detectBlob = DetectBlob()
    
    self.cv_bridge = CvBridge()

    cv2.namedWindow("image_view", 1)
    cv2.namedWindow("depth", 1)
    cv2.startWindowThread()

    rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.processImages, queue_size=1)
    rospy.Subscriber("/camera/depth/image_raw", Image, self.processDepthImage, queue_size=1)
    #rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.buttonListener)
    #rospy.Subscriber("/camera/depth/image", Image, self.processDepthImage, queue_size=1)

    # Publisher for BallDetection
    self.msgBall = rospy.Publisher("/soccer/balldetection/ballPosition", String, queue_size=1)

    # proceed every n-th frame from cmdline
    self.nthframe = int(sys.argv[1])
    self.counter = 1
    self.depthCounter = 1

  def buttonListener(self, data):
      print("buttonListener")

  def processDepthImage(self, data):
    # only n-th image
    if(self.nthframe !=0 and (self.depthCounter % self.nthframe != 0)):
        self.depthCounter = self.depthCounter + 1
        return
    
    #print data.width, data.height
    
    # Draw image 'as it is'
    depth = self.cv_bridge.imgmsg_to_cv2(data, "passthrough")
    #depth = self.cv_bridge.imgmsg_to_cv2(data, "32FC1")
    #depth_array = np.array(depth, dtype=np.float32)

    cv2.imshow("depth", depth)
    
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
   
    img = self.cv_bridge.imgmsg_to_cv2(ros_img, "bgr8")
    keypoints = self.detectBlob.getBlobs(img)
   
    msgBallDetection = BallDetectionMessage()
    msgBallDetection.x = 0
    msgBallDetection.y = 0
    msgBallDetection.distance = 0
    msgBallDetection.ballDetected = False
 
    #TODO: Find best one?
    #if (keypoints.size == 1):
    for item in keypoints :
        #item = keypoints[0]
        print "keypoint"
        
        # Coordinates and radius
        x = int(item.pt[0])
        y = int(item.pt[1])
        r = int(item.size)
        
        # Draw circle around ball
        cv2.circle(img, (x,y), r, (0,255,100), 3)

        try:
            #depth = np.uint16( self.depthImage[int(item.pt[0]),int(item.pt[1])] )
            #depth in mm
            #TODO: Normalize: Average values in detected circle
            
            #
            # ATTENTION!
            # OpenCV uses height in 1st index pos and width on 2nd
            #
            
            #height, width = self.depthImage.shape[:2]
            #print width, height, x, y
            depth = self.depthImage[y, x]
            
            # 0,0 in OpenCV is left upper corner
            # Scale values in message between 0 - 100
            msgBallDetection.x = int((float(x)/ros_img.width) * 100)
            msgBallDetection.y = abs(int((float(y)/ros_img.height) * 100) - 100)
            msgBallDetection.distance = depth.astype(int)
            msgBallDetection.ballDetected = True
            
        except IndexError:
            # Ignore
            pass

    # Publish BallDetectionMessage
    self.msgBall.publish(String(msgBallDetection.toJSONString()))

    # Display the resulting frame
    cv2.imshow("image_view", img)
    
    self.counter = 1

def guiThread(colorCallback, filterShapeCallback, filterBlurCallback):
    # Create GUI
    gui = HSVGui(colorCallback, filterShapeCallback, filterBlurCallback);

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

    # Start mainloop (blocking!)
    gui.mainloop()

    # Write list as JSON
    gui.saveList()

    # Close all opencv windows
    cv2.destroyAllWindows()
    
    # Quit Node
    rospy.signal_shutdown("Correct end.")

if __name__ == '__main__':
    nbd = NodeBallDetection();
    
    # GUI in separate thread
    thread = Thread(target = guiThread, args = (nbd.detectBlob.setColors, nbd.detectBlob.setFilterShape, nbd.detectBlob.setFilterBlur, ))
    thread.start()
    
    rospy.spin()
