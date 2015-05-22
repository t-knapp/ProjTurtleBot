#import rospy
import numpy as np
import cv2
#import message_filters
from detectBlob import DetectBlob
#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#import math

#from kobuki_msgs.msg import ButtonEvent

from HSVGui import HSVGui
from threading import Thread

class NodeBallDetection(object):
  def __init__(self, name="NodeBallDetection"):
    #rospy.init_node(name, anonymous=False)
    #rospy.loginfo("Stop detection by pressing CTRL + C")

    self.detectBlob = DetectBlob()
    
    #self.cv_bridge = CvBridge()

    cv2.namedWindow("image_view", 1)
    cv2.startWindowThread()

    #rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.processImages, queue_size=1)
    #rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.buttonListener)

  def buttonListener(self, data):
      print("buttonListener")

  def processImages(self, ros_img):
    img = self.cv_bridge.imgmsg_to_cv2(ros_img, "bgr8")
    keypoints = self.detectBlob.getBlobs(img)
    for item in keypoints :
    #    print("\tx %d , y %d, d: %d" % (item.pt[0],item.pt[1], item.size))
        cv2.circle(img, (int(item.pt[0]),int(item.pt[1])), int(item.size), (0,255,100),5)
    #print("")

    # Display the resulting frame
    #cv2.imshow('frame',img)
    cv2.imshow("image_view", img)
    #cv2.imwrite('ball.png',img)

def guiThread(colorCallback):
    # Create GUI
    gui = HSVGui(colorCallback);

    # Group min
    groupMin = gui.createLabelFrame("HSV Min-Value");
    gui.createScale(groupMin, "H", gui.fromHvar, 0, 180)
    gui.createScale(groupMin, "S", gui.fromSvar, 0, 255)
    gui.createScale(groupMin, "V", gui.fromVvar, 0, 255)

    # Group max
    groupMax = gui.createLabelFrame("HSV Max-Value");
    gui.createScale(groupMax, "H", gui.toHvar, 0, 180)
    gui.createScale(groupMax, "S", gui.toSvar, 0, 255)
    gui.createScale(groupMax, "V", gui.toVvar, 0, 255)

    # List
    gui.initList()
    gui.loadList()

    # Start mainloop (blocking!)
    gui.mainloop()

    # Write list as JSON
    gui.saveList()

if __name__ == '__main__':
    nbd = NodeBallDetection();
    
    # GUI in separate thread
    thread = Thread(target = guiThread, args = (nbd.detectBlob.setColors, ))
    thread.start()
    
    #rospy.spin()
