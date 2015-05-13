import rospy
import numpy as np
import cv2
import message_filters
from detectBlob import DetectBlob
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math

from kobuki_msgs.msg import ButtonEvent


class NodeBallDetection(object):
  def __init__(self, name="NodeBallDetection"):
    self.name = name
    rospy.init_node(name, anonymous=False)
        rospy.loginfo("Starting node %s" % name)
        self.bridge = CvBridge()            # Creatingan OpenCV bridge object used to create an OpenCV image from the ROS image
        cv2.namedWindow("Image window", 1)  # Opening a window to show the image
        cv2.startWindowThread()
        
        self.image_sub = rospy.Subscriber(  # Creating a subscriber listening to the kinect image topic
                                          "/camera/rgb/image_color",      # The topic to which it should listened
                                          Image,                          # The data type of the topic
                                          callback=self.image_callback,   # The callback function that is triggered when a new message arrives
                                          queue_size=1                    # Disregard every message but the latest
                                          )

    self.colorFrom  = np.array([0, 150, 150], np.uint8)
    self.colorTo    = np.array([25 ,255,255], np.uint8)
    self.detectBlob = DetectBlob(self.colorFrom, self.colorTo)
    
    self.cv_bridge = CvBridge()

    cv2.namedWindow("image_view", 1)

    rospy.Subscriber("/camera/rgb/image_color", Image, self.processImages)
    rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.buttonListener)

  def buttonListener(self, data):
      print("buttonListener")

  def processImages(self, ros_img):
    print("processImages")
    img = self.cv_bridge.imgmsg_to_cv2(ros_img, "bgr8")
    keypoints = self.detectBlob.getBlobs(img)
    for item in keypoints :
        print("\tx %d , y %d, d: %d" % (item.pt[0],item.pt[1], item.size))
        cv2.circle(img, (int(item.pt[0]),int(item.pt[1])), int(item.size), (0,255,100),5)
    print("")

    # Display the resulting frame
    #cv2.imshow('frame',img)
    cv2.imshow("image_view", img)
    cv2.imwrite('ball.png',img)


if __name__ == '__main__':
    c = NodeBallDetection();
    rospy.spin()
