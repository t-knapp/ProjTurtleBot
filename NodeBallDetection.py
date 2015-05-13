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
    rospy.init_node(name, anonymous=False)
    rospy.loginfo("Stop detection by pressing CTRL + C")

    self.colorFrom  = np.array([0, 150, 150], np.uint8)
    self.colorTo    = np.array([25 ,255,255], np.uint8)
    self.detectBlob = DetectBlob(self.colorFrom, self.colorTo)
    
    self.cv_bridge = CvBridge()

    cv2.namedWindow("image_view", 1)
    cv2.,()

    rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.processImages, queue_size=1)
    rospy.Subscriber("/mobile_base/events/button", ButtonEvent, self.buttonListener)

  def buttonListener(self, data):
      print("buttonListener")

  def processImages(self, ros_img):
    #print("processImages")
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


if __name__ == '__main__':
    c = NodeBallDetection();
    rospy.spin()
