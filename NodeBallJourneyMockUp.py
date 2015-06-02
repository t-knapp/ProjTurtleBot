import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
import numpy as np

from messages.BallDetectionMessage import BallDetectionMessage
from messages.DirectionMessage import DirectionMessage
from geometry_msgs.msg import Twist


class NodeBallJourneyMockUp(object):

    def __init__(self, name="NodeBallJourneyMockUp"):

        rospy.init_node(name, anonymous=False)
        rospy.loginfo("Stop detection by pressing CTRL + C")

        rospy.Subscriber("/soccer/balldetection/ballPosition", String, self.detectionCallBack, queue_size = 1)

    # CALLBACKS
    def detectionCallBack(self, data):
        #print data
        print BallDetectionMessage.fromJSONString(data.data)

if __name__ == '__main__':
    NodeBallJourneyMockUp()

    rospy.spin() 
