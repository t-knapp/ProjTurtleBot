import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
import numpy as np


class NodeBallJourney(object):
    def __init__(self, name="NodeBallJourney"):
    rospy.init_node(name, anonymous=False)
    rospy.loginfo("Stop detection by pressing CTRL + C")

    rospy.Subscriber("/soccer/balljourney/run", Bool, self.runCallback, queue_size=1)
    rospy.Subscriber("/soccer/balldetection/ballPosition", String, self.positionCallback, queue_size = 1)

    self.run = False
    while(True):
        while(self.run):
            # do stuff



    def runCallback(self, data):
        self.run = data


    def s