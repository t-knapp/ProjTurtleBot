# Add root directory to path to enable imports ... Python sucks.
import sys
sys.path.append('../../')

from math import radians
import argparse
import rospy
import numpy as np
from subprocess import call


import message_filters

from std_msgs.msg import String
from std_msgs.msg import Bool

from geometry_msgs.msg import Twist

from soccer.messages.BallDetectionMessage import BallDetectionMessage

SEARCH = 1
LOST_BALL = 2
DRIVE = 10
KICK = 20


class NodeStrategy(object):
    
    def __init__(self,name="NodeStrategy"):
        rospy.init_node(name, anonymous=False)

        # Subscribe to SearchEvents
        rospy.Subscriber("/soccer/ballsearch/found", Bool, self.foundBallCallback, queue_size=1)
        rospy.Subscriber("/soccer/lostBall", String, self.lostBallCallback, queue_size=1)
        rospy.Subscriber("/soccer/balljourney/finished", String, self.journeyFinished, queue_size=1)


        # Publisher to movement
        self.journey = rospy.Publisher("/soccer/balljourney/run", Bool, queue_size=1)


        self.state = LOST_BALL
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.state == LOST_BALL:
                self.setState(SEARCH)
            r.sleep()
# ToDo: Start Ballsearch Node


################################################################################
#Callbacks

    def foundBallCallback(self,data):
        self.setState(DRIVE) # Found Ball Start Driving
        self.journey.publish(True) # start balljourney

    def lostBallCallback(self,data):
        self.setState(LOST_BALL)
        self.journey.publish(False)

    def journeyFinished(self,data):
        self.setState(KICK)


################################################################################
# debug stuff

def setState(self, state):
    if(state != self.state):
        print "New State = " + sting(state)
        self.state = state

def string(self,state)
    if state == 1
        return "SEARCH"
    if state == 2
        return "LOST BALL"
    if state == 10
        return "DRIVE"
    if state == 20
        return "KICK"

if __name__ == '__main__':

    NodeStrategy()