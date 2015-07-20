# -*- coding: utf-8 -*-


# Add root directory to path to enable imports ... Python sucks.
import sys
sys.path.append('../../')

from math import radians
import argparse
import rospy
import numpy as np
from subprocess import call
from time import sleep

import message_filters

from std_msgs.msg import String
from std_msgs.msg import Bool

from geometry_msgs.msg import Twist

from soccer.messages.BallDetectionMessage import BallDetectionMessage

SEARCH = 1
LOST_BALL = 2
DRIVE = 10
KICK = 20


class Dev(object):
    
    def __init__(self,name="NodeStrategy"):
        rospy.init_node(name, anonymous=False)

        # Publisher to NodeGoalDetection
        self.pubGoalSearch = rospy.Publisher("/soccer/goaldetection/run", Bool, queue_size=1)
        
        
        
        
        sleep(1)
        self.pubGoalSearch.publish(True)
        
        sleep(1)

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
            print "New State = " + self.string(state)
            self.state = state

    def string(self,state):
        if state == 1:
            return "SEARCH"
        if state == 2:
            return "LOST BALL"
        if state == 10:
            return "DRIVE"
        if state == 20:
            return "KICK"

if __name__ == '__main__':

    print "Dev"
    Dev()
