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
COLLISION = 40

'''
SpielstrategieNode
>>>>>>> master

Speichert aktuellen Status, startet andere Nodes

ALS LETZTE NODE STARTEN, WENN ALLE ANDEREN LAUFEN!
'''
class NodeStrategy(object):
    
    def __init__(self,name="NodeStrategy"):
        rospy.init_node(name, anonymous=False)

        # Subscribe to SearchEvents
        rospy.Subscriber("/soccer/ballsearch/found", Bool, self.foundBallCallback, queue_size=1)
        rospy.Subscriber("/soccer/lostBall", String, self.lostBallCallback, queue_size=1)
        rospy.Subscriber("/soccer/balljourney/finished", Bool, self.journeyFinished, queue_size=1)
        rospy.Subscriber("/soccer/kick/finished", Bool, self.kickFinishedCallback, queue_size=1)
        rospy.Subscriber("/soccer/collision", Bool, self.collisionCallback, queue_size=1)

        # Publisher to BodeBallJourney
        self.journey = rospy.Publisher("/soccer/balljourney/run", Bool, queue_size=1)
        
        # Publisher to NodeGoalDetection
        self.pubGoalDetection = rospy.Publisher("/soccer/goaldetection/run", Bool, queue_size=1)
        
        # Publisher to NodeKick
        self.pubKick = rospy.Publisher("/soccer/kick/run", Bool, queue_size=1)
        


        self.state = LOST_BALL
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            
            if self.state == LOST_BALL:
                self.setState(SEARCH)
                # Blocking Call to NodeBallSearch
                call(["NodeBallSearch", "--num", "5"])
                
                #            elif self.state == DRIVE:
                # Blocking Call to NodeBallJourney
                #call(["NodeBallJourney"])
            
            
            print "Strategy::Loop"    
            r.sleep()


################################################################################
#Callbacks

    def foundBallCallback(self,data):
        self.setState(DRIVE) # Found Ball Start Driving
        self.journey.publish(True) # start balljourney

    def lostBallCallback(self,data):
        self.setState(LOST_BALL)
        self.journey.publish(False)

    def journeyFinished(self, data):
        print "Journey finished"
        self.journey.publish(False)
        self.pubKick.publish(True)
        self.pubGoalDetection.publish(True)
        self.setState(KICK)
        
    def kickFinishedCallback(self, data):
        self.pubGoalDetection.publish(False)
        self.pubKick.publish(False)
        self.setState(LOST_BALL)
        
    def collisionCallback(self, msg):
        if(msg.data):
            self.pubGoalDetection.publish(False)
            self.pubKick.publish(False)
            self.journey.publish(False)
            self.setState(COLLISION)
        else:
            self.setState(LOST_BALL)

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
        if state == 40:
            return "COLLISION"
        return str(state)

if __name__ == '__main__':

    NodeStrategy()
