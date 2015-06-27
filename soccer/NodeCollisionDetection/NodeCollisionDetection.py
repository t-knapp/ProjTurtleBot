# Add root directory to path to enable imports ... Python sucks.
import sys
sys.path.append('../../')

from math import radians
import argparse
import rospy
import numpy as np
import time

import message_filters

from std_msgs.msg import String
from std_msgs.msg import Bool

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent


from soccer.messages.BallDetectionMessage import BallDetectionMessage


class NodeCollisionDetection(object):
    
    LINEAR_SPEED = 0.3
    ANGULAR_SPEED = 0.66
    
    def __init__(self, name="NodeCollisionDetection"):
        rospy.init_node(name, anonymous=False)
        
        # Subscribe to NodeBallDetection
        rospy.Subscriber("/soccer/movement", Twist, self.callbackMovement, queue_size=1)
        # Subscribe to Bumper Events
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.collisionHandler)
        # Subscribe to Referee Events
        rospy.Subscriber('soccer/referee', Bool, self.refereeHandler)
        # Publisher to movement
        self.move = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        
        self.noCollision = True
        self.run = True
        
        r = rospy.Rate(5)
        while not rospy.is_shutdown() and self.run:
            r.sleep()


    def callbackMovement(self, data):
        if self.noCollision and self.run:
            self.move.publish(data)

    def refereeHandler(self,data):
        print data
        self.run = data.data

    def collisionHandler(self, msg):
        if msg.state != BumperEvent.PRESSED:
            self.noCollision = False
            t = Twist()

            t.linear.x = -self.LINEAR_SPEED
            if msg.bumper == 0: # LEFT Bumper
                t.angular.z = self.ANGULAR_SPEED
            if msg.bumper == 2: # RIGHT Bumper
                t.angular.z = -self.ANGULAR_SPEED
            
            for x in range(0,5):
                self.move.publish(t)
                time.sleep(0.01)

        self.noCollision = True



if __name__ == '__main__':
    NodeCollisionDetection()
