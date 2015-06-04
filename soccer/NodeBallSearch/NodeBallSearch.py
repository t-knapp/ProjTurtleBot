# Add root directory to path to enable imports ... Python sucks.
import sys
sys.path.append('../../')

from math import radians
import argparse
import rospy
import numpy as np

import message_filters

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from soccer.messages.BallDetectionMessage import BallDetectionMessage


class NodeBallDetection(object):
  
    def __init__(self, numFoundsInSequence=1, name="NodeBallSearch"):
        rospy.init_node(name, anonymous=False)
        rospy.loginfo("Stop ball search by pressing CTRL + C")

        # Subscribe to NodeBallDetection
        rospy.Subscriber("/soccer/balldetection/ballPosition", String, self.callbackBallPosition, queue_size=1)

        # Publisher to movement
        self.move = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        r = rospy.Rate(5)
        # Turn
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(45); #45 deg/s in radians/s
        
        self.numMsgsInSequenceFoundBall = 0
        self.lastMsgFoundBall = False
        self.numFoundsInSequence = numFoundsInSequence
        
        self.run = True

        while not rospy.is_shutdown() and self.run:
            
	        # Spin for 2 seconds (10 x 5 HZ)
            #          1 second  ( 5 x 5 Hz)
            for x in range(0,3):
                self.move.publish(turn_cmd)
                r.sleep()


    def callbackBallPosition(self, strData):
        
        #TODO: Assume the ball is found if N sequenced messages found
        #      the ball.
        #print "callbackBallPosition"
        #print strData.data
        msg = BallDetectionMessage.fromJSONString(strData.data)
        
        if msg.ballDetected and self.lastMsgFoundBall:
            self.numMsgsInSequenceFoundBall = self.numMsgsInSequenceFoundBall + 1
            if self.numMsgsInSequenceFoundBall >= self.numFoundsInSequence:
                self.run = False
        
        self.lastMsgFoundBall = msg.ballDetected
        
            
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--num', help='Number of found messages in sequence needed to assume ball is found.', default=1, nargs='?', type=int)
    args = parser.parse_args()
    
    try:
        NodeBallDetection(args.num)
    except:
        rospy.loginfo("Node terminated in main-method")
