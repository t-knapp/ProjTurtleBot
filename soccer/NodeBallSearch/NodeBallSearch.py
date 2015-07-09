# Add root directory to path to enable imports ... Python sucks.
import sys
sys.path.append('../../')

from math import radians
import argparse
import rospy
import numpy as np

import message_filters

from std_msgs.msg import String
from std_msgs.msg import Bool

from geometry_msgs.msg import Twist

from soccer.messages.BallDetectionMessage import BallDetectionMessage

STATE_TURN = 1
STATE_MOVE = 2

'''
Sucht den Ball.

Benötigt NodeBallDetection

Terminiert, wenn Ball gefunden

CMD-Line Parameter:
--num N    Nach N 'Ball-gefunden' Nachrichten stoppt die Suche (Minimierung false positives)
'''

class NodeBallDetection(object):
    
    def __init__(self, numFoundsInSequence=1, turnLeft=False, name="NodeBallSearch"):
        rospy.init_node(name, anonymous=False)
        rospy.loginfo("Stop ball search by pressing CTRL + C")

        # Subscribe to NodeBallDetection
        rospy.Subscriber("/soccer/balldetection/ballPosition", String, self.callbackBallPosition, queue_size=1)
        
        # Publisher to movement
        self.move = rospy.Publisher('/soccer/movement', Twist, queue_size=10)
        self.found = rospy.Publisher("/soccer/ballsearch/found", Bool, queue_size=1)

        r = rospy.Rate(5)
        
        # Turn
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(45) if turnLeft else radians(-45); #45 deg/s in radians/s
        
        # Move straight
        move_cmd = Twist()
        move_cmd.linear.x = 0.25 # 0.25 m/s
        move_cmd.angular.z = 0
        
        self.numMsgsInSequenceFoundBall = 0
        self.lastMsgFoundBall = False
        self.numFoundsInSequence = numFoundsInSequence
        
        self.run = True
        
        rounds = 0
        
        state = STATE_TURN

        while not rospy.is_shutdown() and self.run:
            
            if state == STATE_TURN:
	            # Spin for 2 seconds (10 x 5 HZ)
                #          1 second  ( 5 x 5 Hz)
                for x in range(0,5):
                    if self.run:
                        self.move.publish(turn_cmd)
                        r.sleep()

                rounds = rounds + 1
                if rounds == 11:
                    #self.run = False
                    state = STATE_MOVE
                    rounds = 0
                    
            elif state == STATE_MOVE:
                # Move foreward for 1 second and start over
                for x in range(0,15):
                    if self.run:
                        self.move.publish(move_cmd)
                        r.sleep()
                                
                state = STATE_TURN
        self.found.publish(True)

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
    
    #try:
    NodeBallDetection(args.num, turnLeft=False) #False => Right
    #except:
    #    rospy.loginfo("Node terminated in main-method")
