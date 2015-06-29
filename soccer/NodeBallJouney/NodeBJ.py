import sys
sys.path.append('../../')

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import numpy
from math import radians
import random
import math
import time



from messages.BallDetectionMessage import BallDetectionMessage
from messages.DirectionMessage import DirectionMessage
from geometry_msgs.msg import Twist

CORRECT_HEADING = 1
DISTANCE = 3
OPPOSITE_HEADING = 5
OPPOSITE_HEADING_TURN = 6
OPPOSITE_HEADING_TURN_DONE = 7
COMPLETE = 10
LOST_BALL = 99

MIN_LINEAR_SPEED = 0.1
MAX_LINEAR_SPEED = 0.1
MIN_ANGULAR_SPEED = 0.33
MAX_ANGLUAR_SPEED = 1

MIN_DISTANCE = 650


class NodeBallJourney(object):
    
    
    def __init__(self, name="NodeBallJourney"):
        
        rospy.init_node(name, anonymous=False)
        rospy.loginfo("Stop detection by pressing CTRL + C")
        
        rospy.Subscriber("/soccer/balljourney/run", Bool, self.runCallback, queue_size=1)
        rospy.Subscriber("/soccer/balldetection/ballPosition", String, self.detectionCallBack, queue_size = 1)
        rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.directionCallback, queue_size = 1)
        
        self.lostEvent = rospy.Publisher('soccer/lostBall', String, queue_size=3)
        self.finishedEvent = rospy.Publisher('/soccer/balljourney/finished', String, queue_size=3)
        self.cmd_vel = rospy.Publisher('/soccer/movement', Twist, queue_size=1)
        
        self.ballMessage = BallDetectionMessage(0,0,0)
        self.distance = [0,0,0,0,0] # Calculate the Average over the last 5 distances
        self._distanceIndex = 0
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = 0

        self.ballMessage.x = 0
        move_cmd.angular.z = 0
        self.run = True
        
        self.heading =0
        
        r = rospy.Rate(10)
        
        self.state = -1
        self.lostBall = 0
        _distanceCount = 0
        while(not rospy.is_shutdown() and self.run):
            if self.correctHeading():
                if(self.ballMessage.ballDetected):
                    print self.avgDistance()
                    if self.avgDistance > MIN_DISTANCE:
                        move_cmd.linear.x = self.calculateLinearSpeed(self.avgDistance())
                        move_cmd.angular.z = self.calculateAngularSpeed()
                        print self.calculateAngularSpeed()
            else:
                if(self.ballMessage.ballDetected):
                    move_cmd.linear.x = MAX_LINEAR_SPEED*2
                    if self.ballMessage.x > 50:
                        move_cmd.angular.z = self.calculateAngularSpeed(70,99)
                    else:
                        move_cmd.angular.z = self.calculateAngularSpeed(1,30)
                    i = 0
                    print "NOW TURN"
                    while i< 3:
                        self.cmd_vel.publish(move_cmd)
                        i = i+1
                        r.sleep()
                    move_cmd.angular.z = 0
                    print "STRAIGHT"
                    i = 0
                    while self.lostBall < 90 or i < 100:
                        self.cmd_vel.publish(move_cmd)
                        i = i+1
                        r.sleep()
                    i = 0
                    self.lostBall = 0
                    move_cmd.linear.x = 0
                    print "U-TURN"
                    while i < 6:
                        print i
                        self.lostBall = 0
                        move_cmd.angular.z = 1.5
                        i = i+1
                        self.cmd_vel.publish(move_cmd)
                        r.sleep()

            self.cmd_vel.publish(move_cmd)
            r.sleep()
            if self.lostBall > 50:
                self.run = False
                self.lostEvent.publish("LOST BALL")





# CALLBACKS
    def runCallback(self, data):
        self.run = data.data
    
    def directionCallback(self, data):
        w = float(data.orientation.w)
        angle = math.acos(w)
        self.heading = angle
    #  print self.correctHeading()
    
    def detectionCallBack(self, data):
        self.ballMessage = BallDetectionMessage.fromJSONString(data.data)
        if self.ballMessage.ballDetected:
            self.distance[self._distanceIndex] = self.ballMessage.distance
            self._distanceIndex = self._distanceIndex + 1
            self._distanceIndex = self._distanceIndex % 5
            self.lostBall = 0
        else:
            self.lostBall += 1

#print data



# Calculations
    def calculateLinearSpeed(self, distance, min_distance = MIN_DISTANCE, maxSpeed = MAX_LINEAR_SPEED):
        if distance < MIN_DISTANCE:
            return 0
        x = (maxSpeed/2000 * distance) + 0.1
        return x

    def correctHeading(self):
        return self.heading < 0.75
    #        return abs(self.goalPosition - self.heading) <= 90


    def calculateAngularSpeed(self, leftBorder=40, rightBorder=60, maxSpeed = MAX_ANGLUAR_SPEED):
        if leftBorder <= self.ballMessage.x and self.ballMessage.x <= rightBorder:
            return 0.0
        if leftBorder > self.ballMessage.x:
            return maxSpeed-self.ballMessage.x*(maxSpeed/leftBorder)
        if rightBorder < self.ballMessage.x:
            a = (maxSpeed/(100 - rightBorder))
            b = a* 100 - maxSpeed
            return -(self.ballMessage.x * a - b)


    def setState(self, state):
        if(state != self.state):
            print "New State = " + str(state) + " " + str(self.lostBall) + " distance: " + str(self.avgDistance)
            self.state = state
        
    def avgDistance(self):
        return numpy.mean(self.distance)


if __name__ == '__main__':
    NodeBallJourney()
