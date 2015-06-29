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

class NodeBallJourney(object):


    def __init__(self, name="NodeBallJourney"):

        rospy.init_node(name, anonymous=False)
        rospy.loginfo("Stop detection by pressing CTRL + C")

        rospy.Subscriber("/soccer/balljourney/run", Bool, self.runCallback, queue_size=1)
        rospy.Subscriber("/soccer/balldetection/ballPosition", String, self.detectionCallBack, queue_size = 1)
        rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.directionCallback, queue_size = 1)
        
        self.lostEvent = rospy.Publisher('soccer/lostBall', String, queue_size=3)
        self.finishedEvent = rospy.Publisher('/soccer/balljourney/finished', String, queue_size=3)

        #self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=3)
        self.cmd_vel = rospy.Publisher('/soccer/movement', Twist, queue_size=1)
        
        self.goalPosition = 0
        self.heading = 0
        self.ballMessage = BallDetectionMessage(0,0,0)
        self.distance = [0,0,0,0,0]
        self._distanceIndex = 0

        self.run = True

        linear_speed = 0
        angular_speed = 0
        r = rospy.Rate(10)
        move_cmd = Twist()
        self.state = -1
        self.lostBall = 0
        while(not rospy.is_shutdown()):
            while(not rospy.is_shutdown()):
                if(self.state != LOST_BALL):
                    if self.ballMessage.ballDetected:
                        if self.avgDistance > 650:
                            if self.correctHeading():
                                # Ball Tor und Roboter stehen in Richtiger Konstelation zusammen
                                self.setState(CORRECT_HEADING)
                            else:
                                # Der Ball soll von der anderen seite angefahren werden
                                self.setState(OPPOSITE_HEADING)

                    else:
                            if self.correctHeading():
                                # ZU NAH DRAN
                                self.setState(DISTANCE)
                            else:
                                # Der Ball soll von der anderen seite angefahren werden
                                    self.setState(OPPOSITE_HEADING)
            

                if self.lostBall >50:
                    self.setState(LOST_BALL)
                
                
                
                if self.state == CORRECT_HEADING or self.state == DISTANCE:
                    linear_speed = self.calculateLinearSpeed(self.ballMessage.distance)
                    angular_speed = self.calculateAngularSpeed()
                    if angular_speed < 0.05:
                        self.setState(CORRECT_HEADING)
                elif self.state == OPPOSITE_HEADING:
                    for x in xrange(3):
                        linear_speed = 0.5
                        if self.ballMessage.ballPosition > 50:
                            angular_speed = self.calculateAngularSpeed(70,99)
                        else:
                            angular_speed = self.calculateAngularSpeed(1,30)
                        r.sleep
                    self.setState(OPPOSITE_HEADING_TURN)
                elif self.state == OPPOSITE_HEADING_TURN:
                    for x in range(0,10):
                        linear_speed = 0
                        angular_speed = 1
                        move_cmd.linear.x = linear_speed
                        move_cmd.angular.z = angular_speed
                        self.cmd_vel.publish(move_cmd)
                    self.setState(OPPOSITE_HEADING_TURN_DONE)
                    self.lostBall = 0
                elif self.state == OPPOSITE_HEADING_TURN_DONE:
                    if self.ballMessage.ballDetected:
                        self.state = CORRECT_HEADING
                    elif self.lostBall > 4000:
                            self.state = LOST_BALL
                elif self.state == LOST_BALL:
                    self.run = False
                    self.linear_speed = 0
                    self.angular_speed = 0
                    self.lostEvent.publish("LOST BALL")
                
                move_cmd.linear.x = linear_speed
                move_cmd.angular.z = angular_speed
                self.cmd_vel.publish(move_cmd)
            r.sleep()





    # CALLBACKS
    def runCallback(self, data):
        
        self.run = data.data

    def directionCallback(self, data):
        w = float(data.orientation.w)
        angle = math.acos(w)
        self.heading = angle

    def detectionCallBack(self, data):
        self.ballMessage = BallDetectionMessage.fromJSONString(data.data)
        if self.ballMessage.ballDetected:
            self.distance[self._distanceIndex] = self.ballMessage.distance
            self._distanceIndex = self._distanceIndex + 1
            self._distanceIndex = self._distanceIndex % 5
            self.lostBall = 0
        else:
            self.lostBall += 1


    # Calculations
    def calculateLinearSpeed(self, distance, min_distance = 650, maxSpeed = 0.1):
        x = (maxSpeed/2000 * distance) % maxSpeed
        return maxSpeed

    def correctHeading(self):
        return self.heading < 0.75
    #        return abs(self.goalPosition - self.heading) <= 90


    def calculateAngularSpeed(self, leftBorder=45, rightBorder=55, maxAngularSpeed = 0.5):
        return 0
        if leftBorder <= self.ballMessage.x and self.ballMessage.x <= rightBorder:
            return 0.0
        if leftBorder > self.ballMessage.x:
            return maxAngularSpeed #maxAngularSpeed-self.ballMessage.x*(maxAngularSpeed/leftBorder)
        if rightBorder < self.ballMessage.x:
            a = (maxAngularSpeed/(100 - rightBorder))
            b = a* 100 - maxAngularSpeed
            return -maxAngularSpeed#-(self.ballMessage.x * a - b)


    def setState(self, state):
        if(state != self.state):
            print "New State = " + str(state) + " " + str(self.lostBall) + " distance: " + str(self.avgDistance())
        self.state = state

    def avgDistance(self):
        return numpy.mean(self.distance)


if __name__ == '__main__':
    NodeBallJourney()
