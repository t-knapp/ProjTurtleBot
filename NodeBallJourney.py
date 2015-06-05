import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
import numpy as np
from math import radians
import random



from messages.BallDetectionMessage import BallDetectionMessage
from messages.DirectionMessage import DirectionMessage
from geometry_msgs.msg import Twist

CORRECT_HEADING = 1
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
        rospy.Subscriber("/soccer/heading/", String, self.directionCallback, queue_size = 1)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=3)

        self.goalPosition = 0
        self.heading = 0
        self.ballMessage = BallDetectionMessage(0,0,0)

        self.run = True

        linear_speed = 0
        angular_speed = 0
        r = rospy.Rate(10)
        move_cmd = Twist()
        self.state = -1
        self.lostBall = 0
        while(not rospy.is_shutdown()):
            while(self.run):
                print self.state
                if self.ballMessage.distance > 650 and self.ballMessage.ballDetected:
                    # Bei Mindestabstand zum Ball -> anfahrt
                    self.lostBall = 0
                    if True or self.correctHeading():
                        # Ball Tor und Roboter stehen in Richtiger Konstelation zusammen
                        self.state = CORRECT_HEADING
                        linear_speed = 0.1
                        angular_speed = self.calculateAngularSpeed()
                    else:
                        # Der Ball von der anderen seite angefahren werden
                        self.state = OPPOSITE_HEADING
                        linear_speed = 0.1
                        if random.randint(0,9) >=6:
                                angular_speed = self.calculateAngularSpeed(70,99)
                        else:
                                angular_speed = self.calculateAngularSpeed(1,30)
                else:
                    if self.ballMessage.ballDetected:
                        self.lostBall = 0
                    else:
                        self.lostBall = self.lostBall + 1

                    if self.state == CORRECT_HEADING:
                        if self.ballMessage.ballDetected:
                            linear_speed = 0.0
                            angular_speed = self.calculateAngularSpeed()
                            if angular_speed < 0.01:
                                self.state = COMPLETE
                        else:
			                linear_speed = 0.0
			                angular_speed = 0.0
                            #if self.lostBall > 200:
                            #    self.state = LOST_BALL

                    elif self.state == OPPOSITE_HEADING:
                        if self.ballMessage.ballDetected:
                            linear_speed = 1.0
                            angular_speed = self.calculateAngularSpeed(70,99)
                        else:
                            # make turn
                            self.state = OPPOSITE_HEADING_TURN

                    elif self.state == OPPOSITE_HEADING_TURN:
                        #
                        for x in range(0,10):
                            move_cmd.linear.x = 0
                            move_cmd.angular.z = radians(180)
                            self.cmd_vel.publish(move_cmd)
                            r.sleep()
                        self.state = 3

                    elif self.state == OPPOSITE_HEADING_TURN_DONE: 
                        if self.ballMessage.ballDetected:
                            self.state = CORRECT_HEADING
                        else:
                            if self.lostBall > 400:
                                self.state = LOST_BALL
                            else:
                                self.state =OPPOSITE_HEADING

                    elif self.state == LOST_BALL:
                        self.state = CORRECT_HEADING
                    elif self.state == COMPLETE:
                        self.state = COMPLETE


                move_cmd.linear.x = linear_speed
                move_cmd.angular.z = angular_speed
                self.cmd_vel.publish(move_cmd)
            r.sleep()





    # CALLBACKS
    def runCallback(self, data):
        print data
        self.run = data.data

    def directionCallback(self, data):
        print data
        message = DirectionMessage.fromJSONString(data.data)
        if(message.type == DirectionMessage.GOAL_DIRECTION):
            self.goalPosition = message.degrees
        if(message.type == DirectionMessage.SELF_DIRECTION):
            self.heading = message.degrees

    def detectionCallBack(self, data):
        print data
        self.ballMessage = BallDetectionMessage.fromJSONString(data.data)


    # Calculations
    def correctHeading(self):
        return abs(self.goalPosition - self.heading) <= 90
        
        
        return range(((self.heading -90) % 360),((self.heading +90) % 360))

    def calculateAngularSpeed(self, leftBorder=48, rightBorder=52, maxAngularSpeed = 0.8):

        if leftBorder <= self.ballMessage.x and self.ballMessage.x <= rightBorder:
            return 0.0
        if leftBorder > self.ballMessage.x:
            return maxAngularSpeed-self.ballMessage.x*(maxAngularSpeed/leftBorder)
        if rightBorder < self.ballMessage.x:
            a = (maxAngularSpeed/(100 - rightBorder))
            b = a* 100 - maxAngularSpeed
            return -(self.ballMessage.x * a - b)



if __name__ == '__main__':
    NodeBallJourney()
