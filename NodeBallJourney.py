import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
import numpy as np
from math import radians


from messages.BallDetectionMessage import BallDetectionMessage
from messages.DirectionMessage import DirectionMessage
from geometry_msgs.msg import Twist


class NodeBallJourney(object):

    def __init__(self, name="NodeBallJourney"):

        rospy.init_node(name, anonymous=False)
        rospy.loginfo("Stop detection by pressing CTRL + C")

        #rospy.Subscriber("/soccer/balljourney/run", Bool, self.runCallback, queue_size=1)
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
        self.state = 0
        while(not rospy.is_shutdown()):
            while(self.run):
                if self.ballMessage.distance > 10 and self.ballMessage.ballDetected: # TODO: Wert anpassen
                    # Bei Mindestabstand zum Ball -> anfahrt
                    self.state = 0
                    if self.correctHeading():
                        # Ball Tor und Roboter stehen in Richtiger Konstelation zusammen
                        linear_speed = 0.1
                        angular_speed = self.calculateAngularSpeed()
                    else:
                        # Der Ball von der anderen seite angefahren werden
                        linear_speed = 0.125
                        angular_speed = self.calculateAngularSpeed(80,100)
                else:
                    if self.state == 0:
                        if self.ballMessage.ballDetected:
                            linear_speed = 0.0
                            angular_speed = self.calculateAngularSpeed()
                        else:
                            self.state = 1
                    elif self.state == 1:
                        for x in range(0,10):
                            move_cmd.linear.x = 0
                            move_cmd.angular.z = radians(180)
                            self.cmd_vel.publish(move_cmd)
                            r.sleep()
                        self.state = 2
                    elif self.state == 2:
                        if self.ballMessage.ballDetected:
                            self.state = 0
                        else:
                            # lost ball -> do something
                            self.state = 0


                move_cmd.linear.x = linear_speed
                linear_speed.angular.z = angular_speed
                self.cmd_vel.publish(move_cmd)
            r.sleep()





    # CALLBACKS
    def runCallback(self, data):
        self.run = data

    def directionCallback(self, data):
        message = DirectionMessage.fromJSONString(data.data)
        if(message.type == DirectionMessage.GOAL_DIRECTION):
            self.goalPosition = message.degrees
        if(message.type == DirectionMessage.SELF_DIRECTION):
            self.heading == message.degrees

    def detectionCallBack(self, data):
        self.ballMessage = BallDetectionMessage.fromJSONString(data.data)


    # Calculations
    def correctHeading(self):
        return range(((self.heading -90) % 360),((self.heading +90) % 360))

    def calculateAngularSpeed(self, leftBorder=40, rightBorder=60, maxAngularSpeed = 0.33):

        if leftBorder >= self.ballMessage.x <= rightBorder:
            return 0.0
        if leftBorder > self.ballMessage.x:
            return maxAngularSpeed-self.ballMessage.x*(maxAngularSpeed/leftBorder)
        if rightBorder < self.ballMessage.x:
            a = (maxAngularSpeed/(100 - rightBorder))
            b = a* 100 - maxAngularSpeed
            return self.ballMessage.x * a - b



if __name__ == '__main__':
    NodeBallJourney()