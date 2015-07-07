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
from time import sleep
from kobuki_msgs.msg import Sound

from soccer.messages.GoalDetectionMessage import GoalDetectionMessage as GDM
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

SPEED = 1.55

DRIVE_LENGTH = 20 # in steps
TURN = 3


class NodeKick(object):


    
    def __init__(self, name="NodeKick"):
        
        rospy.init_node(name, anonymous=False)
        rospy.loginfo("Stop kick by pressing CTRL + C")
        
        rospy.Subscriber("/soccer/goalPosition", String, self.detectionCallBack, queue_size = 1)
        self.sound = rospy.Publisher('/mobile_base/commands/sound',Sound, queue_size=1)
        # Signal from NodeStrategy
        rospy.Subscriber("/soccer/kick/run", Bool, self.runCallback, queue_size=1)

        # Publish to NodeCollisionDetection to move
        self.cmd_vel = rospy.Publisher('/soccer/movement', Twist, queue_size=1)
        self.finished = rospy.Publisher("/soccer/kick/finished", Bool, queue_size=1)
        self.move_cmd = Twist()
        self.move_cmd.angular.z = 0
        self.move_cmd.linear.x = 0
    
        self.run = False
        self.kick = False
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            while self.run:
                while self.kick:
                    self.sound.publish(Sound(Sound.CLEANINGSTART))
                    self.run = False
                    self.kick = False
                    if self.goal.state == GDM.STATESTRAIGHT or self.goal.state == GDM.STATENONE:
                        i = 0
                        #NOthing to do here


                    if self.goal.state == GDM.STATERIGHT:
                        self.move_cmd.linear.x = 0.2
                        i = 0
                        while  i < 5:
                            i = i+1
                            self.cmd_vel.publish(self.move_cmd)
                            r.sleep()

                    if self.goal.state == GDM.STATELEFT:
                        self.move_cmd.linear.x = 0.2
                        i = 0
                        while  i < 5:
                            i = i+1
                            self.cmd_vel.publish(self.move_cmd)
                            r.sleep()

                    # KICK
                    self.move_cmd.linear.x = SPEED
                    i = 0
                    while  i < DRIVE_LENGTH:
                        i = i+1
                        self.cmd_vel.publish(self.move_cmd)
                        r.sleep()

                    # Move Back
                    i = 0
                    self.move_cmd.linear.x = -SPEED
                    while i<3:
                        i = i+1
                        self.cmd_vel.publish(self.move_cmd)
                        r.sleep()
                    
                    self.move_cmd.linear.x = 0
                    self.cmd_vel.publish(self.move_cmd)
                    self.finished.publish(True)


    






# CALLBACKS

    def detectionCallBack(self, msg):
            self.goal = GDM.fromJSONString(msg.data)
            print msg.data
            self.kick = True

    def runCallback(self, msg):
        self.run = msg.data


if __name__ == '__main__':
    NodeKick()
    rospy.spin()
