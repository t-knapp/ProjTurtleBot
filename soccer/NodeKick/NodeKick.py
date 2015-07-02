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


#from messages.GoalDetectionMessage import GoalDetectionMessage as GDM
from geometry_msgs.msg import Twist



class NodeBallJourney(object):
    
    
    def __init__(self, name="NodeBallJourney"):
        
        rospy.init_node(name, anonymous=False)
        rospy.loginfo("Stop detection by pressing CTRL + C")
        
#        rospy.Subscriber("/soccer/goalDetection/??", String, self.detectionCallBack, queue_size = 1)
        self.cmd_vel = rospy.Publisher('/soccer/movement', Twist, queue_size=1)

        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = 0
        
        self.kick = True
        self.run = True
        r = rospy.Rate(10)

        while(not rospy.is_shutdown() and self.run):
            while (self.kick):
                self.run = False
                self.kick = False
#                if True:# or self.goal.status == GDM.STATESTRAIGHT:
                move_cmd.linear.x = 3
                
                i = 0
                while  i < 5:
                    i = i+1
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                i = 0
                #move_cmd.linear.x = -1
                while i<3:
                    i = i+1
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
    
    







# CALLBACKS

    def detectionCallBack(self, msg):
        #self.goal = GDM.fromJSONString(msg.data)
        self.kick = True



if __name__ == '__main__':
    NodeBallJourney()
