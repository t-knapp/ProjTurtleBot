import rospy
from messages.DirectionMessage import DirectionMessage
from messages.BallDetectionMessage import BallDetectionMessage
from std_msgs.msg import Bool
import numpy as np
import json
from std_msgs.msg import String



class InputWindow(object):
    def __init__(self, name="NodeMock"):
        rospy.init_node(name, anonymous=False)
        dm = DirectionMessage(0, DirectionMessage.GOAL_DIRECTION)
        bdm = BallDetectionMessage()
        bdm_pub = rospy.Publisher("/soccer/balldetection/ballPosition", String, queue_size = 1)
        dm_pub =  rospy.Publisher("/soccer/heading/", String, queue_size = 1)
        run_pub = rospy.Publisher("/soccer/balljourney/run", Bool, queue_size=1)

        dm.GOAL_DIRECTION
        dm.degrees = 90;
        dm_pub.publish(String(dm.toJSONString()))
        dm.SELF_DIRECTION
        dm_pub.publish(String(dm.toJSONString()))
        bdm_pub.publish(String(bdm.toJSONString()))
        bdm.distance = 100
        bdm.y = 4
        bdm_pub.publish(String(bdm.toJSONString()))
        while True:
            x = input("Enter x: ")
            bdm.x =  x
            bdm_pub.publish(String(bdm.toJSONString()))
            if x == 99:
                run_pub.publish(Bool(False))








if __name__ == '__main__':
    InputWindow()
