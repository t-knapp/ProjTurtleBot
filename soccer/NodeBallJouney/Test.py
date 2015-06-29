from Tkinter import *
import sys
sys.path.append('../../')
import rospy
from messages.DirectionMessage import DirectionMessage
from messages.BallDetectionMessage import BallDetectionMessage
from std_msgs.msg import String
from std_msgs.msg import Bool

class BallDetectionMockWindow():
    def __init__(self):
        rospy.init_node("MockupDingsi", anonymous=False)
        root = Tk()
        root.title("Ball Detection Mock Window")
        root.geometry("200x200")
        

        self.run_pub = rospy.Publisher("/soccer/balljourney/run", Bool, queue_size=1)
        self.bdm_pub = rospy.Publisher("/soccer/balldetection/ballPosition", String, queue_size = 1)
        self.referee = rospy.Publisher("/soccer/referee", Bool, queue_size = 1)
    

        Label(root, text="x").grid(row=0)
        Label(root, text="y").grid(row=1)
        Label(root, text="distance").grid(row=2)
        Label(root, text="Goal Dir").grid(row=3)
        Label(root, text="Own Dir").grid(row=4)
        self.e1 = Entry(root)
        self.e1.insert(0, "0")
        self.e2 = Entry(root)
        self.e2.insert(0, "0")
        self.e3 = Entry(root)
        self.e3.insert(0, "0")
        self.e4 = Entry(root)
        self.e4.insert(0, "0")
        self.e5 = Entry(root)
        self.e5.insert(0, "0")

        self.e1.grid(row=0, column=1)
        self.e2.grid(row=1, column=1)
        self.e3.grid(row=2, column=1)
        self.e4.grid(row=3, column=1)
        self.e5.grid(row=4, column=1)

        b = Button(root, text="send", width=10, command=self.send)
        b1 = Button(root, text="On", width=10, command=self.on)
        b2 = Button(root, text="Off", width=10, command=self.off)
        
        b.grid(row = 5, column=1)
        b1.grid(row = 6, column=1)
        b2.grid(row = 7, column=1)


        root.mainloop()

    def send(self):
        bdm = BallDetectionMessage()
        bdm.x = int(self.e1.get())
        bdm.y = int(self.e2.get())
        bdm.distance = int(self.e3.get())
        self.bdm_pub.publish(String(bdm.toJSONString()))
        dm = DirectionMessage(int(self.e4.get()), DirectionMessage.GOAL_DIRECTION)
        self.dm_pub.publish(String(dm.toJSONString()))
        dm = DirectionMessage(int(self.e5.get()), DirectionMessage.SELF_DIRECTION)
        self.dm_pub.publish(String(dm.toJSONString()))


    def on(self):
        self.sendRun(True)
        self.referee.publish(True)


    def off(self):
        self.sendRun(False)
        self.referee.publish(False)

    def sendRun(self, b):
        i = 1
        #self.run_pub.publish(Bool(b))


if __name__ == '__main__':
    org_msg = BallDetectionMessage(0,0,0)
    print(org_msg.toJSONString())
    new_msg = BallDetectionMessage.fromJSONString(org_msg.toJSONString())
    print(new_msg.toJSONString())
    BallDetectionMockWindow()

