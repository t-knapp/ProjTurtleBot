import json

''' Messages Send as Python from the BallDetection Node to all
listening Nodes

x in the Range from 0 to 100 - tells where the Ball was detectet horizontally
y in the Range from 0 to 100 - tells where the Ball was detectet vertically
distance - tells the distance to the ball
ballDetected - true if the ball was detected, false if no ball was detected

   y^
100_|
    |
    |
    |
    |
    |               o <- Ball
    |
    |
    |
    |                                  x
    +-------------------------------->
    0                               100
'''


class BallDetectionMessage:
    def __init__(self, x= 50, y= 50, distance= 0, ballDetected = True):
        self.x = x
        self.y = y
        self.distance = distance;
        self.ballDetected = ballDetected

    def toJSONString(self):
        pyDict = {'x':self.x, 'y':self.y, 'distance': self.distance, 'ballDetected' : self.ballDetected}
        return json.dumps(pyDict)

    @classmethod
    def fromJSONString(cls, jsonString):
        pyDict = json.loads(jsonString)
        return BallDetectionMessage(pyDict['x'],pyDict['y'],pyDict['distance'], pyDict['ballDetected'])


