import json

''' Messages Send as Python from the BallDetection Node to all
listening Nodes

x in the Range from -1 to 100 - tells where the Ball was detectet horizontally / -1 if no ball detected
y in the Range from -1 to 100 - tells where the Ball was detectet vertically / -1 if no ball detected
distance tells the distance to the ball

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
    def __init__(self, x= 50, y= 50, distance= 0):
        self.x = x
        self.y = y
        self.distance = distance;

    def toJSONString(self):
        pyDict = {'x':self.x, 'y':self.y, 'distance': self.distance}
        return json.dumps(pyDict)

    @classmethod
    def fromJSONString(cls, jsonString):
        pyDict = json.loads(jsonString)
        return BallDetectionMessage(pyDict['x'],pyDict['y'],pyDict['distance'])


