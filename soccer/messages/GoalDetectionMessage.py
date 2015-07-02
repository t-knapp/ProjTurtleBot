import json

''' Messages Send as Python from the BallDetection Node to all
listening Nodes

state        - STATENONE  => No goal (default)
               STATELEFT  => Goal on left side
               STATERIGHT => Goal on right side
               STATESTRAIGHT => Goal direction is straight ahead
distance     - tells the distance in mm to the goal

'''


class GoalDetectionMessage:
    
    STATENONE     = 0
    STATELEFT     = 1
    STATERIGHT    = 2
    STATESTRAIGHT = 4
    
    def __init__(self, state= STATENONE, distance= 0):
        self.state = state
        self.distance = distance;

    def toJSONString(self):
        pyDict = {'state':self.state, 'distance': self.distance}
        return json.dumps(pyDict)

    @classmethod
    def fromJSONString(cls, jsonString):
        pyDict = json.loads(jsonString)
        return GoalDetectionMessage(pyDict['state'], pyDict['distance'])

