import json


class DirectionMessage:
    BALL_DIRECTION = "BallDirection"
    GOAL_DIRECTION = "GoalDirection"
    SELF_DIRECTION = "CurrentHeading"

    def __init__(self, degrees, type):
        self.degrees = degrees
        self.type = type

    def toJSONString(self):
        pyDict = {'degrees':self.degrees, 'type': self.type}
        return json.dumps(pyDict)

    @classmethod
    def fromJSONString(cls, jsonString):
        pyDict = json.loads(jsonString)
        return cls(pyDict['degrees'], pyDict['type'])