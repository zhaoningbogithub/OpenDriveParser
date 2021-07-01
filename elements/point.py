class Point:
    def __init__(self, x:float = 0.0, y:float = 0.0, theta:float = 0.0, s:float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta
        #s是相对于第一个laneSection的偏移，也就是该点对应的参考线的长度
        self.s = s

    def __str__(self):
        return str(self.x) + ',' + str(self.y)

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getTheta(self):
        return self.theta

    def getS(self):
        return self.s

    def setX(self, x:float):
        self.x = x

    def setY(self, y:float):
        self.y = y

    def setTheta(self, theta:float):
        self.theta = theta

    def setS(self, s:float):
        self.s = s
