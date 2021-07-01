from .getDoc import *
from .point import Point
from .decisionConstant import *
import math

class LaneLinkElement:
    def __init__(self, node_element):
        self.id = get_int(node_element, 'id')

    def getId(self) -> int:
        return self.id


class LaneLink:
    def __init__(self, node_roadlink):
        # <link>
        #     <predecessor id="-1" />
        #     <successor id="-1" />
        # </link>
        self.predecessor = None
        self.successor = None

        node_predecessor = node_roadlink.find('predecessor')
        if node_predecessor is not None:
            self.predecessor = LaneLinkElement(node_predecessor)

        node_successor = node_roadlink.find('successor')
        if node_successor is not None:
            self.successor = LaneLinkElement(node_successor)

    def getPredecessor(self) -> LaneLinkElement:
        return self.predecessor

    def getSuccessor(self) -> LaneLinkElement:
        return self.successor


class LaneWidth:
    def __init__(self, node_width):
        # <width a="0.2" b="0" c="0" d="0" sOffset="0" />
        self.a = get_float(node_width, 'a')
        self.b = get_float(node_width, 'b')
        self.c = get_float(node_width, 'c')
        self.d = get_float(node_width, 'd')
        self.sOffset = get_float(node_width, 'sOffset')

    def getA(self) -> float:
        return self.a

    def getB(self) -> float:
        return self.b

    def getC(self) -> float:
        return self.c

    def getD(self) -> float:
        return self.d

    def getSOffset(self) -> float:
        return self.sOffset


class LaneMark:
    def __init__(self, node_mark):
        # <roadMark color="standard" sOffset="0" type="none" weight="standard" />
        self.color = get_string(node_mark, 'color')
        self.sOffset = get_float(node_mark, 'sOffset')
        self.type = get_string(node_mark, 'type')
        self.weight = get_string(node_mark, 'weight')
        self.laneChange = get_string(node_mark, 'laneChange')
        #self.laneChange = 'both'

    def getSOffset(self):
        return self.sOffset

    def getType(self):
        return self.type

    def getLaneChange(self):
        return self.laneChange


class LaneMarkEmpty:
    def __init__(self, s:float, type:str, laneChange:str):
        self.sOffset = s
        self.type = type
        self.laneChange = laneChange

    def getSOffset(self) -> float:
        return self.sOffset

    def getType(self) -> str:
        return self.type

    def getLaneChange(self) -> str:
        return self.laneChange


class LaneSpeed:
    def __init__(self, node_speed):
        # <speed sOffset="0" max="13.8" />
        self.sOffset = get_float(node_speed, 'sOffset')
        self.max = get_float(node_speed, 'max')


class Lane:
    def __init__(self, node_lane):
        self.s = -1
        self.roadId = -1
        self.laneSectionId = -1
        self.laneId = get_int(node_lane, 'id')
        self.uniqueId = 0
        self.level = get_string(node_lane, 'level')
        self.type = get_string(node_lane, 'type')
        self.link = None
        self.widths = []
        self.roadMarks = []
        self.speed = []
        self.length = -1

        self.laneChangeFlag = []  #intlist
        self.laneWidth = [] #floatlist

        self.centerLine = [] #PointList
        self.leftLine = []
        self.rightLine = []

        node_link = node_lane.find('link')
        if node_link is not None:
            self.link = LaneLink(node_link)

        for node_width in node_lane.findall('width'):
            width = LaneWidth(node_width)
            self.widths.append(width)

        for node_mark in node_lane.findall('roadMark'):
            roadMark = LaneMark(node_mark)
            self.roadMarks.append(roadMark)

        if len(self.roadMarks) == 0:
            self.roadMarks.append(LaneMarkEmpty(self.s, 'none', 'none'))

        for node_speed in node_lane.findall('speed'):
            roadSpeed = LaneSpeed(node_speed)
            self.speed.append(roadSpeed)

    def setId(self, roadId:int, laneSectionId:int):
        self.roadId = roadId
        self.laneSectionId = laneSectionId

    def setS(self, s:float):
        self.s = s

    def getWidths(self) -> list:
        return self.widths

    def getRoadId(self) -> int:
        return self.roadId

    def getLaneSectionId(self) -> int:
        return self.laneSectionId

    def getId(self) -> int:
        return self.laneId

    def getUniqueId(self) -> int:
        return self.uniqueId

    def getType(self) -> str:
        return self.type

    def getLevel(self) -> str:
        return self.level

    def getLength(self) -> float:
        return self.length

    def setLength(self, length:float):
        self.length = length

    def getNormalWidth(self) -> float:
        widthSize = len(self.widths)
        normalWidth = 0
        for i in range(0, widthSize):
            a = self.widths[i].getA()
            b = self.widths[i].getB()
            c = self.widths[i].getC()
            d = self.widths[i].getD()
            if a > normalWidth and math.fabs(b) < 1e-4 and math.fabs(c) < 1e-4 and math.fabs(d) < 1e-4:
                normalWidth = a
        return normalWidth

    def getSOffset(self):
        return self.s

    def getWidths(self) -> list:
        return self.widths

    def getRoadId(self) -> int:
        return self.roadId

    def getLaneSectionId(self) -> int:
        return self.laneSectionId

    def getId(self) -> int:
        return self.laneId

    def getType(self) -> str:
        return self.type

    def getLevel(self) -> str:
        return self.level

    def getLink(self) -> LaneLink:
        return self.link

    def getRoadMarkSize(self) -> int:
        return len(self.roadMarks)

    def getSpeedSize(self) -> int:
        return len(self.speed)

    def setUniqueId(self, uniqueId:int):
        self.uniqueId = uniqueId

    def getLaneChangeFlag(self) -> list:
        return self.laneChangeFlag

    def getLaneWidth(self) -> list:
        return self.laneWidth

    def getCenterLine(self) -> list:
        return self.centerLine

    def getLeftLine(self) -> list:
        return self.leftLine

    def getRightLine(self) -> list:
        return self.rightLine

    def setCenterLine(self, centerLine:list):
        self.centerLine = centerLine

    def setLeftLine(self, leftLine:list):
        self.leftLine = leftLine

    def setRightLine(self, rightLine:list):
        self.rightLine = rightLine

    def format(self):
        lanePointsLength = len(self.centerLine)
        widthIndex = 0
        widthSize = len(self.widths)
        roadMarkIndex = 0
        roadMarkSize = len(self.roadMarks)
        self.laneWidth = [0.0] * lanePointsLength
        self.laneChangeFlag = [0] * lanePointsLength
        #设置中心线和左右车道线的方向
        if lanePointsLength > 1:
            for i in range(1, lanePointsLength):
                self.centerLine[i].setTheta(math.atan2(self.centerLine[i].getY() - self.centerLine[i - 1].getY(),
                                                       self.centerLine[i].getX() - self.centerLine[i - 1].getX()))
                self.centerLine[i].setTheta(math.atan2(self.leftLine[i].getY() - self.leftLine[i - 1].getY(),
                                                       self.leftLine[i].getX() - self.leftLine[i - 1].getX()))
                self.centerLine[i].setTheta(math.atan2(self.rightLine[i].getY() - self.rightLine[i - 1].getY(),
                                                       self.rightLine[i].getX() - self.rightLine[i - 1].getX()))
            self.centerLine[0].setTheta(self.centerLine[1].getTheta())
            self.leftLine[0].setTheta(self.leftLine[1].getTheta())
            self.rightLine[0].setTheta(self.rightLine[1].getTheta())

        for i in range(0, lanePointsLength):
            j = i
            if self.laneId > 0:
                #因为centerLine的排序是按照车辆行驶方向，当laneId>0时，按照s从大到小排序,所以应该转变成按照s从大到小计算width
                j = lanePointsLength - 1 - i
            p = self.centerLine[j]
            #计算宽度
            p_s = p.getS()
            if widthIndex == widthSize - 1:
                widthEndS = self.s + self.length
            else:
                widthEndS = self.s + self.widths[widthIndex + 1].getSOffset()
            if p_s - widthEndS > 1e-5:
                widthIndex += 1
            a = self.widths[widthIndex].getA()
            b = self.widths[widthIndex].getB()
            c = self.widths[widthIndex].getC()
            d = self.widths[widthIndex].getD()
            p_s -= self.s + self.widths[widthIndex].getSOffset()
            self.laneWidth[j] = a + p_s * b + p_s ** 2 * c + p_s ** 3 * d

            #计算laneChange
            p_s = p.getS()
            if roadMarkIndex == roadMarkSize - 1:
                roadMarkEndS = self.s + self.length
            else:
                roadMarkEndS = self.s + self.roadMarks[roadMarkIndex + 1].getSOffset()
            if p_s - roadMarkEndS > 1e-5:
                roadMarkIndex += 1
            laneChange = self.roadMarks[roadMarkIndex].getLaneChange()
            #左右都可以换道
            if laneChange == 'both':
                self.laneChangeFlag[j] = LaneChangeBoth
            #左右都不可以换道
            elif laneChange == 'none':
                self.laneChangeFlag[j] = LaneChangeNone
            #可以往左换道
            elif laneChange == 'increase' and self.laneId < 0 or laneChange == 'decrease' and self.laneId > 0:
                self.laneChangeFlag[j] = LaneChangeLeft
            #可以往右换道
            elif laneChange == 'increase' and self.laneId > 0 or laneChange == 'decrease' and self.laneId < 0:
                self.laneChangeFlag[j] = LaneChangeRight

        self.laneChangeFlag[int(lanePointsLength / 2)] = LaneChangeBoth
        self.laneChangeFlag[lanePointsLength - 1] = LaneChangeBoth

