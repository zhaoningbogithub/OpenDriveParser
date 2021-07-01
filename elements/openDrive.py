import copy
import matplotlib.pyplot as plt

from .road import *
from .lane import *
from .junction import Junction
from .header import Header
from .decisionConstant import *
import queue
from .getCal import *


class Position:
    def __init__(self, roadId: int, laneSectionId: int, laneId: int, offset: float):
        self.roadId = roadId
        self.laneSectionId = laneSectionId
        self.laneId = laneId
        self.offset = offset

    def __str__(self):
        return str(self.roadId) + "," + str(self.laneSectionId) + "," + str(self.laneId)


class Edge:
    def __init__(self, id: int):
        self.id = id
        self.next: Edge = None


class Node:
    def __init__(self, roadId: int, laneSectionId: int, laneId: int):
        self.roadId = roadId
        self.laneSectionId = laneSectionId
        self.laneId = laneId
        self.next: Edge = None


class Vertax:
    def __init__(self):
        self.id = -1
        self.cost = 0.0
        self.parent = -1
        self.closed = -1
        self.flag = -1
        self.startOffset = 0.0

    def __lt__(self, other):
        return self.cost < other.cost


class _Matrix:
    def __init__(self, mm: int = 0, nn: int = 0):
        self.m = mm
        self.n = nn

    def set_m(self, mm: int):
        self.m = mm

    def set_n(self, nn: int):
        self.n = nn

    def init_matrix(self):
        self.arr = [0.0] * self.m * self.n

    def read(self, i: int, j: int):
        if i >= self.m or j >= self.n:
            return -31415
        return self.arr[i * self.n + j]

    def write(self, i: int, j: int, val: float):
        if i >= self.m or j >= self.n:
            return -1

        self.arr[i * self.n + j] = val
        return 1


class _Matrix_Calc:
    def add(self, A: _Matrix, B: _Matrix, C: _Matrix):
        # 判断是否可以运算
        if A.m != B.m or A.n != B.n or A.m != C.m or A.n != C.n:
            return -1
        # 运算
        for i in range(0, C.m):
            for j in range(0, C.n):
                C.write(i, j, A.read(i, j) + B.read(i, j))

        return 1

    def subtract(self, A: _Matrix, B: _Matrix, C: _Matrix):
        # 判断是否可以运算
        if A.m != B.m or A.n != B.n or A.m != C.m or A.n != C.n:
            return -1
        # 运算
        for i in range(0, C.m):
            for j in range(0, C.n):
                C.write(i, j, A.read(i, j) - B.read(i, j))

        return 1

    def multiply(self, A: _Matrix, B: _Matrix, C: _Matrix):
        if A.m != C.m or B.n != C.n or A.n != B.m:
            return -1

        for i in range(0, C.m):
            for j in range(0, C.n):
                temp = 0
                for k in range(0, A.n):
                    temp += A.read(i, k) * B.read(k, j)
                C.write(i, j, temp)

        return 1

    def det(self, A: _Matrix):

        # 判断是否可以运算
        if A.m != A.n or (A.m != 2 and A.m != 3):
            return -31415

        # 运算
        if A.m == 2:
            value = A.read(0, 0) * A.read(1, 1) - A.read(0, 1) * A.read(1, 0)
        else:
            value = (A.read(0, 0) * A.read(1, 1) * A.read(2, 2) +
                     A.read(0, 1) * A.read(1, 2) * A.read(2, 0) +
                     A.read(0, 2) * A.read(1, 0) * A.read(2, 1) +
                     A.read(0, 0) * A.read(1, 2) * A.read(2, 1) +
                     A.read(0, 1) * A.read(1, 0) * A.read(2, 2) +
                     A.read(0, 2) * A.read(1, 1) * A.read(2, 0))
        return value

    def transpos(self, A: _Matrix, B: _Matrix):
        # 判断是否可以运算
        if A.m != B.n or A.n != B.m:
            return -1
        # 运算
        for i in range(0, B.m):
            for j in range(0, B.n):
                B.write(i, j, A.read(j, i))

        return 1

    def printff_matrix(self, A: _Matrix):
        m = A.m
        n = A.n
        for i in range(0, m):
            for j in range(0, n):
                print(A.read(i, j))

    def inverse(self, A: _Matrix, B: _Matrix):
        m = _Matrix(A.m, 2 * A.m)

        # 判断是否可以运算
        if A.m != A.n or B.m != B.n or A.m != B.m:
            return -1

        # 增广矩阵m = A | B 初始化
        m.init_matrix()
        for i in range(0, m.m):
            for j in range(0, m.n):
                if j <= A.n - 1:
                    m.write(i, j, A.read(i, j))
                else:
                    if i == j - A.n:
                        m.write(i, j, 1)
                    else:
                        m.write(i, j, 0)

        # 高斯消元
        # 变换下三角
        for k in range(0, m.m - 1):
            # 如果坐标为k，k的行数为0,则行变换
            if m.read(k, k) == 0:
                for i in range(k + 1, m.m):
                    if m.read(i, k) != 0:
                        break
                if i >= m.m:
                    return -1
                else:
                    # 交换行
                    for j in range(0, m.n):
                        temp = m.read(k, j)
                        m.write(k, j, m.read(k + 1, j))
                        m.write(k + 1, j, temp)
            # 消元
            for i in range(k + 1, m.m):
                # 获得倍数
                b = m.read(i, k) / m.read(k, k)
                # 行变换
                for j in range(0, m.n):
                    temp = m.read(i, j) - b * m.read(k, j)
                    m.write(i, j, temp)

        # 变换上三角
        for k in range(m.m - 1, 0, -1):
            # 如果坐标为k，k的数为0，则行变换
            if m.read(k, k) == 0:
                for i in range(k + 1, m.m):
                    if m.read(i, k) != 0:
                        break
                if i >= m.m:
                    return -1
                else:
                    # 交换行
                    for j in range(0, m.n):
                        temp = m.read(k, j)
                        m.write(k, j, m.read(k + 1, j))
                        m.write(k + 1, j, temp)
            # 消元
            for i in range(k - 1, -1, -1):
                # 获得倍数
                b = m.read(i, k) / m.read(k, k)
                # 行变换
                for j in range(0, m.n):
                    temp = m.read(i, j) - b * m.read(k, j)
                    m.write(i, j, temp)

        # 将左边的方阵化为单位矩阵
        for i in range(0, m.m):
            if m.read(i, i) != 1:
                # 获得倍数
                b = 1 / m.read(i, i)
                # 行变换
                for j in range(0, m.n):
                    temp = m.read(i, j) * b
                    m.write(i, j, temp)

        # 求得逆矩阵
        for i in range(0, B.m):
            for j in range(0, B.m):
                B.write(i, j, m.read(i, j + m.m))

        return 1


class CubicLine:
    def __init__(self, start: Point, end: Point):
        self.start = start
        self.end = end

    def calPoints(self):
        te = math.sqrt(
            math.pow(self.start.getX() - self.end.getX(), 2) + math.pow(self.start.getY() - self.end.getY(), 2))
        px = []
        py = []
        # 计算参数
        self.calPara(te, px, py)
        res = []
        t = 0
        while t < te:
            x = px[0] + t * px[1] + t * t * px[2] + t * t * t * px[3]
            y = py[0] + t * py[1] + t * t * py[2] + t * t * t * py[3]
            temp = Point(x, y, 0, 0)
            res.append(temp)
            t = t + 0.5
        t = te
        x = px[0] + t * px[1] + t * t * px[2] + t * t * t * px[3]
        y = py[0] + t * py[1] + t * t * py[2] + t * t * t * py[3]
        temp = Point(x, y, 0, 0)
        res.append(temp)
        # 计算角度
        size = len(res)
        for i in range(1, size):
            res[i].setTheta(math.atan2(res[i].getY() - res[i - 1].getY(), res[i].getX() - res[i - 1].getX()))
        res[0] = res[1]
        return res

    def calPara(self, te: float, px: list, py: list):
        A = _Matrix(2, 2)
        A.init_matrix()
        A.write(0, 0, math.pow(te, 3))
        A.write(0, 1, math.pow(te, 2))
        A.write(1, 0, math.pow(te, 2) * 3)
        A.write(1, 1, te * 2)

        b1 = _Matrix(2, 1)
        b1.init_matrix()
        b1.write(0, 0, self.end.getX() - self.start.getX() - math.cos(self.start.getTheta()) * te)
        b1.write(1, 0, math.cos(self.end.getTheta()) - math.cos(self.start.getTheta()))

        invA = _Matrix(2, 2)
        invA.init_matrix()
        param1 = _Matrix(2, 1)
        param1.init_matrix()
        c = _Matrix_Calc()
        c.inverse(A, invA)
        c.multiply(invA, b1, param1)
        # X的参数依次，abcd，a+bt+ct^2+dt^3
        px.append(self.start.getX())
        px.append(math.cos(self.start.getTheta()))
        px.append(param1.read(1, 0))
        px.append(param1.read(0, 0))

        b2 = _Matrix(2, 1)
        b2.init_matrix()
        b2.write(0, 0, self.end.getY() - self.start.getY() - math.sin(self.start.getTheta()) * te)
        b2.write(1, 0, math.sin(self.end.getTheta()) - math.sin(self.start.getTheta()))

        param2 = _Matrix(2, 1)
        param2.init_matrix()
        c.multiply(invA, b2, param2)

        py.append(self.start.getY())
        py.append(math.sin(self.start.getTheta()))
        py.append(param2.read(1, 0))
        py.append(param2.read(0, 0))

    def setStart(self, start: Point):
        self.start = start

    def setEnd(self, end: Point):
        self.end = end

    def getStart(self):
        return self.start

    def getEnd(self):
        return self.end


class Arc:
    def __init__(self, s: float, startPosition: Point, heading: float, length: float, curvature: float):
        self.startPosition = startPosition
        self.heading = heading
        self.s = s
        self.length = length
        self.curvature = curvature
        self.points = []

    def calPosition(self, step: float):
        x = 0.0
        y = 0.0
        theta = self.heading
        sx = self.startPosition.getX()
        sy = self.startPosition.getY()
        length = self.length

        points = []
        dis = 0.0
        while dis < length:
            seg = calArcSegment(x, y, sx, sy, dis, theta, self.curvature)
            p = Point(seg[0], seg[1], theta, self.s + dis)
            points.append(p)
            dis = dis + step
        for i in range(1, len(points)):
            points[i].setTheta(math.atan2(points[i].getY() - points[i - 1].getY(),
                                          points[i].getX() - points[i - 1].getX()))
        self.points = points

    def getPoints(self):
        return self.points


class OpenDrive:
    def __init__(self, node_root):
        self.header = None
        self.outPutPath = "./files/output"
        self.debug = True
        self.Z_ = -2.0
        self.laneNum = 0
        self.nodes = []
        self.invalidPosition = []
        self.roads = {}
        self.junctions = {}

        node_header = node_root.find('header')
        if node_header is not None:
            self.header = Header(node_header)

        for node_road in node_root.findall('road'):
            road = Road(node_road)
            self.roads[road.getId()] = road

        for road in self.roads.values():
            for laneSection in road.lanes.laneSections:
                if laneSection.getLeft() is not None:
                    for lane in laneSection.getLeft().getLanes().values():
                        lane.setS(laneSection.getS())
                        lane.setId(road.getId(), laneSection.getId())
                        lane.setUniqueId(self.laneNum)
                        self.laneNum += 1
                        self.nodes.append(Node(road.getId(), laneSection.getId(), lane.getId()))
                if laneSection.getCenter() is not None:
                    for lane in laneSection.getCenter().getLanes().values():
                        lane.setS(laneSection.getS())
                        lane.setId(road.getId(), laneSection.getId())
                        lane.setUniqueId(self.laneNum)
                        self.laneNum += 1
                        self.nodes.append(Node(road.getId(), laneSection.getId(), lane.getId()))
                if laneSection.getRight() is not None:
                    for lane in laneSection.getRight().getLanes().values():
                        lane.setS(laneSection.getS())
                        lane.setId(road.getId(), laneSection.getId())
                        lane.setUniqueId(self.laneNum)
                        self.laneNum += 1
                        self.nodes.append(Node(road.getId(), laneSection.getId(), lane.getId()))
        for node_junction in node_root.findall('junction'):
            junction = Junction(node_junction)
            self.junctions[junction.getId()] = junction

        self.validState = True

    def format(self, uniqueId: int):
        self.getLaneLengthByUniqueId(uniqueId)
        self.getLaneByUniqueId(uniqueId).format()

    def getRoad(self, roadId: int) -> Road:
        return self.roads[roadId]

    def roadExist(self, roadId) -> bool:
        return roadId in self.roads

    def getLaneById(self, roadId: int, laneSectionId: int, laneId: int) -> Lane:
        return self.roads[roadId].getLane(laneSectionId, laneId)

    def getLaneByUniqueId(self, uniqueId: int) -> Lane:
        node = self.nodes[uniqueId]
        return self.getLaneById(node.roadId, node.laneSectionId, node.laneId)

    def getLaneTypeById(self, roadId: int, laneSectionId: int, laneId: int) -> str:
        return self.getLaneById(roadId, laneSectionId, laneId).getType()

    def getLaneTypeByUniqueId(self, uniqueId: int) -> str:
        return self.getLaneTypeById(self.nodes[uniqueId].roadId, self.nodes[uniqueId].laneSectionId,
                                    self.nodes[uniqueId].laneId)

    def getLaneLengthById(self, roadId: int, laneSectionId: int, laneId: int) -> float:
        lane = self.getLaneById(roadId, laneSectionId, laneId)
        length = lane.getLength()
        if length > 0:
            return length
        road = self.getRoad(roadId)
        lanes = road.getLanes()
        startS = lanes.getLaneSectionById(laneSectionId).getS()
        if laneSectionId == len(lanes.getLaneSections()) - 1:
            endS = road.getLength()
        else:
            endS = lanes.getLaneSectionById(laneSectionId + 1).getS()

        length = endS - startS
        lane.setLength(length)
        return length

    def getLaneLengthByUniqueId(self, uniqueId: int) -> float:
        node = self.nodes[uniqueId]
        return self.getLaneLengthById(node.roadId, node.laneSectionId, node.laneId)

    def laneExist(self, roadId: int, laneSectionId: int, laneId: int) -> bool:
        return self.roadExist(roadId) and self.roads[roadId].laneExist(laneSectionId, laneId)

    def getLaneUniqueId(self, roadId: int, laneSectionId: int, laneId: int) -> int:
        if not self.laneExist(roadId, laneSectionId, laneId):
            return False
        lane = self.getLaneById(roadId, laneSectionId, laneId)
        return lane.getUniqueId()

    # reverse == True refPoints中的Points按照S递减存放，否则递增存放
    # start == True 返回S右边的第一个点
    # start == False 返回S左边的第一个点
    def findIndex(self, refPoints: list, s: float, reverse: bool, start: bool):
        startIndex = 0
        endIndex = len(refPoints) - 1
        while startIndex <= endIndex:
            midIndex = int((startIndex + endIndex) / 2)
            if math.fabs(refPoints[midIndex].getS() - s) <= 1e-5:
                return midIndex
            if refPoints[midIndex].getS() - s > 1e-5 and reverse or s - refPoints[
                midIndex].getS() > 1e-5 and not reverse:
                startIndex = midIndex + 1
            else:
                endIndex = midIndex - 1

        if start:
            if startIndex == len(refPoints):
                startIndex = len(refPoints) - 1
            elif startIndex == -1:
                startIndex = 0
            return startIndex
        if endIndex == len(refPoints):
            endIndex = len(refPoints) - 1
        elif endIndex == -1:
            endIndex = 0
        return endIndex

    def calOffsetWithThreshold(self, refPoints: list, startIndex: int, endIndex: int, res: list, startIndex2: int,
                               a: float, b: float, c: float, d: float, offset: float, reverse: bool, left: bool,
                               threshold: float):
        flag = False
        size = endIndex - startIndex + 1
        start = startIndex
        step = 1
        theta = - math.pi / 2.0
        if reverse:
            start = endIndex
            step = -1
        if left:
            theta = -theta
        for i in range(0, size):
            p = refPoints[start]
            s = p.getS() - offset
            dis = a + b * s + c * s ** 2 + d * s ** 3
            if threshold > 0 and math.fabs(dis) < threshold:
                flag = True
            res[startIndex2].setS(p.getS())
            res[startIndex2].setTheta(p.getTheta())
            res[startIndex2].setX(p.getX() + dis * math.cos(p.getTheta() + theta))
            res[startIndex2].setY(p.getY() + dis * math.sin(p.getTheta() + theta))
            startIndex2 += 1
            start += step
        return flag

    def calOffsetWithOutThreshold(self, refPoints: list, startIndex: int, endIndex: int, res: list, startIndex2: int,
                                  a: float, b: float, c: float, d: float, offset: float, reverse: bool, left: bool):
        self.calOffsetWithThreshold(refPoints, startIndex, endIndex, res, startIndex2, a, b, c, d, offset, reverse,
                                    left, -1)

    def calOneSideLine(self, lr: LeftCenterRight, left: bool, refPoints: list, laneSectionStartS: float,
                       laneSectionEndS: float):
        laneNum = lr.getLaneSize()
        if laneNum == 0:
            return
        sign = -1
        if left:
            sign = -sign

        reverse = True
        noReverse = False
        start = True
        end = False
        startIndexOfLaneSection = self.findIndex(refPoints, laneSectionStartS, noReverse, start)
        endIndexOfLaneSection = self.findIndex(refPoints, laneSectionEndS, noReverse, end)
        # LaneSectionPoints是当前车道的计算时的参考车道，即上一条车道，初始为参考线
        laneSectionPoints = copy.deepcopy(refPoints[startIndexOfLaneSection:endIndexOfLaneSection + 1])
        centerLine = copy.deepcopy(laneSectionPoints)
        for i in range(1, laneNum + 1):
            lane = lr.getLane(sign * i)
            widths = lane.getWidths()
            widthNum = len(widths)

            offsetRatio = 0.5
            if i == 1:
                if left:
                    laneSectionPoints.reverse()
                    centerLine.reverse()
            # 设置左车道线
            lane.setLeftLine(copy.deepcopy(laneSectionPoints))

            # special处理特殊标识，如果当前车道与参考车道在某一处存在距离小于一个阈值，则为True，表示当前车道的中心线需要1特殊处理
            special = False
            normalWidth = 0.0
            offsetWidth = []
            for j in range(0, widthNum):
                width = widths[j]
                widthStartS = width.getSOffset() + laneSectionStartS
                if j == widthNum - 1:
                    widthEndS = laneSectionEndS
                else:
                    widthEndS = widths[j + 1].getSOffset() + laneSectionStartS
                if left:
                    if j == 0:
                        startIndex = self.findIndex(laneSectionPoints, widthEndS, reverse, start)
                        endIndex = self.findIndex(laneSectionPoints, widthStartS, reverse, end)
                    else:
                        endIndex = startIndex - 1
                        startIndex = self.findIndex(laneSectionPoints, widthEndS, reverse, start)
                else:
                    if j == 0:
                        startIndex = self.findIndex(laneSectionPoints, widthStartS, noReverse, start)
                        endIndex = self.findIndex(laneSectionPoints, widthEndS, noReverse, end)
                    else:
                        startIndex = endIndex + 1
                        endIndex = self.findIndex(laneSectionPoints, widthEndS, noReverse, end)
                normalWidth = lane.getNormalWidth()
                a = width.getA()
                b = width.getB()
                c = width.getC()
                d = width.getD()
                self.calOffsetWithOutThreshold(laneSectionPoints, startIndex, endIndex, centerLine, startIndex,
                                               offsetRatio * a, offsetRatio * b, offsetRatio * c, offsetRatio * d,
                                               widthStartS, noReverse, left)
                self.calOffsetWithThreshold(laneSectionPoints, startIndex, endIndex, laneSectionPoints,
                                            startIndex, a, b, c, d, widthStartS, noReverse, left, 0.5)
            lane.setRightLine(copy.deepcopy(laneSectionPoints))
            lane.setCenterLine(copy.deepcopy(centerLine))
            self.format(lane.getUniqueId())

    def calLaneLineAndCenterLine(self):
        file = open('geometry.txt', mode='w')
        for road in self.roads.values():
            lanes = road.getLanes()
            refPoints = []
            geos = road.getPlanView().getGeometries()
            for geo in geos:
                ps = geo.getPoints()
                refPoints.extend(ps)

            laneOffsets = lanes.getLaneOffsets()
            # 偏移参考线的点
            if len(laneOffsets) != 0:
                laneOffsetSize = len(laneOffsets)
                for i in range(0, laneOffsetSize):
                    startS = laneOffsets[i].getS()
                    if i == laneOffsetSize - 1:
                        endS = road.getLength()
                    else:
                        endS = laneOffsets[i + 1].getS()
                    start = True
                    end = False
                    noReverse = False
                    toLeft = True
                    if i == 0:
                        startIndex = self.findIndex(refPoints, startS, noReverse, start)
                        endIndex = self.findIndex(refPoints, endS, noReverse, end)
                    else:
                        startIndex = endIndex + 1
                        endIndex = self.findIndex(refPoints, endS, noReverse, end)

                    a = laneOffsets[i].getA()
                    b = laneOffsets[i].getB()
                    c = laneOffsets[i].getC()
                    d = laneOffsets[i].getD()
                    self.calOffsetWithOutThreshold(refPoints, startIndex, endIndex, refPoints, startIndex, a, b, c, d,
                                                   startS, noReverse, toLeft)
            # 将偏离后的参考线输出
            refPointsSize = len(refPoints)
            for ii in range(0, refPointsSize):
                file.write(str(refPoints[ii].getX()) + ',' + str(refPoints[ii].getY()) + ',' + str(self.Z_) + '\n')

            # 计算车道线：
            laneSectionSize = lanes.getLaneSectionSize()
            for i in range(0, laneSectionSize):
                laneSection = lanes.getLaneSectionById(i)
                laneSectionS = laneSection.getS()
                left = laneSection.getLeft()
                right = laneSection.getRight()
                if i == laneSectionSize - 1:
                    laneSectionEndS = road.getLength()
                else:
                    laneSectionEndS = lanes.getLaneSectionById(i + 1).getS()
                self.calOneSideLine(left, True, refPoints, laneSectionS, laneSectionEndS)
                self.calOneSideLine(right, False, refPoints, laneSectionS, laneSectionEndS)
        file.close()

    def buildLaneLinkBetweenTwoLane(self, startRoadId: int, startLaneSectionId: int, startLaneId: int, endRoadId: int,
                                    endLaneSectionId: int, endLaneId: int) -> bool:
        if self.laneExist(startRoadId, startLaneSectionId, startLaneId) == False:
            print(
                '未找到构建链接的startlane:' + str(startRoadId) + ',' + str(startLaneSectionId) + ',' + str(startLaneId) + '\n')
            return False
        if self.laneExist(endRoadId, endLaneSectionId, endLaneId) == False:
            print('未找到构建链接的endlane:' + str(endRoadId) + ',' + str(endLaneSectionId) + ',' + str(endLaneId) + '\n')
            return False
        startLane = self.getLaneById(startRoadId, startLaneSectionId, startLaneId)
        endLane = self.getLaneById(endRoadId, endLaneSectionId, endLaneId)
        startId = startLane.getUniqueId()
        endId = endLane.getUniqueId()
        p = self.nodes[startId].next
        if p is None:
            self.nodes[startId].next = Edge(endId)
            return True
        while True:
            if p.id == endId:
                return True
            if p.next is None:
                break
            p = p.next
        p.next = Edge(endId)
        return True

    def buildLaneLinkBetweenTwoRoad(self, incomingRoadId: int, connectingRoadId: int, contactPoint: str, fromId: int,
                                    toId: int):
        # 按照车道lane的行驶方向定义出开始车道和结束车道所在的road编号和laneSection编号和车道lane编号
        if contactPoint == 'start':
            if fromId > 0 and toId > 0:
                startRoadId = connectingRoadId
                startLaneSectionId = 0
                startLaneId = toId

                endRoadId = incomingRoadId
                endLaneSectionId = len(self.getRoad(endRoadId).getLanes().getLaneSections()) - 1
                endLaneId = fromId
            elif fromId < 0 and toId < 0:
                startRoadId = incomingRoadId
                startLaneSectionId = len(self.getRoad(startRoadId).getLanes().getLaneSections()) - 1
                startLaneId = fromId

                endRoadId = connectingRoadId
                endLaneSectionId = 0
                endLaneId = toId
            elif fromId > 0 and toId < 0:
                startRoadId = incomingRoadId
                startLaneSectionId = 0
                startLaneId = fromId

                endRoadId = connectingRoadId
                endLaneSectionId = 0
                endLaneId = toId
            elif fromId < 0 and toId > 0:
                startRoadId = connectingRoadId
                startLaneSectionId = 0
                startLaneId = toId

                endRoadId = incomingRoadId
                endLaneSectionId = 0
                endLaneId = fromId
        else:
            if fromId > 0 and toId > 0:
                startRoadId = incomingRoadId
                startLaneSectionId = 0
                startLaneId = fromId

                endRoadId = connectingRoadId
                endLaneSectionId = len(self.getRoad(endRoadId).getLanes().getLaneSections()) - 1
                endLaneId = toId
            elif fromId < 0 and toId < 0:
                startRoadId = connectingRoadId
                startLaneSectionId = len(self.getRoad(startRoadId).getLanes().getLaneSections()) - 1
                startLaneId = toId

                endRoadId = incomingRoadId
                endLaneSectionId = 0
                endLaneId = fromId
            elif fromId > 0 and toId < 0:
                startRoadId = connectingRoadId
                startLaneSectionId = len(self.getRoad(startRoadId).getLanes().getLaneSections()) - 1
                startLaneId = toId

                endRoadId = incomingRoadId
                endLaneSectionId = len(self.getRoad(endRoadId).getLanes().getLaneSections()) - 1
                endLaneId = fromId
            elif fromId < 0 and toId > 0:
                startRoadId = incomingRoadId
                startLaneSectionId = len(self.getRoad(startRoadId).getLanes().getLaneSections()) - 1
                startLaneId = fromId

                endRoadId = connectingRoadId
                endLaneSectionId = len(self.getRoad(endRoadId).getLanes().getLaneSections()) - 1
                endLaneId = toId

        return self.buildLaneLinkBetweenTwoLane(startRoadId, startLaneSectionId, startLaneId, endRoadId,
                                                endLaneSectionId, endLaneId)

    def buildOneSideLink(self, roadId: int, laneSectionId: int, left: bool) -> bool:
        road = self.getRoad(roadId)
        roadPre = road.getRoadLink().getPredecessor()
        roadSuccessor = road.getRoadLink().getSuccessor()

        if roadPre is not None:
            roadLinkPreId = roadPre.getId()
            roadLinkPreType = roadPre.getType()
            roadLinkPreContactPoint = roadPre.getContactPoint()

        if roadSuccessor is not None:
            roadLinkSuccId = roadSuccessor.getId()
            roadLinkSuccType = roadSuccessor.getType()
            roadLinkSuccContactPoint = roadSuccessor.getContactPoint()

        lanes = {}
        if left:
            lanes = road.getLanes().getLaneSectionById(laneSectionId).getLeft().getLanes()
        else:
            lanes = road.getLanes().getLaneSectionById(laneSectionId).getRight().getLanes()
        for lane in lanes.values():
            link = lane.getLink()
            lanePre = None
            laneSucc = None
            if link is not None:
                lanePre = link.getPredecessor()
                laneSucc = link.getSuccessor()
            laneId = lane.getId()
            # 后继车道所在的laneSection编号
            laneSectionIdOfLaneSucc = -1
            # 后继车道所在的road编号
            RoadIdOfLaneSucc = -1
            # 前驱车道所在的laneSection编号
            laneSectionIdOfLanePre = -1
            # 前驱车道所在的road编号
            RoadIdOfLanePre = -1

            # 将每个车道作为节点创建邻接表，每个车道节点连接的边表示该车道按车道方向行驶直接相连的下一条车道
            if lanePre is not None:
                if laneSectionId == 0:
                    if roadPre is None:
                        print('roadId:' + str(roadId) + '没有找到前驱' + '\n')
                        return False
                    lanePreId = lanePre.getId()
                    if roadLinkPreType == 'road' and self.buildLaneLinkBetweenTwoRoad(roadId, roadLinkPreId,
                                                                                      roadLinkPreContactPoint, laneId,
                                                                                      lanePreId) == False:
                        return False
                else:
                    if left:
                        laneSuccId = lanePre.getId()
                        laneSectionIdOfLaneSucc = laneSectionId - 1
                        RoadIdOfLaneSucc = roadId
                        # 构建连接关系
                        if self.buildLaneLinkBetweenTwoLane(roadId, laneSectionId, laneId, RoadIdOfLaneSucc,
                                                            laneSectionIdOfLaneSucc, laneSuccId) == False:
                            return False
                    else:
                        lanePreId = lanePre.getId()
                        laneSectionIdOfLanePre = laneSectionId - 1
                        RoadIdOfLanePre = roadId

                        if self.buildLaneLinkBetweenTwoLane(RoadIdOfLanePre, laneSectionIdOfLanePre, lanePreId, roadId,
                                                            laneSectionId, laneId) == False:
                            return False
            if laneSucc is not None:
                if laneSectionId == len(self.getRoad(roadId).getLanes().getLaneSections()) - 1:
                    if roadSuccessor is None:
                        print('road' + str(roadId) + '没有找到后驱' + '\n')
                        return False
                    laneSuccId = laneSucc.getId()
                    if roadLinkSuccType == 'road' and self.buildLaneLinkBetweenTwoRoad(roadId, roadLinkSuccId,
                                                                                       roadLinkSuccContactPoint, laneId,
                                                                                       laneSuccId) == False:
                        return False
                else:
                    if left:
                        lanePreId = laneSucc.getId()
                        laneSectionIdOfLanePre = laneSectionId + 1
                        RoadIdOfLanePre = roadId

                        if self.buildLaneLinkBetweenTwoLane(RoadIdOfLanePre, laneSectionIdOfLanePre, lanePreId, roadId,
                                                            laneSectionId, laneId) == False:
                            return False
                    else:
                        laneSuccId = laneSucc.getId()
                        laneSectionIdOfLaneSucc = laneSectionId + 1
                        RoadIdOfLaneSucc = roadId
                        if self.buildLaneLinkBetweenTwoLane(roadId, laneSectionId, laneId, RoadIdOfLaneSucc,
                                                            laneSectionIdOfLaneSucc, laneSuccId) == False:
                            return False
        return True

    def buildMap(self):
        if self.validState == False:
            return False
        for road in self.roads.values():
            roadId = road.getId()
            junctionId = road.getJunctionId()
            laneSections = road.getLanes().getLaneSections()
            for laneSection in laneSections:
                left = True
                right = False
                if self.buildOneSideLink(roadId, laneSection.getId(), left) == False or self.buildOneSideLink(roadId,
                                                                                                              laneSection.getId(),
                                                                                                              right) == False:
                    return False

        for junction in self.junctions.values():
            connections = junction.getConnections()

            for connection in connections:
                incomingRoad = connection.getIncomingRoad()
                connectingRoad = connection.getConnectionRoad()
                contactPoint = connection.getContactPoint()
                laneLinks = connection.getLaneLinks()

                for laneLink in laneLinks:
                    fromId = laneLink.getFrom()
                    toId = laneLink.getTo()
                    if self.buildLaneLinkBetweenTwoRoad(incomingRoad, connectingRoad, contactPoint, fromId,
                                                        toId) == False:
                        return False
        return True

    def print(self):
        print('打印:' + str(len(self.roads)) + '\n')
        for i in range(0, len(self.nodes)):
            lane = self.getLaneById(self.nodes[i].roadId, self.nodes[i].laneSectionId, self.nodes[i].laneId)
            if lane.getType() == 'driving':
                print(
                    str(lane.getUniqueId()) + '(' + str(self.nodes[i].roadId) + ',' + str(self.nodes[i].laneSectionId) +
                    ',' + str(self.nodes[i].laneId) + ',' + lane.getType() + ')')
                edge = self.nodes[lane.getUniqueId()].next
                while edge is not None:
                    print('->' + str(edge.id) + '(' + str(self.nodes[edge.id].roadId) + ',' + str(
                        self.nodes[edge.id].laneSectionId) + ',' + str(
                        self.nodes[edge.id].laneId) + ',' + self.getLaneTypeByUniqueId(edge.id) + ')')
                    edge = edge.next
                print('\n')

    def drawLeftRightPic(self):
        for i in range(0, len(self.nodes)):
            lane = self.getLaneByUniqueId(i)
            if lane.getType() == 'driving' or lane.getType() == 'parking':
                leftLanePoints = lane.getLeftLine()
                rightLanePoints = lane.getRightLine()
                leftPointX = []
                leftPointY = []
                rightPointX = []
                rightPointY = []

                for point in leftLanePoints:
                    leftPointX.append(point.getX())
                    leftPointY.append(point.getY())
                for point in rightLanePoints:
                    rightPointX.append(point.getX())
                    rightPointY.append(point.getY())

                plt.plot(leftPointX, leftPointY, color = 'black',lineWidth = 0.1)
                plt.plot(rightPointX, rightPointY, color = 'black',lineWidth = 0.1)

        #plt.savefig('plotLeftRight.png', dpi=500)

    def drawCenterPic(self):
        for i in range(0, len(self.nodes)):
                lane = self.getLaneByUniqueId(i)
                if lane.getType() == 'driving' or lane.getType() == 'parking':
                    centerLanePoints = lane.getCenterLine()
                    centerPointX = []
                    centerPointY = []

                    for point in centerLanePoints:
                        centerPointX.append(point.getX())
                        centerPointY.append(point.getY())

                    plt.plot(centerPointX, centerPointY, color = 'red',lineWidth=0.3)

        plt.savefig('plotCenter.png', dpi = 500)

    def drawSpecificLine(self, roadId:int, laneSectionId, laneId:int):
        lane = self.getLaneById(roadId, laneSectionId, laneId)
        pointX = []
        pointY = []
        for point in lane.getCenterLine():
            pointX.append(point.getX())
            pointY.append(point.getY())
        self.drawLeftRightPic()
        plt.plot(pointX, pointY, color = 'red', lineWidth = 0.2)
        plt.savefig('linePic.png', dpi =1000)

    def drawLeftRightCenter(self):
        for i in range(0, len(self.nodes)):
            lane = self.getLaneByUniqueId(i)
            if lane.getType() == 'driving':
                leftLanePoints = lane.getLeftLine()
                rightLanePoints = lane.getRightLine()
                centerLanePoints = lane.getCenterLine()
                leftPointX = []
                leftPointY = []
                rightPointX = []
                rightPointY = []
                centerPointX = []
                centerPointY = []

                for point in leftLanePoints:
                    leftPointX.append(point.getX())
                    leftPointY.append(point.getY())
                for point in rightLanePoints:
                    rightPointX.append(point.getX())
                    rightPointY.append(point.getY())
                for point in centerLanePoints:
                    centerPointX.append(point.getX())
                    centerPointY.append(point.getY())

                plt.plot(leftPointX, leftPointY, color = 'black',lineWidth=0.05)
                plt.plot(rightPointX, rightPointY, color = 'black',lineWidth=0.05)
                plt.plot(centerPointX, centerPointY, color = 'red',lineWidth=0.1)

        plt.savefig('plotCenterLeftRight.png', dpi=2000)

    def findPosition(self, x: float, y: float):
        mindis = -1
        for index, road in self.roads.items():
            laneSectionLength = road.getLanes().getLaneSectionSize()
            lanes = road.getLanes()
            for i in range(0, laneSectionLength):
                laneSection = lanes.getLaneSectionById(i)
                left = laneSection.getLeft()
                right = laneSection.getRight()
                laneSize = left.getLaneSize()
                for j in range(1, laneSize + 1):
                    lane = left.getLane(j)
                    if lane.getType() == 'driving':
                        centerLine = lane.getCenterLine()
                        centerLineSize = len(centerLine)
                        for k in range(0, centerLineSize):
                            xx = x - centerLine[k].getX()
                            yy = y - centerLine[k].getY()
                            dis = xx * xx + yy * yy
                            if mindis == -1 or dis < mindis:
                                x1 = centerLine[k].getX()
                                y1 = centerLine[k].getY()
                                roadId = index
                                laneSectionId = i
                                laneId = j
                                offset = centerLine[0].getS() - centerLine[k].getS()
                                mindis = dis

                laneSize = right.getLaneSize()
                for j in range(1, laneSize + 1):
                    lane = right.getLane(-j)
                    if lane.getType() == 'driving':
                        centerLine = lane.getCenterLine()
                        centerLineSize = len(centerLine)
                        for k in range(0, centerLineSize):
                            xx = x - centerLine[k].getX()
                            yy = y - centerLine[k].getY()
                            dis = xx * xx + yy * yy
                            if mindis == -1 or dis < mindis:
                                x1 = centerLine[k].getX()
                                y1 = centerLine[k].getY()
                                roadId = index
                                laneSectionId = i
                                laneId = -j
                                offset = centerLine[k].getS() - centerLine[0].getS()
                                mindis = dis
        # print(str(x1) + ',' + str(y1) + '\n')
        print(str(roadId) + ',' + str(laneSectionId) + ',' + str(laneId) + ',' + str(offset))
        return Position(roadId, laneSectionId, laneId, offset)

    def AstarPlan(self, start: Position, end: Position, append: bool, turnRound: bool):
        open = queue.PriorityQueue()
        vertaxs = []
        for i in range(0, self.laneNum):
            vertax = Vertax()
            vertax.id = i
            vertax.parent = -1
            vertax.closed = 0
            # INFDIS表示无限大，此路不通
            vertax.cost = 1000000
            vertaxs.append(vertax)
        startLane = self.getLaneById(start.roadId, start.laneSectionId, start.laneId)
        endLane = self.getLaneById(end.roadId, end.laneSectionId, end.laneId)
        # 换道的三次样条索引相差的值
        laneChangeIndexInterval = 40
        if startLane is None or endLane is None:
            print("起始路径不存在")
            return False
        # 特殊情况：当起点和终点在同一条道路节点时，并且终点在前起点在后，此时只能转一圈达到终点
        # 每次从open表中移出一次目的节点，remainRemovalTimes减少一，如果remainRemovalTimes并且达到目标节点，说明找到路径。默认为一，特殊情况下为二
        remainRemovalTimes = 1
        if startLane is endLane and start.offset > end.offset:
            remainRemovalTimes = 2
        startId = startLane.getUniqueId()
        endId = endLane.getUniqueId()
        vertaxs[startId].cost = 0
        vertaxs[startId].startOffset = start.offset
        open.put(vertaxs[startId])
        while not open.empty():
            # 从open表中弹出最小路径代价的节点
            minCostNode = open.get()
            minLane = self.getLaneById(self.nodes[minCostNode.id].roadId, self.nodes[minCostNode.id].laneSectionId,
                                       self.nodes[minCostNode.id].laneId)
            # 如果最小代价的点已经无穷大，则不必要搜索
            if vertaxs[minCostNode.id].cost >= 1000000:
                break
            # 将此节点closed置1表示从start节点到该节点的最短路径已经找到
            vertaxs[minCostNode.id].closed = 1
            # 如果达到目的地节点，这需要判断
            if minCostNode.id == endId:
                remainRemovalTimes -= 1
                if remainRemovalTimes == 0:
                    break
                else:
                    vertaxs[minCostNode.id].closed = 0
            # cost表示从起点到当前道路的startoffset的代价
            minCosLaneLength = self.getLaneLengthByUniqueId(minCostNode.id)
            weight = minCosLaneLength - vertaxs[minCostNode.id].startOffset
            # minCostNode节点的所有后继节点
            neiborNode = self.nodes[minCostNode.id].next
            while neiborNode is not None:
                # 从open表中弹出最小路径代价的节点
                neiborNodeId = neiborNode.id
                neiborLane = self.getLaneById(self.nodes[neiborNodeId].roadId, self.nodes[neiborNodeId].laneSectionId,
                                              self.nodes[neiborNodeId].laneId)
                # 判断neiborLane路段是否可行：如果minLane有路障，并且沿着minLane的行驶方向会有invalidPosition(路障位置)，则neiborLane不可行
                neiborLaneValid = True
                invalidPositionSize = len(self.invalidPosition)
                for i in range(0, invalidPositionSize):
                    invalidId = self.getLaneUniqueId(self.invalidPosition[i].roadId,
                                                     self.invalidPosition[i].laneSectionId,
                                                     self.invalidPosition[i].laneId)
                    if invalidId == minCostNode.id:
                        offsetInMinLane = self.invalidPosition[i].offset
                        carOffsetInMinLane = minCostNode.startOffset
                        if carOffsetInMinLane <= offsetInMinLane:
                            neiborLaneValid = False

                # 如果没有被路障挡住行驶方向&&该节点是driving && 该节点最短路径未找到 && 该节点的前驱节点不是minCostNode
                if neiborLaneValid and neiborLane.getType() == 'driving' and vertaxs[neiborNodeId].closed == 0 and \
                        vertaxs[neiborNodeId].parent != minCostNode.id:
                    # 如果是特殊情况或代价更短
                    if vertaxs[neiborNodeId].cost == 0 or vertaxs[neiborNodeId].cost > vertaxs[minCostNode.id].cost + weight:
                        # 换道标志
                        vertaxs[neiborNodeId].flag = 0
                        # 如果没换道则是从起点开始
                        vertaxs[neiborNodeId].startOffset = 0
                        # 更新代价
                        vertaxs[neiborNodeId].cost = vertaxs[minCostNode.id].cost + weight
                        # 如果该节点没有放在open表中，放入open表
                        if vertaxs[neiborNodeId].parent == -1:
                            open.put(vertaxs[neiborNodeId])
                        # 更新父节点
                        vertaxs[neiborNodeId].parent = minCostNode.id
                neiborNode = neiborNode.next

            # 下面是换道和掉头的考虑
            laneId = minLane.getId()
            laneChange = minLane.getLaneChangeFlag()
            laneChangeSize = len(laneChange)
            # s[0]表示左侧车道的车道编号，s[1]表示右侧车道的车道编号
            s = [0, 0]
            # indexOfLane表示换道位置在当前车道点的位置下标
            # 如果位于参考线右侧车道
            if laneId < 0:
                # 如果该车道是最左边车道，则该车道左车道为逆向车道
                if laneId == -1:
                    s[0] = self.getLaneUniqueId(self.nodes[minCostNode.id].roadId,
                                                self.nodes[minCostNode.id].laneSectionId, 1)
                else:
                    s[0] = self.getLaneUniqueId(self.nodes[minCostNode.id].roadId,
                                                self.nodes[minCostNode.id].laneSectionId,
                                                self.nodes[minCostNode.id].laneId + 1)
                s[1] = self.getLaneUniqueId(self.nodes[minCostNode.id].roadId,
                                            self.nodes[minCostNode.id].laneSectionId,
                                            self.nodes[minCostNode.id].laneId - 1)
            # 如果位于参考线左侧车道
            if laneId > 0:
                # 如果该车道是最左边车道，则该车道左车道为逆向车道
                if laneId == 1:
                    s[0] = self.getLaneUniqueId(self.nodes[minCostNode.id].roadId,
                                                self.nodes[minCostNode.id].laneSectionId, -1)
                else:
                    s[0] = self.getLaneUniqueId(self.nodes[minCostNode.id].roadId,
                                                self.nodes[minCostNode.id].laneSectionId,
                                                self.nodes[minCostNode.id].laneId - 1)
                s[1] = self.getLaneUniqueId(self.nodes[minCostNode.id].roadId,
                                            self.nodes[minCostNode.id].laneSectionId,
                                            self.nodes[minCostNode.id].laneId + 1)

            indexOfLane = int(laneChangeSize * vertaxs[minCostNode.id].startOffset / minCosLaneLength)
            indexOfLane = min(int(laneChangeSize - 1), indexOfLane)
            laneChangeFlag = laneChange[indexOfLane]
            # 定义越道代价，直行到可以换道的距离代价
            crossLaneCost = 0.0
            straightCost = 0.0
            turnRoundFlag = False
            laneChangeOffset = 0.0
            # 开始换道的索引（centerLine中）
            laneChangeIndexStart = 0
            for i in range(0, 2):
                crossLaneCost = 5
                turnRoundFlag = False
                laneChangeIndexStart = indexOfLane
                # 如果该节点编号有效&&如果是可行道路&&该节点最短路径未找到&&该节点的前驱节点不是minCostNode
                if s[i] != -1 and self.getLaneTypeByUniqueId(s[i]) == 'driving' and vertaxs[s[i]].closed == 0 and \
                        vertaxs[s[i]].parent != minCostNode.id:
                    tempLane = self.getLaneByUniqueId(s[i])
                    tempLaneWidths = tempLane.getLaneWidth()
                    # 如果要换道相反方向的道路上（掉头）&&不允许掉头，则不掉头换道
                    if self.nodes[s[i]].laneId * laneId < 0 and turnRound is False:
                        continue
                    # 如果要换道相反方向的道路上（掉头）&&允许掉头，则可以掉头换道
                    if self.nodes[s[i]].laneId * laneId < 0 and turnRound is True:
                        turnRoundFlag = True
                        crossLaneCost *= 20
                        laneChangeIndexStart = laneChangeSize - 1
                    # 往左掉头或者换道
                    if i == 0:
                        # 一直搜索到可以往左换道的标志出现 并且左道宽度不为0
                        while laneChangeIndexStart < laneChangeSize and (
                                laneChange[laneChangeIndexStart] != LaneChangeLeft and
                                laneChange[laneChangeIndexStart] != LaneChangeBoth or
                                tempLaneWidths[laneChangeIndexStart] < 0.1):
                            laneChangeIndexStart += 1
                    # 向右换道（向右应该没有掉头）
                    elif i == 1:
                        # 一直搜索到可以往右换道的标志出现 并且右道宽度不为0
                        while laneChangeIndexStart < laneChangeSize and (
                                laneChange[laneChangeIndexStart] != LaneChangeRight and
                                laneChange[laneChangeIndexStart] != LaneChangeBoth or
                                tempLaneWidths[laneChangeIndexStart] < 0.1):
                            laneChangeIndexStart += 1
                    # 不允许掉头或者换道
                    if laneChangeIndexStart >= laneChangeSize:
                        continue
                    # 继续向前索引n个
                    laneChangeIndexStart = min(laneChangeSize, int(laneChangeIndexStart + laneChangeIndexInterval))
                    # 在当前车道沿车道行驶方向的偏移，如果是换道则不变
                    laneChangeOffset = float(laneChangeIndexStart) / laneChangeSize * minCosLaneLength
                    straightCost = laneChangeOffset - vertaxs[minCostNode.id].startOffset
                    # 表示掉头
                    if self.nodes[s[i]].laneId * laneId < 0:
                        laneChangeOffset = minCosLaneLength - laneChangeOffset
                    # 如果代价更短
                    if vertaxs[s[i]].cost > vertaxs[minCostNode.id].cost + crossLaneCost + straightCost:
                        # 更新代价
                        vertaxs[s[i]].cost = vertaxs[minCostNode.id].cost + crossLaneCost + straightCost
                        # 换道标志1,掉头标志2
                        if turnRoundFlag:
                            vertaxs[s[i]].flag = 2
                        else:
                            vertaxs[s[i]].flag = 1
                        # 如果该节点没有在open表中，放入open表
                        if vertaxs[s[i]].parent == -1:
                            open.put(vertaxs[s[i]])
                        # 更新父节点
                        vertaxs[s[i]].parent = minCostNode.id
                        vertaxs[s[i]].startOffset = laneChangeOffset

        # 如果未找到最短路径，则退出
        if vertaxs[endId].closed == 0:
            print("can not reach")
            return False
        # 保存搜索的路径编号，从终点到起点
        uniqueLaneIdFromEndToStart = [endId]
        print('(' + str(end.roadId) + ',' + str(end.laneSectionId) + ',' + str(end.laneId) + ')')
        i = vertaxs[endId].parent
        while i != -1:
            print('<--(' + str(self.nodes[vertaxs[i].id].roadId) + ',' + str(
                self.nodes[vertaxs[i].id].laneSectionId) + ',' + str(self.nodes[vertaxs[i].id].laneId) + ')')
            uniqueLaneIdFromEndToStart.append(vertaxs[i].id)
            if vertaxs[i].id == endId:
                break
            i = vertaxs[i].parent

        # 输出路径点
        pathSize = len(uniqueLaneIdFromEndToStart)
        #####
        ####
        ### 这里面写输出路径点的信息
        pathX = []
        pathY = []
        ##
        #
        total = 0
        preX = 0.0
        preY = 0.0
        for i in range(pathSize - 1, -1, -1):
            turnRoundFlag = False
            laneChangeStartIndex = -1
            laneChangeEndIndex = -1
            uniqueId = uniqueLaneIdFromEndToStart[i]
            lane = self.getLaneByUniqueId(uniqueId)
            leftLane = None
            rightLane = None
            roadId = lane.getRoadId()
            laneSectionId = lane.getLaneSectionId()
            laneId = lane.getId()
            if lane.getId() > 0:
                if self.laneExist(roadId, laneSectionId, laneId - 1):
                    leftLane = self.getLaneById(roadId, laneSectionId, laneId - 1)
                if self.laneExist(roadId, laneSectionId, laneId + 1):
                    rightLane = self.getLaneById(roadId, laneSectionId, laneId + 1)
            else:
                if self.laneExist(roadId, laneSectionId, laneId + 1):
                    leftLane = self.getLaneById(roadId, laneSectionId, laneId + 1)
                if self.laneExist(roadId, laneSectionId, laneId - 1):
                    rightLane = self.getLaneById(roadId, laneSectionId, laneId - 1)
            laneLength = self.getLaneLengthByUniqueId(uniqueId)
            # 下面的有的变量会改变 得用deepcopy
            centerLine = copy.deepcopy(lane.getCenterLine())
            laneWidth = lane.getLaneWidth()
            laneChangeFlag = lane.getLaneChangeFlag()
            if leftLane is not None:
                leftLaneWidth = leftLane.getLaneWidth()
            if rightLane is not None:
                rightLaneWidth = rightLane.getLaneWidth()
            startIndex = 0
            endIndex = len(centerLine)
            centerLineSize = endIndex
            # 起点道路必须分开讨论，因为当终点和起点在同一条Lane的时候，startOffset会被覆盖
            if i == pathSize - 1:
                startIndex = math.ceil(start.offset / laneLength * centerLineSize)
            else:
                startIndex = math.ceil(vertaxs[uniqueId].startOffset / laneLength * centerLineSize)
            # 终点道路
            if i == 0:
                endIndex = int(end.offset / laneLength * centerLineSize)
            # 如果下一条道路是由当前道路换道过来的，换道只在同一个laneSection中换道，因此道路长度相同，点的个数也相同
            elif vertaxs[uniqueLaneIdFromEndToStart[i - 1]].flag == 1:
                # 换道过来的，这需要三次样条拟合换道轨迹
                # 如果连续换道，直接找到最后一条道
                k = i - 1
                while k >= 0 and vertaxs[uniqueLaneIdFromEndToStart[k]].flag == 1:
                    k -= 1
                k += 1
                # 连续换道，将道路编号置位，这里i改变了，注意下，下面掉头用到i了，但掉头和换道只会发生一个，因此不会影响
                i = k + 1
                endIndex = int(vertaxs[uniqueLaneIdFromEndToStart[k]].startOffset / laneLength * centerLineSize)
                startPoint = centerLine[max(0, int(endIndex - laneChangeIndexInterval))]
                endPoint = self.getLaneByUniqueId(uniqueLaneIdFromEndToStart[k]).getCenterLine()[
                    min(centerLineSize - 1, endIndex)]
                cubicLine = CubicLine(startPoint, endPoint)
                temp = cubicLine.calPoints()
                endIndex = max(0, int(endIndex - laneChangeIndexInterval))
                laneChangeStartIndex = endIndex + 1
                size = len(temp)
                for j in range(0, size):
                    if centerLineSize <= endIndex:
                        centerLine.append(temp[j])
                    else:
                        centerLine[endIndex] = temp[j]
                        endIndex += 1

                laneChangeEndIndex = endIndex
            # 如果下一条道路是由当前道路掉头过来的
            elif vertaxs[uniqueLaneIdFromEndToStart[i - 1]].flag == 2:
                endIndex = int((laneLength - vertaxs[
                    uniqueLaneIdFromEndToStart[i - 1]].startOffset) / laneLength * centerLineSize)
                #endIndex = centerLineSize
                turnRoundFlag = True
            lfWidth = 0.0
            rgWidth = 0.0
            midWidth = 0.0
            speed = 10.0
            laneChangef = LaneChangeNone
            intervalDis = 0.3
            for j in range(startIndex, endIndex):
                Area = 0
                if endIndex - j < 10 and turnRoundFlag:
                    Area = 2
                # 第一个点||当前点距离前一个输出点距离不太近
                if total == 0 or not ((math.pow(centerLine[j].getX() - preX, 2) + math.pow(centerLine[j].getY() - preY,
                                                                                           2)) < intervalDis):
                    roadType = 0
                    lfWidth = 0.0
                    rgWidth = 0.0
                    midWidth = 0.0
                    laneChangef = LaneChangeNone
                    # 三次样条拟合的点给个固定宽度
                    if j >= laneChangeStartIndex and j < laneChangeEndIndex:
                        midWidth = 3.65
                    else:
                        midWidth = laneWidth[j]
                        laneChangef = laneChangeFlag[j]
                        if leftLane is not None and leftLane.getType() == 'driving':
                            lfWidth = leftLaneWidth[j]
                        if rightLane is not None and rightLane.getType() == 'driving':
                            rgWidth = rightLaneWidth[j]
                    # 如果不能往左换道，则左道宽度为0
                    if laneChangef == LaneChangeNone or laneChangef == LaneChangeRight:
                        lfWidth = 0.0
                    # 可以往左换道但是左道为0
                    elif (laneChangef == LaneChangeBoth or laneChangef == LaneChangeLeft) and lfWidth < 0.1:
                        lfWidth = 3.5
                    # 如果不能往右换道，则右道为0
                    if laneChangef == LaneChangeNone or laneChangef == LaneChangeLeft:
                        rgWidth = 0.0
                    elif (laneChangef == LaneChangeBoth or laneChangef == LaneChangeRight) and rgWidth < 0.1:
                        rgWidth = 3.5
                    pathX.append(centerLine[j].getX())
                    pathY.append(centerLine[j].getY())
                    preX = centerLine[j].getX()
                    preY = centerLine[j].getY()
                    total += 1
            if turnRoundFlag:
                sp = Point(0, 0, 0, 0)
                startLane = self.getLaneByUniqueId(uniqueLaneIdFromEndToStart[i])
                startLaneCenterLine = startLane.getCenterLine()
                if endIndex - 1 >= 0:
                    sp = startLaneCenterLine[endIndex - 1]
                else:
                    sp = startLaneCenterLine[startIndex]
                # 同一laneSection的道路长度都一样，中心线点的个数也一样，掉头和换道都是在一个laneSection
                nextLane = self.getLaneByUniqueId(uniqueLaneIdFromEndToStart[i - 1])
                nextCenterLine = nextLane.getCenterLine()
                e = nextCenterLine[centerLineSize - endIndex]
                r = 0.5 * math.sqrt(math.pow(sp.getX() - e.getX(), 2) + math.pow(sp.getY() - e.getY(), 2))
                arcLength = math.pi * r
                m = Arc(sp.getS(), sp, sp.getTheta(), arcLength, 1.0 / r)
                m.calPosition(0.5)  # stepLength
                ps = m.getPoints()
                for j in range(0, len(ps)):
                    if total == 0 or not((math.pow(ps[j].getX(), 2) + math.pow(ps[j].getY() - preY, 2)) < intervalDis):
                        pathX.append(ps[j].getX())
                        pathY.append(ps[j].getY())
        self.drawLeftRightPic()
        plt.plot(pathX, pathY, color = 'red', lineWidth = 0.2)
        plt.savefig('path.png', dpi = 1000)

    def findPath(self, x1:float, y1:float, x2:float, y2:float, append:bool, turnRound:bool):
        if self.validState == False:
            print("OpenDrive未初始化")
            return False

        start = self.findPosition(x1, y1)
        end = self.findPosition(x2, y2)

        return self.AstarPlan(start, end, append, turnRound)

