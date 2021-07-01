from .getDoc import *
from .point import Point
from .getCal import *
from enum import Enum

class GeometryType(Enum):
    unknown = 0
    Line = 1
    Arc = 2
    Poly3 = 3
    ParamPoly3 = 4


class Geometry:
    def __init__(self, node_geometry):
        # <geometry hdg="1.97298146738386" length="204.215477960887" s="0" x="-901.947546366" y="1610.90849739965">
        self.s = get_float(node_geometry, 's')
        self.x = get_float(node_geometry, 'x')
        self.y = get_float(node_geometry, 'y')
        self.hdg = get_float(node_geometry, 'hdg')
        self.heading = self.hdg
        self.length = get_float(node_geometry, 'length')
        self.geo_type = GeometryType.unknown
        self.startPosition = Point(self.x, self.y)
        self.points = []
        self.step = 0.5

    def getS(self) -> float:
        return self.s

    def getLength(self) -> float:
        return self.length

    def getHeading(self) -> float:
        return self.heading

    def getstartPosition(self) -> Point:
        return self.startPosition

    def getPoints(self) -> list:
        return self.points

    def addPoint(self, point:Point):
        self.points.append(point)

    def setPoints(self, points:list):
        self.points = points

class GeometryLine(Geometry):
    def __init__(self, node_geometry):
        super().__init__(node_geometry)
        self.geo_type = GeometryType.Line
        self.calPosition(self.step)

    def calPosition(self, step:float):
        x = 0.0
        y = 0.0
        sx = self.getstartPosition().getX()
        sy = self.getstartPosition().getY()
        length = self.getLength()
        dis = 0.0
        while dis < length:
            seg = calLineSegment(x, y, sx, sy, dis, self.getHeading())
            p = Point(seg[0], seg[1], self.getHeading(), self.getS() + dis)
            self.addPoint(p)
            dis = dis + step


class GeometryArc(Geometry):
    def __init__(self, node_geometry):
        super().__init__(node_geometry)
        self.geo_type = GeometryType.Arc
        self.curvature = None

        node_arc = node_geometry.find('arc')
        if node_arc is not None:
            self.curvature = get_float(node_arc, 'curvature')
        self.calPosition(self.step)

    def getCurvature(self) -> float:
        return self.curvature

    def calPosition(self, step:float):
        x = 0.0
        y = 0.0
        theta = self.getHeading()
        sx = self.getstartPosition().getX()
        sy = self.getstartPosition().getY()
        length = self.getLength()

        points = []
        dis = 0.0
        while dis < length:
            seg = calArcSegment(x, y, sx, sy, dis, theta, self.getCurvature())
            p = Point(seg[0], seg[1], theta, self.getS() + dis)
            points.append(p)
            dis = dis + step
        for i in range(1, len(points)):
            points[i].setTheta(math.atan2(points[i].getY() - points[i - 1].getY(),
                                          points[i].getX() - points[i - 1].getX()))
        self.setPoints(points)


class GeometryPoly3(Geometry):
    def __init__(self, node_geometry):
        super().__init__(node_geometry)
        self.geo_type = GeometryType.Poly3
        self.a = 0
        self.b = 0
        self.c = 0
        self.d = 0

        node_poly3 = node_geometry.find('poly3')
        if node_poly3 is not None:
            self.a = get_float(node_poly3, 'a')
            self.b = get_float(node_poly3, 'b')
            self.c = get_float(node_poly3, 'c')
            self.d = get_float(node_poly3, 'd')


class GeometryParamPoly3(Geometry):
    def __init__(self, node_geometry):
        super().__init__(node_geometry)
        self.geo_type = GeometryType.ParamPoly3
        self.aU = 0
        self.bU = 0
        self.cU = 0
        self.dU = 0
        self.aV = 0
        self.bV = 0
        self.cV = 0
        self.dV = 0
        self.pRange = 0

        node_param_poly3 = node_geometry.find('ParamPoly3')
        if node_param_poly3 is not None:
            self.aU = get_float(node_param_poly3, 'aU')
            self.bU = get_float(node_param_poly3, 'bU')
            self.cU = get_float(node_param_poly3, 'cU')
            self.dU = get_float(node_param_poly3, 'dU')
            self.aV = get_float(node_param_poly3, 'aV')
            self.bV = get_float(node_param_poly3, 'bV')
            self.cV = get_float(node_param_poly3, 'cV')
            self.dV = get_float(node_param_poly3, 'dV')
            self.pRange = get_float(node_param_poly3, 'pRange')
