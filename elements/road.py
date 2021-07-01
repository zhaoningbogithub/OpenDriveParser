from .getDoc import *
from .geometry import GeometryLine, GeometryArc, GeometryPoly3
from .lane import Lane


class PlanView:
    def __init__(self):
        self.geometries = []

    def setLine(self, line:GeometryLine):
        self.geometries.append(line)

    def setArc(self, arc:GeometryArc):
        self.geometries.append(arc)

    def setPoly(self, poly3:GeometryPoly3):
        self.geometries.append(poly3)

    def getGeometries(self) -> list:
        return self.geometries


class RoadLaneOffset:
    def __init__(self, node_laneOffset):
        # <laneOffset s="0" a="0" b="0" c="0" d="0" />
        self.s = get_float(node_laneOffset, 's')
        self.a = get_float(node_laneOffset, 'a')
        self.b = get_float(node_laneOffset, 'b')
        self.c = get_float(node_laneOffset, 'c')
        self.d = get_float(node_laneOffset, 'd')

    def getS(self):
        return self.s

    def getA(self):
        return self.a

    def getB(self):
        return self.b

    def getC(self):
        return self.c

    def getD(self):
        return self.d


class LeftCenterRight:
    def __init__(self, node_lanes):
        self.lanes = {}

        for node_lane in node_lanes.findall('lane'):
            lane = Lane(node_lane)
            self.lanes[lane.laneId] = lane

    def getLane(self, laneId:int):
        return self.lanes[laneId]

    def getLaneSize(self):
        return len(self.lanes)

    def getLanes(self):
        return self.lanes

    def laneExist(self, laneId:int) -> bool:
        return laneId in self.lanes


class LeftCenterRightEmpty:
    def __init__(self):
        self.lanes = {}

    def getLaneSize(self):
        return len(self.lanes)

    def getLanes(self):
        return self.lanes

    def laneExist(self, laneId:int) -> bool:
        return False


class RoadLaneSection:
    def __init__(self, node_laneSection):
        self.id = -1
        self.s = get_float(node_laneSection, 's')
        self.left_lanes = LeftCenterRightEmpty()
        self.center_lanes = LeftCenterRightEmpty()
        self.right_lanes = LeftCenterRightEmpty()

        node_left = node_laneSection.find('left')
        if node_left is not None:
            self.left_lanes = LeftCenterRight(node_left)

        node_center = node_laneSection.find('center')
        if node_center is not None:
            self.center_lanes = LeftCenterRight(node_center)

        node_right = node_laneSection.find('right')
        if node_right is not None:
            self.right_lanes = LeftCenterRight(node_right)

    def getLaneByLaneId(self, laneId:int) -> Lane:
        if laneId > 0:
            return self.left_lanes.getLane(laneId)
        elif laneId < 0:
            return self.right_lanes.getLane(laneId)
        return self.center_lanes.getLane(0)

    def laneExist(self, laneId:int) -> bool:
        if laneId > 0:
            return self.left_lanes.laneExist(laneId)
        elif laneId < 0:
            return self.right_lanes.laneExist(laneId)
        return False

    def getLeft(self) -> LeftCenterRight:
        if self.left_lanes is not None:
            return self.left_lanes

    def getCenter(self) -> LeftCenterRight:
        if self.center_lanes is not None:
            return self.center_lanes

    def getRight(self) -> LeftCenterRight:
        if self.right_lanes is not None:
            return self.right_lanes

    def getS(self) -> float:
        return self.s

    def getId(self) -> int:
        return self.id


class Lanes:
    def __init__(self, node_lanes):
        self.laneSections = []
        self.laneOffsets = []

        for node_laneOffset in node_lanes.findall('laneOffset'):
            laneOffset = RoadLaneOffset(node_laneOffset)
            self.laneOffsets.append(laneOffset)

        for node_laneSection in node_lanes.findall('laneSection'):
            laneSection = RoadLaneSection(node_laneSection)
            laneSection.id = len(self.laneSections)
            self.laneSections.append(laneSection)

    def getLaneSections(self) -> list:
        return self.laneSections

    def getLaneOffsets(self) -> list:
        return self.laneOffsets

    def laneSectionExist(self, laneSectionId:int) -> bool:
        if laneSectionId >= len(self.laneSections) or laneSectionId < 0:
            return False
        else:
            return True

    def laneExist(self, laneSectionId:int, laneId:int) -> bool:
        return self.laneSectionExist(laneSectionId) and self.laneSections[laneSectionId].laneExist(laneId)

    def getLaneSectionById(self, laneSectionId:int) -> RoadLaneSection:
        return self.laneSections[laneSectionId]

    def getLane(self, laneSectionId:int, laneId:int) -> Lane:
        return self.laneSections[laneSectionId].getLaneByLaneId(laneId)

    def getLaneSectionSize(self) -> int:
        return len(self.laneSections)

    def getLaneOffsetsSize(self) -> int:
        return len(self.laneOffsets)


class RoadObject:
    def __init__(self, node_object):
        # <object id="100101" type="vegetation" name="VegBush04.flt" s="0" t="1" zOffset="0" length="3" width="3" height="3" hdg="0.7055475" pitch="0" roll="0" />
        self.id = get_int(node_object, 'id')
        self.type = get_string(node_object, 'type')
        self.name = get_string(node_object, 'name')
        self.s = get_float(node_object, 's')
        self.t = get_float(node_object, 't')
        self.zOffset = get_float(node_object, 'zOffset')
        self.length = get_float(node_object, 'length')
        self.width = get_float(node_object, 'width')
        self.height = get_float(node_object, 'height')
        self.hdg = get_float(node_object, 'hdg')
        self.pitch = get_float(node_object, 'pitch')
        self.roll = get_float(node_object, 'roll')


class RoadLinkElement:
    def __init__(self, node_roadlink_element):
        # <predecessor contactPoint="end" elementId="91" elementType="junction" />
        self.contactPoint = get_string(node_roadlink_element, 'contactPoint')
        self.elementId = get_int(node_roadlink_element, 'elementId')
        self.elementType = get_string(node_roadlink_element, 'elementType')

    def getContactPoint(self) -> str:
        return self.contactPoint

    def getId(self) -> int:
        return self.elementId

    def getType(self) -> str:
        return self.elementType


class RoadLink:
    def __init__(self, node_roadlink):
        self.predecessor = None
        self.successor = None

        node_predecessor = node_roadlink.find('predecessor')
        if node_predecessor is not None:
            self.predecessor = RoadLinkElement(node_predecessor)

        node_successor = node_roadlink.find('successor')
        if node_successor is not None:
            self.successor = RoadLinkElement(node_successor)

    def getSuccessor(self) -> RoadLinkElement:
        return self.successor

    def getPredecessor(self) -> RoadLinkElement:
        return self.predecessor


class RoadType:
    def __init__(self, node_roadtype):
        # <type s="0" type="motorway" />
        self.s = get_float(node_roadtype, 's')
        self.type = get_string(node_roadtype, 'type')


class Road:
    def __init__(self, node_road):
        self.id = -1
        self.name = None
        self.junctionId = None
        self.length = None

        self.link = None
        self.type = None
        self.lanes = None
        self.planView = None
        self.objects = []

        self.id = get_int(node_road, 'id')
        self.name = get_string(node_road, 'name')
        self.junctionId = get_int(node_road, 'junction')
        self.length = get_float(node_road, 'length')

        node_link = node_road.find('link')
        if node_link is not None:
            self.link = RoadLink(node_link)

        node_type = node_road.find('type')
        if node_type is not None:
            self.type = RoadType(node_type)

        node_planview = node_road.find('planView')
        if node_planview is not None:
            self.planView = PlanView()
            for node_geometry in node_planview.findall('geometry'):
                geometry = None
                if node_geometry.find('line') is not None:
                    geometry = GeometryLine(node_geometry)
                    self.planView.setLine(geometry)
                elif node_geometry.find('arc') is not None:
                    geometry = GeometryArc(node_geometry)
                    self.planView.setArc(geometry)
                elif node_geometry.find('poly3') is not None:
                    geometry = GeometryPoly3(node_geometry)
                    self.planView.setPoly(geometry)

        node_lanes = node_road.find('lanes')
        if node_lanes is not None:
            self.lanes = Lanes(node_lanes)

        node_objects = node_road.find('objects')
        if node_objects is not None:
            for node_object in node_objects.findall('object'):
                road_object = RoadObject(node_object)
                self.objects.append(road_object)

    def getId(self) -> int:
        return self.id

    def getLanes(self) -> Lanes:
        return self.lanes

    def getPlanView(self) -> PlanView:
        return self.planView

    def getRoadLink(self) -> RoadLink:
        return self.link

    def getJunctionId(self) -> int:
        return self.junctionId

    def getLength(self) -> float:
        return self.length

    def getName(self) -> str:
        return self.name

    def laneExist(self, laneSectionId:int, laneId:int) -> bool:
        return self.lanes.laneExist(laneSectionId, laneId)

    def getLane(self, laneSectionId:int, laneId:int):
        return self.lanes.getLane(laneSectionId, laneId)