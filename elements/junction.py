from .getDoc import *

class JunctionLaneLink:
    def __init__(self, node_laneLink):
        self.from_lane = get_int(node_laneLink, 'from')
        self.to_lane = get_int(node_laneLink, 'to')

    def getFrom(self) -> int:
        return self.from_lane

    def getTo(self) -> int:
        return self.to_lane


class JunctionConnection:
    def __init__(self, node_connection):
        self.id = get_int(node_connection, 'id')
        self.incomingRoad = get_int(node_connection, 'incomingRoad')
        self.connectingRoad = get_int(node_connection, 'connectingRoad')
        self.contactPoint = get_string(node_connection, 'contactPoint')
        self.laneLinks = []

        for node_laneLink in node_connection.findall('laneLink'):
            laneLink = JunctionLaneLink(node_laneLink)
            self.laneLinks.append(laneLink)

    def getId(self) -> int:
        return self.id

    def getIncomingRoad(self) -> int:
        return self.incomingRoad

    def getConnectionRoad(self) -> int:
        return self.connectingRoad

    def getContactPoint(self) -> str:
        return self.contactPoint

    def getLaneLinks(self) -> list:
        return self.laneLinks


class Junction:
    def __init__(self, node_junction):
        self.id = get_int(node_junction, 'id')
        self.name = get_string(node_junction, 'name')
        self.connections = []

        for node_connection in node_junction.findall('connection'):
            connection = JunctionConnection(node_connection)
            self.connections.append(connection)

    def getId(self) -> int:
        return self.id

    def getName(self) -> str:
        return self.name

    def getConnections(self) -> list:
        return self.connections
