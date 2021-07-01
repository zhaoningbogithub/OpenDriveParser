from .getDoc import *

class Header:
    def __init__(self, node_header):
        self.date = get_string(node_header, 'date')
        self.name = get_string(node_header, 'name')
        self.east = get_float(node_header, 'east')
        self.north = get_float(node_header, 'north')
        self.south = get_float(node_header, 'south')
        self.west = get_float(node_header, 'west')
        self.revMajor = get_int(node_header, 'revMajor')
        self.revMinor = get_int(node_header, 'revMinor')
        self.vendor = get_string(node_header, 'vendor')
        self.version = get_string(node_header, 'version')

        if node_header.find("geoReference") is not None:
            self.geo_reference = node_header.find("geoReference").text