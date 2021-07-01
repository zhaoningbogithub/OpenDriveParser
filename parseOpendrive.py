# -*- coding=utf-8 -*-

from elements.openDrive import OpenDrive, Position

import os
from lxml import etree
from elements.point import Point

if __name__ == "__main__":

    xodr_file = r'yiqiqiming_9.xodr'

    if os.path.exists(xodr_file):
        doc = etree.parse(xodr_file)
        opendrive = OpenDrive(doc.getroot())
        print('parse ok')
        print(opendrive.buildMap())
        opendrive.print()

        # for i in range(0, len(opendrive.nodes)):
        #     opendrive.format(i)
        opendrive.calLaneLineAndCenterLine()
        # A = Position(4056, 0, -1, 35)
        # B = Position(64, 0, -1, 49)
        # opendrive.AstarPlan(A, B, True, True)

        opendrive.findPath(-679.259, -306.336, -545.177, 170.256, False, True)

        #opendrive.drawSpecificLine(4056,0,-1)
        #opendrive.drawLeftRightCenter()
        # for i in range(0, len(opendrive.nodes)):
        #     if i ==559 or i == 610:
        #         print(opendrive.getLaneLengthByUniqueId(i))



        #print(opendrive.findPosition(-505.360, -418.726))

    else:
        print(xodr_file, 'not found')
