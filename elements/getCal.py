import math

PI = 3.141592653589793
INFDIS = 1000000

def calLineSegment(endX:float, endY:float, startX:float, startY:float, length:float, heading:float):
    endX = startX + length * math.cos(heading)
    endY = startY + length * math.sin(heading)

    return (endX, endY)

def calArcSegment(endX:float, endY:float, startX:float, startY:float, length:float, heading:float, curvature:float):
    heading = heading - PI / 2.0

    a = 2.0 / curvature * math.sin(length * curvature / 2.0)
    alpha = (PI - length * curvature) / 2.0 - heading

    endX = -1 * a * math.cos(alpha) + startX
    endY = a * math.sin(alpha) + startY

    return (endX, endY)
