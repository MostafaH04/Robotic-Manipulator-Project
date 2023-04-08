from math import pi, sin, cos

GRAVITY = 9.807 # meters per second square

class centerOfMass:
    def __init__(self, position, force) -> None:
        self.position = position                # coordinate (in meters)
        self.posX = position[0]                 # meters
        self.posY = position[1]                 # meters

        self.force = force                      # newtons

class Link:
    def __init__(self, startPos, length, angleDeg, geoConstant) -> None:
        self.length = length                    # meters
        self.angle = angleDeg                   # degrees
        self._rad = self._toRad(self.angle)      # radians

        self.start = startPos                   # coordinate (in meters)
        self.startX = startPos[0]               # meters
        self.startY = startPos[1]               # meters

        self.end = self._getEndPosition()       # coordinate (in meters)
        
        self._geoConstant = geoConstant         # kilogram per meter
        self.weight = self._getWeight()         # newtons 

        self.COM = self._getCenterOfMass()      # coordinate (in meters)

        pass
    
    
    def _toRad(self, angle):
        return angle * pi / 180

    def _getEndPosition(self) -> list:
        beamLen = self.length # length of the member
        ang = self._rad # angle in radians

        # sine of angle gets Y-component
        self.spanY = yComp = sin(ang) * beamLen
        
        # cosine gets X-component
        self.spanX = xComp = cos(ang) * beamLen

        self.endX = self.startX + xComp
        self.endY = self.startY + yComp

        return [self.endX, self.endY]

    def _getMass(self) -> float:
        return self._geoConstant * self.length

    def _getWeight(self) -> float:
        return GRAVITY * self._getMass()

    def _getCenterOfMass(self) -> centerOfMass:
        force = self.weight

        actingPosX = self.spanX/2 + self.startX
        actingPosY = self.spanY/2 +self.startY

        actingPos = [actingPosX, actingPosY]

        return centerOfMass(actingPos, force)
