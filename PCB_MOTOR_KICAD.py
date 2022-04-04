
import sys
import os
import pcbnew
import numpy as np

def CalculateAngle( vec1, vec2):
    scalarProduct = np.dot( vec1, vec2)
    #print("Scalar product " + (str)(scalarProduct))
    sizeVec1 = np.linalg.norm( vec1)
    #print("Size of the vector 1 " + (str)(sizeVec1))
    sizeVec2 = np.linalg.norm( vec2)
    #print("Size of the vector 2 " + (str)(sizeVec2))
    cosValue = scalarProduct / (sizeVec1 * sizeVec2)
    #print("Cos value " + (str)(cosValue))
    angleValue = np.arccos(cosValue)
    #print("Angle value " + (str)(angleValue))
    #print("Angle value deg " + (str)(angleValue * 180.0/ np.pi))
    return angleValue

class Arc:
    centerPoint = np.array([])
    startPoint = np.array([])
    endPoint = np.array([])
    diameter = 60
    trackWidth = 0.5
    angleDeg = 10
    def UpdateEndPoint( self, pcbObject):
        angle = self.angleDeg * np.pi / 180.0
        rot = np.array([[ np.cos(angle), -np.sin(angle)], [ np.sin(angle), np.cos(angle)]])
        v1 = np.array([self.startPoint[0] - self.centerPoint[0], self.startPoint[1] - self.centerPoint[1]])
        v2 = np.dot(rot, v1)
        self.endPoint = np.array([v2[0], v2[1]])
    
    def GenerateGeometryToPCB(self, pcbObject):
        # drawing the arc
        arc = pcbnew.DRAWSEGMENT( pcbObject)
        arc.SetShape(pcbnew.S_ARC)
        
        cx = self.centerPoint[0]
        cy = self.centerPoint[1]
        
        sx = self.startPoint[0]
        sy = self.startPoint[1]
        
        arc.SetCenter(pcbnew.wxPoint(pcbnew.FromMM( (float)(cx)), pcbnew.FromMM( (float)(cy))))
        arc.SetArcStart(pcbnew.wxPoint(pcbnew.FromMM( (float)(sx)), pcbnew.FromMM( (float)(sy))))
        arc.SetAngle(self.angleDeg * 10)
        arc.SetWidth( (int)(self.trackWidth * 1000000))
        arc.SetLayer(0)
        pcbObject.Add(arc)
    
class Line:
    startPoint = np.array([])
    endPoint = np.array([])
    trackWidth = 0.5
    def GenerateGeometryToPCB(self, pcbObject):
        line = pcbnew.DRAWSEGMENT( pcbObject)
        line.SetShape( pcbnew.S_SEGMENT)
                
        sx = self.startPoint[0]
        sy = self.startPoint[1]
        
        ex = self.endPoint[0]
        ey = self.endPoint[1]
        
        line.SetStart( pcbnew.wxPoint(pcbnew.FromMM( (float)(sx)), pcbnew.FromMM( (float)(sy))))
        line.SetEnd( pcbnew.wxPoint(pcbnew.FromMM( (float)(ex)), pcbnew.FromMM( (float)(ey))))
        line.SetWidth( (int)(self.trackWidth * 1000000))
        line.SetLayer(0)
        pcbObject.Add(line)

class MotorWinding:
    centerPoint = np.array([0.0, 0.0])
    
    pcbObject = None
    
    clearanceDistance = 0.5
    trackWidth = 0.25
    
    minDiameter = 20.0
    maxDiameter = 120.0
    meanDiameter = (maxDiameter + minDiameter) * 0.5
    
    numberOfSections = 3.0
    sectionAngle = (float)(360.0 / numberOfSections) * np.pi / 180.0
    semiSectionAngle = sectionAngle * 0.5
    semiSectionVec = np.array([])
    
    previousPoint = np.array([0.0, 0.0])
    movingPoint = np.array([0.0, 0.0])
    
    arcArrayUp = []
    arcArrayBottom = []
    lineArray = []
    
    minArcAngle = 5.0 * np.pi / 180.0
    
    def _GenerateSemisectionVector( self):
        l = 1
        x = l * np.cos( self.semiSectionAngle)
        y = l * np.sin( self.semiSectionAngle)
        self.semiSectionVec = np.array([ x, y])
        
    def _CalculateArcAngle( self, startPoint):
        baseVec = np.array([startPoint[0] - self.centerPoint[0], startPoint[1] - self.centerPoint[1]])
        arcAngle = CalculateAngle(baseVec, self.semiSectionVec) * 2.0
        return arcAngle
    
    def _CalculateArcLength( self, radius, arcAngle):
        length = radius * arcAngle
        return length
    
    def _CalculateCurrentRadius( self, point):
        rad = np.sqrt( np.square( point[0]) + np.square( point[1]))
        return rad
        

    def BuildArcArray( self):
        numberOfBottomSections = 0
        self._GenerateSemisectionVector()
        self.movingPoint[0] = self.centerPoint[0] + self.clearanceDistance
        self.movingPoint[1] = self.centerPoint[1] + self.minDiameter * 0.5 + self.clearanceDistance
        arcAngle = self._CalculateArcAngle( self.movingPoint)
        if (arcAngle > self.minArcAngle):
            arc = Arc()
            arc.trackWidth = self.trackWidth
            arc.centerPoint = self.centerPoint
            arc.startPoint = self.movingPoint
            arc.angleDeg = - arcAngle * 180.0 / np.pi
            arc.UpdateEndPoint(self.pcbObject)
            print("Adding arc with angle : " + (str)(arc.angleDeg))
            self.arcArrayBottom.append( arc)
            numberOfBottomSections = numberOfBottomSections + 1
        previuosRadius = 0
        currentRadius = self._CalculateCurrentRadius( self.movingPoint)
        while ( currentRadius < self.meanDiameter * 0.5):
            previuosRadius = currentRadius
            currentRadius = previuosRadius + self.clearanceDistance
            
            x = self.movingPoint[0] + self.clearanceDistance
            y = np.sqrt(np.square(currentRadius) - np.square(x))
            self.movingPoint = np.array( [ x, y])
            
            arcAngle = self._CalculateArcAngle( self.movingPoint)
            
            #currentRadius = self._CalculateCurrentRadius( self.movingPoint)
            if (arcAngle > self.minArcAngle):
                arc = Arc()
                arc.trackWidth = self.trackWidth
                arc.centerPoint = self.centerPoint
                arc.startPoint = self.movingPoint
                arc.angleDeg = - arcAngle * 180.0 / np.pi
                arc.UpdateEndPoint(self.pcbObject)
                print("Adding arc with angle : " + (str)(arc.angleDeg))
                self.arcArrayBottom.append( arc)
                numberOfBottomSections = numberOfBottomSections + 1
            else:
                break
        
        numberOfUpperSections = 0
        self.movingPoint[0] = self.centerPoint[0] + self.clearanceDistance
        self.movingPoint[1] = self.centerPoint[1] + self.maxDiameter * 0.5 + self.clearanceDistance
        arcAngle = self._CalculateArcAngle( self.movingPoint)
        if (arcAngle > self.minArcAngle):
            arc = Arc()
            arc.trackWidth = self.trackWidth
            arc.centerPoint = self.centerPoint
            arc.startPoint = self.movingPoint
            arc.angleDeg = - arcAngle * 180.0 / np.pi
            arc.UpdateEndPoint(self.pcbObject)
            print("Adding arc with angle : " + (str)(arc.angleDeg))
            self.arcArrayUp.append( arc)
            numberOfUpperSections = numberOfUpperSections + 1
        previuosRadius = 0
        currentRadius = self._CalculateCurrentRadius( self.movingPoint)
        while ( currentRadius > self.meanDiameter * 0.5):
            previuosRadius = currentRadius
            currentRadius = previuosRadius - self.clearanceDistance
            
            x = self.movingPoint[0] + self.clearanceDistance
            y = np.sqrt(np.square(currentRadius) - np.square(x))
            self.movingPoint = np.array( [ x, y])
            
            arcAngle = self._CalculateArcAngle( self.movingPoint)
            if (arcAngle > self.minArcAngle):
                arc = Arc()
                arc.trackWidth = self.trackWidth
                arc.centerPoint = self.centerPoint
                arc.startPoint = self.movingPoint
                arc.angleDeg = - arcAngle * 180.0 / np.pi
                arc.UpdateEndPoint(self.pcbObject)
                print("Adding arc with angle : " + (str)(arc.angleDeg))
                self.arcArrayUp.append( arc)
                numberOfUpperSections = numberOfUpperSections + 1
            else:
                break
            if (numberOfBottomSections == numberOfUpperSections):
                break
            
        print("Number of bottom sections : " + (str)(numberOfBottomSections))
        print("Number of upper sections : " + (str)(numberOfUpperSections))
        return True
    
    def BuildLineArcArray(self):
        for i in range( len( self.arcArrayBottom)):
            line = Line()
            arcBottom = self.arcArrayBottom[i]
            arcUp = self.arcArrayUp[i]
            line.startPoint = arcBottom.startPoint
            line.endPoint = arcUp.startPoint
            line.trackWidth = self.trackWidth
            self.lineArray.append( line)
        for i in range( len( self.arcArrayBottom) - 1):
            line = Line()
            arcBottom = self.arcArrayBottom[i + 1]
            arcUp = self.arcArrayUp[i]
            line.startPoint = arcBottom.endPoint
            line.endPoint = arcUp.endPoint
            line.trackWidth = self.trackWidth
            self.lineArray.append( line)
        return True
    
    def GenerateWindingGeometry(self):
        for arc in self.arcArrayUp:
            if (arc != None) and (self.pcbObject != None):
                arc.GenerateGeometryToPCB( self.pcbObject)
        for arc in self.arcArrayBottom:
            if (arc != None) and (self.pcbObject != None):
                arc.GenerateGeometryToPCB( self.pcbObject)
        for line in self.lineArray:
            if (line != None) and (self.pcbObject != None):
                line.GenerateGeometryToPCB( self.pcbObject)
        return True

def DrawNewLine( pcbObject, startX = 0, startY = 0, endX = 0, endY = 0, width = 1):
    line = pcbnew.DRAWSEGMENT( pcbObject)
    line.SetShape( pcbnew.S_SEGMENT)
    line.SetStart( pcbnew.wxPoint(    pcbnew.FromMM( startX), pcbnew.FromMM( startY)))
    line.SetEnd( pcbnew.wxPoint(    pcbnew.FromMM( endX), pcbnew.FromMM( endY)))
    line.SetWidth( (int)(width * 1000000))
    line.SetLayer(0)
    pcbObject.Add(line)

def DrawNewArc( pcbObject, cx = 0, cy = 0, startX = 0, startY = 0, angleDeg = 10, width = 1):
    # drawing the arc
    arc = pcbnew.DRAWSEGMENT( pcbObject)
    arc.SetShape(pcbnew.S_ARC)
    arc.SetCenter(pcbnew.wxPoint(pcbnew.FromMM( cx), pcbnew.FromMM( cy)))
    arc.SetArcStart(pcbnew.wxPoint(    pcbnew.FromMM( startX), pcbnew.FromMM( startY)))
    arc.SetAngle(angleDeg * 10)
    arc.SetWidth( (int)(width * 1000000))
    arc.SetLayer(0)
    pcbObject.Add(arc)


def GenerateWinding( pcbObject, cx = 0, cy = 0, minDiameter = 60, maxDiameter = 120, clearance = 0.5, width = 0.5, numOfWindings = 4):
    minRad = minDiameter * 0.5
    maxRad = maxDiameter * 0.5
    #section angle
    sectionAngle = 360 / numOfWindings
    #creating first line along radial direction
    DrawNewLine( pcbObject, cx, minRad + cy, cx, maxRad + cy, width)
    DrawNewArc( pcbObject, cx, cy, cx, maxRad + cy, sectionAngle, width)

def main(args):
    pcb = pcbnew.LoadBoard("noname.kicad_pcb")
    motorWinding = MotorWinding()
    motorWinding.pcbObject = pcb
    motorWinding.BuildArcArray()
    motorWinding.BuildLineArcArray()
    motorWinding.GenerateWindingGeometry()
    pcb.Save("mod1.kicad_pcb")
    print("Modified file saved")
    
    return 0
    
    

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
