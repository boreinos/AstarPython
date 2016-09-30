#!/usr/bin/env python
import roslib
import re
import numpy
from numpy.linalg import inv
from matplotlib import pyplot
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Goal
#----------------------------------------------------------------------------

#----------------------------------------------------------------------------
# Global Functions:

# listener function:
def listener():
    rospy.init_node('goal_listener', anonymous=True)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rospy.spin()
#
def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
        
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))
#-----------------------------------------------------------------------------
#functions associated with drawing into map
def drawPath(path,image,radius,gridWorldSize):
    for node in path.path:
        nodeDiameter=radius * 2
        nodeRadius = radius
        xpoint = node.gridX*nodeDiameter+nodeRadius              
        ypoint = gridWorldSize[1] - (node.gridY*nodeDiameter+nodeRadius) 
        x1 = xpoint-nodeRadius
        x2 = xpoint+nodeRadius
        y1 = ypoint-nodeRadius
        y2 = ypoint+nodeRadius
        image[y1:y2,x1:x2] =100
    return image

def paintNode(node,radius, image):
    nodeRadius = radius
    xpoint = node.getX()
    ypoint = node.getY()
    x1 = xpoint-nodeRadius
    x2 = xpoint+nodeRadius
    y1 = ypoint-nodeRadius
    y2 = ypoint+nodeRadius
    image[y1:y2,x1:x2] =10
    return image
#-------------------------------------------------------------------------
# functions for mapping locations
# origin pose is a 2d tuple:
def offsetLocation(transformA, pose, resolution):
    #offset = originPose
    #currentPose = pose
    locationVector = numpy.dot(transformA,pose)/float(resolution)
    currentLocation = (locationVector.item(0),locationVector.item(1))
    return currentLocation
def getPoseFromLocation(transformA,location, resolution):
    #offset = originPose
    #desiredLocation = location
    inverseA = inv(transformA)
    vectorPose = numpy.dot(inverseA,location)*float(resolution)
    desiredPose= (vectorPose.item(0),vectorPose.item(1),vectorPose.item(2),vectorPose.item(3))
    return desiredPose
def getListOfGoals(path,transformA,resolution):
    listOfGoals=[]
    for node in path.path:
        location=(node.getX(),node.getY(),0,1)
        pose = getPoseFromLocation(transformA, location, resolution) 
        listOfGoals.append(pose)
    return listOfGoals
def mapToPGM(bottom,resolution,point,worldSize):
    newX = int((abs(bottom[0])+point[0])/resolution)
    newY = worldSize[1]-int(abs(bottom[1])/resolution)+int(point[1]/resolution)
    return (newX,newY)
def PGMToMap(bottom,resolution,point,worldSize):
    mapX = point[0]*resolution - bottom[0]
    mapY = 
#-------------------------------------------------------------------------
# functions associated with A*
def retracePath(startNode, endNode):
    currentNode = endNode
    path =[]
    while(currentNode != startNode):
        path.append(currentNode)
        currentNode = currentNode.parent
    path.reverse()
    return path

def getDistance(nodeA, nodeB):
    dstX = abs(nodeB.gridX-nodeA.gridX)
    dstY = abs(nodeB.gridY-nodeB.gridY)
    if(dstX > dstY):
        return (14*dstY + 10*(dstX-dstY))
    return (14*dstX + 10*(dstY-dstX))

def getNeighbors(node,grid,startNode,targetNode):
    neighbors = []
    for y in range(-1,2):
        for x in range(-1,2):
            if (x==0 and y == 0):
                continue
            checkY = node.gridY + y
            checkX = node.gridX + x
            if (checkX >=0 and checkX<grid.gridSizeX and checkY>= 0 and checkY<grid.gridSizeY):
                nodeNeighbor = grid.grid[checkY][checkX]
                nodeNeighbor.setgCost(getDistance(nodeNeighbor,startNode))
                nodeNeighbor.sethCost(getDistance(nodeNeighbor,targetNode))
                neighbors.append(nodeNeighbor)
    return neighbors

def nodeFromWorldPoint(position,grid):
    percentX = float(position[0])/grid.worldSize[0] 
    percentY = float((grid.worldSize[1]-position[1]))/grid.worldSize[1] 
    x = int((grid.gridSizeX-1)*percentX) 
    y = int((grid.gridSizeY-1)*percentY) 
    return grid.grid[y][x] 
    
#---------------------------------------------------------------------------
#---------------------------------------------------------------------------
# Classes
# node Class:
# worldPosition is a 2d vector (tuple): (x,y)
class Node(object):
    def __init__(self,worldPosition,x,y):
        self.worldPosition = worldPosition
        self.gridX = x
        self.gridY = y
        
    def getX(self):
        return self.worldPosition[0] 
    def getY(self):
        return self.worldPosition[1] 
    def setWalkable(self, fullNode):
        self.walkable = self.isWalkable(fullNode)
    def setgCost(self,cost):
        self.gcost=cost
    def sethCost(self,cost):
        self.hcost=cost
    def getfCost(self):
        return self.gcost+self.hcost
    def setParentNode(self,node):
        self.parent = node
    def isWalkable(self, plane):
        for i in range (len(plane)):
            for k in range (len(plane[i])):
                if plane[i][k] <230:
                    return False
        return True
    
    
# Grid Class
# gridWorldSize is a 2d vector (tuple): (x,y)       
class Grid(object):
    def __init__(self,nodeRadius,image):
        self.nodeRadius = nodeRadius
        gridWorldSize = (len(image[0]),len(image))
        nodeDiameter = nodeRadius*2
        self.gridSizeX=(gridWorldSize[0]/nodeDiameter)
        self.gridSizeY=(gridWorldSize[1]/nodeDiameter)
        grid = [[0 for x in range(self.gridSizeX)]for y in range(self.gridSizeY)]
        for j in range (self.gridSizeY):
            for i in range(self.gridSizeX):
                xpoint = i*nodeDiameter+nodeRadius
                ypoint = gridWorldSize[1] - (j*nodeDiameter+nodeRadius)
                x1 = xpoint-nodeRadius
                x2 = xpoint+nodeRadius
                y1 = ypoint-nodeRadius
                y2 = ypoint+nodeRadius
                nodePlane = image[y1:y2,x1:x2]
                newNode=Node((xpoint,ypoint),i,j)
                newNode.setWalkable(nodePlane)
                grid[j][i]= newNode
        self.grid=grid
        self.worldSize = gridWorldSize


# path finding method/Class            
class Pathfinding(object):
    def __init__(self,startPosition, targetPosition,grid,image):
        self.mappy=image
        self.pathExist=False
        startNode = nodeFromWorldPoint(startPosition,grid)
        targetNode = nodeFromWorldPoint(targetPosition,grid)
        startNode.sethCost(getDistance(startNode,targetNode))
        startNode.setgCost(0)
        self.startNode=startNode
        self.targetNode=targetNode
        openSet=[startNode]
        closedSet=[]
        while(len(openSet)>0):
            currentNode = openSet[0]
            for i in range(len(openSet)):
                if(openSet[i].getfCost()<currentNode.getfCost() or (openSet[i].getfCost() == currentNode.getfCost() and openSet[i].hcost < currentNode.hcost)):
                    currentNode = openSet[i]
            openSet.remove(currentNode)
            closedSet.append(currentNode)
            #self.mappy = paintNode(currentNode,self.mappy)
            if (currentNode.worldPosition == targetNode.worldPosition):
                self.path = retracePath(startNode,targetNode)
                self.pathExist=True
                return
            for nodeNeighbor in getNeighbors(currentNode,grid,startNode,targetNode):
                if ((not nodeNeighbor.walkable) or (nodeNeighbor in closedSet)):
                    continue                
                newMovementCostToNeighbor = currentNode.gcost + getDistance(currentNode,nodeNeighbor)
                nodeNeighbor.setgCost(newMovementCostToNeighbor)
                nodeNeighbor.sethCost(getDistance(nodeNeighbor, targetNode))
                nodeNeighbor.setParentNode(currentNode)
                if(not nodeNeighbor in openSet):
                    openSet.append(nodeNeighbor)
#---------------------------------------------------------------------------------------------------
#main script:                     
                        
if __name__ == "__main__":
    
    resolution = 1
    start=(1380,992,3,1)
    end=(237,154,0,1)
    nodeRadius = 4
    transformMatrix = numpy.matrix('1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1')
    start= offsetLocation(transformMatrix,start,resolution)
    end = offsetLocation(transformMatrix, end, resolution)
    image = read_pgm("3rd-Floor-2D-Map-Test-2.pgm", byteorder='<')
    image.flags.writeable=True
    grid = Grid(nodeRadius,image)
    path = Pathfinding(start,end,grid,image)
    image = paintNode(path.startNode,nodeRadius,image)
    print "path was found: ",path.pathExist
    if(path.pathExist):
        image = drawPath(path,image,nodeRadius, grid.worldSize)
    image = paintNode(path.targetNode,nodeRadius,image)
    pyplot.imshow(image, pyplot.cm.gray)
    pyplot.show()   
