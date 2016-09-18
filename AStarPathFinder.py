#!/usr/bin/env python
import re
import numpy

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
def drawPath(path,image,gridWorldSize):
    for node in path.path:
        nodeDiameter=8
        nodeRadius = 4
        xpoint = node.gridX*nodeDiameter+nodeRadius              # node diameter hardcoded and radius
        ypoint = gridWorldSize[1] - (node.gridY*nodeDiameter+nodeRadius) # diamter and radius hardcoded
        x1 = xpoint-nodeRadius
        x2 = xpoint+nodeRadius
        y1 = ypoint-nodeRadius
        y2 = ypoint+nodeRadius
        image[y1:y2,x1:x2] =100
    return image

def paintNode(node,image):
    nodeRadius = 4
    xpoint = node.getX()
    ypoint = node.getY()
    x1 = xpoint-nodeRadius
    x2 = xpoint+nodeRadius
    y1 = ypoint-nodeRadius
    y2 = ypoint+nodeRadius
    image[y1:y2,x1:x2] =10
    return image
    


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
                # where to get gridSizeX, gridSizeY, the actual grid?
    return neighbors

def nodeFromWorldPoint(position,grid):
    percentX = float(position[0])/grid.worldSize[0] # find the gridWorldSizeX image size X in px
    percentY = float((grid.worldSize[1]-position[1]))/grid.worldSize[1] # ...
    x = int((grid.gridSizeX-1)*percentX) #find the gridSizeX - sixe of grid list
    y = int((grid.gridSizeY-1)*percentY) # ...
    return grid.grid[y][x] # returning the node of grid x,y or return just x,y and figure it out later
            
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
                


                        
                        
if __name__ == "__main__":
    from matplotlib import pyplot
    start=(1380,992)
    end=(731,305)
    image = read_pgm("3rd-Floor-2D-Map-Test-2.pgm", byteorder='<')
    image.flags.writeable=True
    grid = Grid(4,image)
    path = Pathfinding(start,end,grid,image)
    image = paintNode(path.startNode,image)
    print "path was found: ",path.pathExist
    if(path.pathExist):
        image = drawPath(path,image,grid.worldSize)
    image = paintNode(path.targetNode,image)
    pyplot.imshow(image, pyplot.cm.gray)
    pyplot.show()   
