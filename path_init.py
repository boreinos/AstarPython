#!/usr/bin/env python
import roslib
import re
import numpy
from numpy.linalg import inv
from matplotlib import pyplot
from geometry_msgs.msg import Pose
from AStarPathFinder import *
from pgm_reader import *
from transform_methods import *
#---------------------------------------------------------------------------------------------------
# drawing methods:
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
#---------------------------------------------------------------------
# read map from file:
    image = read_pgm("3rd-Floor-2D-Map-Test-2.pgm", byteorder='<')
    image.flags.writeable=True
#--------------------------------------------------------------------
# create a grid and compute path:
    grid = Grid(nodeRadius,image)
    path = Pathfinding(start,end,grid,image)
    print "path was found: ",path.pathExist
#-------------------------------------------------------------------
# draw the path for demonstration purposes only:
    if(path.pathExist):
        image = drawPath(path,image,nodeRadius, grid.worldSize)
    image = paintNode(path.startNode,nodeRadius,image)
    image = paintNode(path.targetNode,nodeRadius,image)
    pyplot.imshow(image, pyplot.cm.gray)
    pyplot.show()  
