#!/usr/bin/env python
from numpy.linalg import inv
from AStarPathFinder import *
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
    newY = worldSize[1]-int(abs(bottom[1])/resolution)-int(point[1]/resolution)
    return (newX,newY)
def PGMToMap(bottom,resolution,point,worldSize):
    mapX = point[0]*resolution - bottom[0]
    mapY = worldSize[1]*resolution -point[1]*resolution - bottom[1]
    return (mapX,mapY)
