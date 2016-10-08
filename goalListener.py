#!/usr/bin/env python
# Listener:
import rospy
from matplotlib import pyplot
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

map_array= np.zeros((1,1))
goal = None
ready = False

def callback3(data):
    #get goal, raise flag and allow current pose to handle
    rospy.loginfo(rospy.get_caller_id()+" goal recieved" )
    goal = data.data
    ready = True

def callback2(data):
    if ready:
        #acknowledge pose was recieved:
        rospy.loginfo(rospy.get_caller_id()+" pose map received")
        #do something ...
        ready = False
    return 

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+" map data received") #acknowledge for degub purposes
    width=data.info.width
    height=data.info.height
    image=np.zeros((height,width))
    rawArray = data.data
    length = len(rawArray)
    start = 0
    for i in range(height):
        n=(i+1)*width
        row = list(rawArray[start:n])
        for k in range(len(row)):
            if row[k] >5:
                image[i,k]=1
            elif row[k] == -1:
                image[i,k]=200
            else:
                image[i,k]=256
        start = n+1
    reversed_image = image[::-1]
    map_array = reveresed_image #save map
    #pyplot.imshow(reversed_image, pyplot.cm.gray)
    #pyplot.show()  

def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber("map", OccupancyGrid, callback)
    rospy.Subscriber("RosAria/pose_map",Odometry, callback2)
    rospy.Subscriber("move_base_simple/goal",PoseStamped,callback3)
    rospy.spin()
#--------------------------------------------------------------
if __name__ == '__main__':
    listener()
