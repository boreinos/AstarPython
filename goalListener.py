#!/usr/bin/env python
# Listener:
import rospy
from matplotlib import pyplot
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"message received")
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
    pyplot.imshow(reversed_image, pyplot.cm.gray)
    pyplot.show()  

def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber("map", OccupancyGrid, callback)
    rospy.spin()
#--------------------------------------------------------------
if __name__ == '__main__':
    listener()
