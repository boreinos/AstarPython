#!/usr/bin/env python
#publisher node:
import rospy
from std_msgs.msg import String

def PathPublisher():
    pub = rospy.Publisher('Path', String, queue_size = 10)
    rospy.init_node('PathPlanner', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        hello_str="hellow world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
#----------------------------------------------------
#Main:
if __name__ =='__main__':
    try:
        PathPublisher()
    except rospy.ROSInterruptExecption:
        pass    
