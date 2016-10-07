#!/usr/bin/env python
from geometry_msgs.msg import Pose
#from geometry_msgs.msg import Goal

# listener function:
def listener():
    rospy.init_node('goal_listener', anonymous=True)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rospy.spin()
