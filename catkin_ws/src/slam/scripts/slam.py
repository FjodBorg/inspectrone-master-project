#!/usr/bin/env python3.7
import numpy as np 
# import g2oclasses
from g2oclasses import PoseGraphOptimization

import rospy
import geometry_msgs.msg


def callback(data):
    rospy.loginfo("I heard %s", data.pose.pose)
    rospy.loginfo("I heard %s", data.time)

if __name__ == "__main__":
    # graph = PoseGraphOptimization
    rospy.init_node('slam')
    rospy.Subscriber("/matcher_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()





