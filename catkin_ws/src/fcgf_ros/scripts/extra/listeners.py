#!/usr/bin/env python3.7
import rospy
import sensor_msgs.msg


class PointCloudListener:
    # TODO make dem all depend on config
    def __init__(self, config):
        self.pc = None
        self.n = 0
        self.init_listener(config)

    def init_listener(self, config):
        rospy.init_node("fcgf", anonymous=True, disable_signals=True) #TODO find a better solution for keyboard events not working with rospy.sleep()
        # rospy.Subscriber("/ballast_tank_ply", PointCloud2, self.callback)
        rospy.Subscriber(config.topic_ply, sensor_msgs.msg.PointCloud2, self.callback)

    def callback(self, points):
        self.pc = points
        self.n = self.n + 1
