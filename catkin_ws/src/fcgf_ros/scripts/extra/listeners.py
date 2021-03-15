#!/usr/bin/env python3.7
import rospy
import sensor_msgs.msg
import numpy as np
import open3d


class PointCloudListener:
    # TODO make dem all depend on config
    def __init__(self, config):
        self.pc = None
        self.n = 0
        self.init_listener()
        self.map = self.find_ply(config.paths.catkin_ws, config.static_ply)

    def init_listener(self):
        rospy.init_node("fcgf", anonymous=True, disable_signals=True) #TODO find a better solution for keyboard events not working with rospy.sleep()
        # rospy.Subscriber("/ballast_tank_ply", PointCloud2, self.callback)
        rospy.Subscriber("/points_throttle", sensor_msgs.msg.PointCloud2, self.callback)

    def callback(self, points):
        self.pc = points
        self.n = self.n + 1

    def find_ply(self, catkin_ws_path, ply_filename):
        ply_file = catkin_ws_path+'src/ply_publisher/cfg/'+ply_filename
        ply_map = open3d.io.read_point_cloud(ply_file)
        ply_map.colors = open3d.utility.Vector3dVector((np.asarray(ply_map.colors))/2)
        return ply_map
        #print("command: "+catkin_ws_path+"**/*.ply")
        #print(glob.glob())
