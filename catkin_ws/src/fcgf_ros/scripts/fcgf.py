#!/usr/bin/env python3.7
import rospy
import os
import glob
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d
import numpy as np

from urllib.request import urlretrieve

# make these into arguments for the launch file
os.environ["OMP_NUM_THREADS"] = "12"
HOME_path = os.getenv("HOME")
ply_filename = "ballast_tank.ply"
catkin_ws_path = HOME_path + "/repos/inspectrone/catkin_ws/"
downloads_path = catkin_ws_path + "downloads/"


class Viewer:
    def __init__(self, listener, matcher, parent=None):
        self.listener = listener
        rospy.loginfo("initialization")

        self.vis = open3d.visualization.Visualizer()
        self.open3d_pc = None
        self.prev_red_n = None  # yes because "red" is written read :)
        self.updater()

    def ros_to_open3d(self, pc_ros):
        # convert to xyz point cloud
        pc_xyz = pc2.read_points(
            pc_ros, skip_nans=True, field_names=("x", "y", "z")
        )
        # convert to open3d point cloud
        return open3d.utility.Vector3dVector(pc_xyz)

    def updater(self):

        rospy.loginfo("start")
        while self.listener.pc is None:
            rospy.loginfo("No Publsihed Pointclouds Yet, trying again in 0.2 sec")
            rospy.sleep(0.2)

        self.prev_red_n = self.listener.n

        rospy.loginfo("rendering pointcloud #{}".format(self.prev_red_n))
        self.open3d_pc = open3d.geometry.PointCloud()

        self.open3d_pc.points = self.ros_to_open3d(self.listener.pc)

        self.vis.create_window()
        self.vis.add_geometry(self.listener.map)
        self.vis.update_geometry(self.listener.map)
        print("get points")
        self.vis.add_geometry(self.open3d_pc)
        print("add points")
        self.vis.update_geometry(self.open3d_pc)
        self.vis.poll_events()

        self.vis.update_renderer()
        while not rospy.is_shutdown():
            if self.prev_red_n != self.listener.n:
                self.prev_red_n = self.listener.n
                self.open3d_pc.points = self.ros_to_open3d(self.listener.pc)
                #teest = copy.deepcopy(self.open3d_pc).translate((1.3*self.prev_red_n, 0, 0))
                rospy.loginfo("rendering pointcloud #{}".format(self.prev_red_n))
                # self.vis.add_geometry(self.open3d_pc)
                self.vis.update_geometry(self.open3d_pc)
            self.vis.poll_events()
            self.vis.update_renderer()

            #rospy.loginfo(self.listener.n)


class PcMatcher:
    def __init__(self, listener, parent=None):
        self.listener = listener
        rospy.loginfo("initialization")

        self.vis = open3d.visualization.Visualizer()
        self.open3d_pc = None
        self.prev_red_n = None  # yes because "red" is written read :)
        print("resnet location at: " + downloads_path + "ResUNetBN2C-16feat-3conv.pth")

        if not os.path.isfile(downloads_path + "ResUNetBN2C-16feat-3conv.pth"):
            print("Downloading weights to ", downloads_path + "ResUNetBN2C-16feat-3conv.pth")
            urlretrieve(
                "https://node1.chrischoy.org/data/publications/fcgf/2019-09-18_14-15-59.pth",
                downloads_path + "ResUNetBN2C-16feat-3conv.pth",
            )


class PointCloudListner:
    def __init__(self):
        self.pc = None
        self.n = 0
        self.map = self.find_ply(catkin_ws_path, ply_filename)
        self.init_listener()

    def init_listener(self):
        rospy.init_node("fcgf", anonymous=True, disable_signals=True) #TODO find a better solution for keyboard events not working with rospy.sleep()
        # rospy.Subscriber("/ballast_tank_ply", PointCloud2, self.callback)
        rospy.Subscriber("/points_throttle", PointCloud2, self.callback)

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


if __name__ == "__main__":
    listener = PointCloudListner()
    matcher = PcMatcher(listener)
    updater = Viewer(listener, matcher)
    rospy.spin()
