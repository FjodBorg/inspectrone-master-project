#!/usr/bin/env python3.7
import rospy
import os
import glob
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d
import numpy as np
import time

from urllib.request import urlretrieve

# deeplearning
import model as mdl
import torch
import copy
from util.misc import extract_features

# teaser
import teaserpp_python

# make these into arguments for the launch file
os.environ["OMP_NUM_THREADS"] = "12"
HOME_path = os.getenv("HOME")
ply_filename = "ballast_tank.ply"
catkin_ws_path = HOME_path + "/repos/inspectrone/catkin_ws/"
downloads_path = catkin_ws_path + "downloads/"
voxel_size = 0.025

class PerformanceMetric:
    def __init__(self, parent=None):
        self.timings = dict()

    def start_time(self, name):
        cur_time = time.time()
        self.timings[name] = - cur_time  # negative means it hasen't got the second timing yet

    def stop_time(self, name):
        cur_time = time.time()
        elapsed_time = self.timings[name]

        self.timings[name] = cur_time + elapsed_time  # eleapsed time is negative

    def print_time(self, names):

        rospy.loginfo("printing metrics for: " + str(names))
        for name in names:
            print("{:20s} took: {:2.5f} sec".format(name, self.timings[name]))


class Main:
    def __init__(self, listener, matcher, parent=None):
        self.listener = listener
        self.matcher = matcher
        rospy.loginfo("initialization Visualization")

        self.vis = open3d.visualization.Visualizer()
        self.open3d_pc = None
        self.open3d_map = self.listener.map
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

        self.open3d_pc.points = self.matcher.ros_to_open3d(self.listener.pc)

        transformation = self.matcher.find_transform(self.open3d_pc, self.open3d_map)
        self.open3d_pc.transform(transformation)

        # visualize map
        self.vis.create_window()
        self.vis.add_geometry(self.open3d_map)
        self.vis.update_geometry(self.open3d_map)

        # add point cloud to visualization
        self.vis.add_geometry(self.open3d_pc)
        self.vis.update_geometry(self.open3d_pc)
        self.vis.poll_events()
        self.vis.update_renderer()
        while not rospy.is_shutdown():
            if self.prev_red_n != self.listener.n:
                self.prev_red_n = self.listener.n
                self.open3d_pc.points = self.matcher.ros_to_open3d(self.listener.pc)
                #teest = copy.deepcopy(self.open3d_pc).translate((1.3*self.prev_red_n, 0, 0))
                rospy.loginfo("Calculating transform")
                
                transformation = self.matcher.find_transform(self.open3d_pc, self.open3d_map)
                self.open3d_pc.transform(transformation)
                self.vis.update_geometry(self.open3d_pc)
                rospy.loginfo("Rendering transformed pointcloud #{}".format(self.prev_red_n))
                # self.vis.add_geometry(self.open3d_pc)
            self.vis.poll_events()
            self.vis.update_renderer()

            #rospy.loginfo(self.listener.n)


class PcMatcher:
    def __init__(self, parent=None):
        rospy.loginfo("initialization Matching method")
        self.model, self.checkpoint, self.device = self.load_checkpoint()
        self.voxel_size = voxel_size
        self.map_pc = open3d.geometry.PointCloud()
        self.map_feat = open3d.pipelines.registration.Feature()
        self.pc = open3d.geometry.PointCloud()
        self.feat = open3d.pipelines.registration.Feature()

    def load_checkpoint(self):
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo("cuda for torch: " + str(torch.cuda.is_available()))
        model_file = downloads_path + "ResUNetBN2C-16feat-3conv.pth"
        rospy.loginfo("resnet location at: " + model_file)

        if not os.path.isfile(model_file):
            rospy.loginfo("Downloading weights to ", model_file)
            urlretrieve(
                "https://node1.chrischoy.org/data/publications/fcgf/2019-09-18_14-15-59.pth",
                downloads_path + "ResUNetBN2C-16feat-3conv.pth",
            )

        model = mdl.resunet.ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
        checkpoint = torch.load(model_file)
        model.load_state_dict(checkpoint['state_dict'])
        model.eval()
        model = model.to(device)

        return model, checkpoint, device

    def get_features(self, point_cloud):
        xyz_down, feature = extract_features(
            self.model,
            xyz=np.array(point_cloud.points),
            voxel_size=self.voxel_size,
            device=self.device,
            skip_check=True)
        return xyz_down, feature

    def find_transform(self, point_cloud, global_map):
        if len(self.map_pc.points) == 0:  # this depends if we have the whole global map or not
            metrics.start_time("loading tank")
            map_pc, map_feat = self.get_features(global_map)

            self.map_pc.points = open3d.utility.Vector3dVector(map_pc)
            self.map_feat.data = (map_feat.detach().cpu().numpy().T)#.astype(np.float64)
            metrics.stop_time("loading tank")

        metrics.start_time("loading ply")
        pc, feat = self.get_features(point_cloud)

        self.pc.points = open3d.utility.Vector3dVector(pc)
        self.feat.data = (feat.detach().cpu().numpy().T)#.astype(np.float64)
        metrics.stop_time("loading ply")
        # print(self.pc, "\n", self.map_pc, "\n")
        # print(self.feat, "\n", self.map_feat, "\n\n")
        metrics.start_time("ransac")
        result_ransac = self.execute_global_registration(self.pc, self.map_pc, self.feat, self.map_feat, self.voxel_size)
        metrics.stop_time("ransac")

        metrics.print_time(["loading tank", "loading ply", "ransac"])

        return result_ransac.transformation

    def execute_global_registration(self, source_down, target_down, source_fpfh,
                                    target_fpfh, voxel_size):
        distance_threshold = voxel_size * 0.4
        # print(":: RANSAC registration on downsampled point clouds.")
        # print("   Since the downsampling voxel size is %.3f," % voxel_size)
        # print("   we use the distance threshold %.3f." % distance_threshold)
        result = open3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            open3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            4, [
                open3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                    0.9),
                open3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], open3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
        return result

    def ros_to_open3d(self, pc_ros):
        # convert to xyz point cloud
        pc_xyz = pc2.read_points(
            pc_ros, skip_nans=True, field_names=("x", "y", "z")
        )
        # convert to open3d point cloud
        return open3d.utility.Vector3dVector(pc_xyz)

class PcListner:
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
    global metrics
    metrics = PerformanceMetric()

    listener = PcListner()
    matcher = PcMatcher()
    updater = Main(listener, matcher)
    rospy.spin()
