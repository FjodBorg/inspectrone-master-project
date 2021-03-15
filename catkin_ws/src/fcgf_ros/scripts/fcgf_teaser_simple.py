#!/usr/bin/env python3.7
import os
import numpy as np #NUMPY MUST COME BEFORE open3d
import open3d
import time

from urllib.request import urlretrieve
from util.visualization import get_colored_point_cloud_feature

# deeplearning
import model as mdl
import torch
import copy
from util.misc import extract_features

# teaser
import teaserpp_python
from core.knn import find_knn_gpu
from extra.helpers import get_teaser_solver, Rt2T


#ros related
import rospy
#import tf_conversions
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
#from geometry_msgs.msg import PoseStamped

#custom modules
from extra import extensions, listeners, matching_helpers, essentials

# make these into arguments for the launch file
os.environ["OMP_NUM_THREADS"] = "12"
HOME_path = os.getenv("HOME")
ply_filename = "ballast_tank.ply"
catkin_ws_path = HOME_path + "/repos/inspectrone/catkin_ws/"
downloads_path = catkin_ws_path + "downloads/"
voxel_size = 0.05  # 0.025





class Main:
    def __init__(self, listener, parent=None):
        self.listener = listener
        rospy.loginfo("initialization Visualization")

        self.vis = open3d.visualization.Visualizer()
        self.open3d_pc = open3d.geometry.PointCloud()
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
        
        self.open3d_pc.points = self.ros_to_open3d(self.listener.pc)
        demo(self.open3d_pc)

        while not rospy.is_shutdown():
            if self.prev_red_n != self.listener.n:
                self.open3d_pc.points = self.ros_to_open3d(self.listener.pc)
                demo(self.open3d_pc)


def demo(pcd):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("cuda active: ", torch.cuda.is_available())

    #print(config.model)
    model_file = downloads_path + "ResUNetBN2C-16feat-3conv.pth"
    
    if not os.path.isfile(model_file):
        if not os.path.exists(downloads_path):
            os.makedirs(downloads_path)
        rospy.loginfo("Downloading weights to " + model_file)
        urlretrieve(
            "https://node1.chrischoy.org/data/publications/fcgf/2019-09-18_14-15-59.pth",
            downloads_path + "ResUNetBN2C-16feat-3conv.pth",
        )
    metrics.start_time("loading resnet")
    model = mdl.resunet.ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
    checkpoint = torch.load(model_file)


    #checkpoint = torch.load(downloads_path + "ResUNetBN2C-16feat-3conv.pth")
    model = mdl.resunet.ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()

    model = model.to(device)
    metrics.stop_time("loading resnet")
    
    metrics.start_time("loading tank")
    pcd_map = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"pcl_ballast_tank.ply")
    metrics.stop_time("loading tank")

    metrics.start_time("processing tank")
    xyz_down_map, feature_map = matcher.get_features(pcd_map)

    map_pcd = open3d.geometry.PointCloud()
    map_pcd.points = open3d.utility.Vector3dVector(xyz_down_map)
    map_pcd.paint_uniform_color([1, 0.706, 0])
    
    metrics.stop_time("processing tank")

    metrics.start_time("processing scan")
    #pcd = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"sensor_tank1.ply")
    xyz_down, feature = matcher.get_features(pcd)
    sensor_pcd = open3d.geometry.PointCloud()
    sensor_pcd.points = open3d.utility.Vector3dVector(xyz_down)
    sensor_pcd.paint_uniform_color([0, 0, 0])
    metrics.stop_time("processing scan")

    metrics.start_time("finding correspondences")
    corrs_A, corrs_B = matcher.find_correspondences(feature_map, feature, mutual_filter=True)
    A_xyz = pcd2xyz(map_pcd)  # np array of size 3 by N
    B_xyz = pcd2xyz(sensor_pcd)  # np array of size 3 by M
    A_corr = A_xyz[:, corrs_A]  # np array of size 3 by num_corrs
    B_corr = B_xyz[:, corrs_B]  # np array of size 3 by num_corrs
    metrics.stop_time("finding correspondences")

    metrics.start_time("visualizing correspondences")
    num_corrs = A_corr.shape[1]
    #print("FCGF generates {} putative correspondences.".format(num_corrs))
    
    # visualize the point clouds together with feature correspondences
    points = np.concatenate((A_corr.T, B_corr.T), axis=0)
    lines = []
    for i in range(num_corrs):
        lines.append([i, i + num_corrs])
    colors = [[0, 1, 0] for i in range(len(lines))]  # lines are shown in green
    line_set = open3d.geometry.LineSet(
        points=open3d.utility.Vector3dVector(points),
        lines=open3d.utility.Vector2iVector(lines),
    )
    line_set.colors = open3d.utility.Vector3dVector(colors)
    metrics.stop_time("visualizing correspondences")


    metrics.start_time("calculating transform")
    # robust global registration using TEASER++
    NOISE_BOUND = 0.05  # config.voxel_size
    teaser_solver = get_teaser_solver(NOISE_BOUND)
    teaser_solver.solve(A_corr, B_corr)
    solution = teaser_solver.getSolution()
    R_teaser = solution.rotation
    t_teaser = solution.translation
    T_teaser = Rt2T(R_teaser, t_teaser)
    metrics.stop_time("calculating transform")

    # Visualize the registration results
    metrics.print_all_timings()

    map_pcd_T_teaser = copy.deepcopy(map_pcd).transform(T_teaser)
    open3d.visualization.draw([map_pcd, sensor_pcd, line_set])
    open3d.visualization.draw([map_pcd_T_teaser, sensor_pcd])


def pcd2xyz(pcd):
    return np.asarray(pcd.points).T


if __name__ == "__main__":

    global metrics, matcher
    config = essentials.Config()
    metrics = extensions.PerformanceMetrics()
    listener = listeners.PointCloudListener(config)
    matcher = matching_helpers.Matcher(config)

    updater = Main(listener)
    rospy.spin()
