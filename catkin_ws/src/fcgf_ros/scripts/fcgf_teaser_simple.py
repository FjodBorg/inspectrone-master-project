#!/usr/bin/env python3.7
import os
import glob
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
from core.knn import find_knn_gpu
from extra.helpers import *

#ros related
import rospy
#import tf_conversions
import tf
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
#from geometry_msgs.msg import PoseStamped

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

        self.open3d_pc.points = self.ros_to_open3d(self.listener.pc)
        demo(self.open3d_pc)

        while not rospy.is_shutdown():
            if self.prev_red_n != self.listener.n:
                demo(self.open3d_pc)


class PcListner:
    def __init__(self):
        self.pc = None
        self.n = 0
        self.map = self.find_ply(catkin_ws_path, ply_filename)
        self.init_listener()

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


def demo(pcd):


    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("cuda active: ", torch.cuda.is_available())

    #print(config.model)
    model_file = downloads_path + "ResUNetBN2C-16feat-3conv.pth"
    

    if not os.path.isfile(model_file):
        rospy.loginfo("Downloading weights to " + model_file)
        urlretrieve(
            "https://node1.chrischoy.org/data/publications/fcgf/2019-09-18_14-15-59.pth",
            downloads_path + "ResUNetBN2C-16feat-3conv.pth",
        )

    model = mdl.resunet.ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
    checkpoint = torch.load(model_file)


    #checkpoint = torch.load(downloads_path + "ResUNetBN2C-16feat-3conv.pth")
    model = mdl.resunet.ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()

    model = model.to(device)

    #pcd = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"sensor_tank1.ply")
    pcd_map = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"pcl_ballast_tank.ply")

    xyz_down, feature = extract_features(
        model,
        xyz=np.array(pcd.points),
        voxel_size=voxel_size,
        device=device,
        skip_check=True,
    )

    xyz_down_map, feature_map = extract_features(
        model,
        xyz=np.array(pcd_map.points),
        voxel_size=voxel_size,
        device=device,
        skip_check=True,
    )

    map_pcd = open3d.geometry.PointCloud()
    map_pcd.points = open3d.utility.Vector3dVector(xyz_down_map)
    map_pcd.paint_uniform_color([1, 0.706, 0])
    sensor_pcd = open3d.geometry.PointCloud()
    sensor_pcd.points = open3d.utility.Vector3dVector(xyz_down)
    sensor_pcd.paint_uniform_color([0, 0, 0])

    ref = open3d.pipelines.registration.Feature()
    ref.data = feature_map.detach().cpu().numpy().T  # .astype(np.float64)
    test = open3d.pipelines.registration.Feature()
    test.data = feature.detach().cpu().numpy().T  # .astype(np.float64)

    corrs_A, corrs_B = find_correspondences(feature_map, feature, mutual_filter=True)
    A_xyz = pcd2xyz(map_pcd)  # np array of size 3 by N
    B_xyz = pcd2xyz(sensor_pcd)  # np array of size 3 by M
    A_corr = A_xyz[:, corrs_A]  # np array of size 3 by num_corrs
    B_corr = B_xyz[:, corrs_B]  # np array of size 3 by num_corrs

    num_corrs = A_corr.shape[1]
    print("FCGF generates {} putative correspondences.".format(num_corrs))

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
    open3d.visualization.draw_geometries([map_pcd, sensor_pcd, line_set])

    # robust global registration using TEASER++
    NOISE_BOUND = 0.05  # config.voxel_size
    teaser_solver = get_teaser_solver(NOISE_BOUND)
    teaser_solver.solve(A_corr, B_corr)
    solution = teaser_solver.getSolution()
    R_teaser = solution.rotation
    t_teaser = solution.translation
    T_teaser = Rt2T(R_teaser, t_teaser)

    # Visualize the registration results
    map_pcd_T_teaser = copy.deepcopy(map_pcd).transform(T_teaser)
    open3d.visualization.draw_geometries([map_pcd_T_teaser, sensor_pcd])


def pcd2xyz(pcd):
    return np.asarray(pcd.points).T


def find_correspondences(feats0, feats1, mutual_filter=True):
    nns01 = find_knn_gpu(feats0, feats1, nn_max_n=250, knn=1, return_distance=False)
    # corres01_idx0 = (torch.arange(len(nns01))
    # corres01_idx1 = (nns01.long().squeeze())
    corres01_idx0 = (torch.arange(len(nns01)).long().squeeze()).detach().cpu().numpy()
    corres01_idx1 = (nns01.long().squeeze()).detach().cpu().numpy()
    # corres01_idx0 = corres01_idx0.detach().cpu().numpy()
    # corres01_idx1 = corres01_idx1.detach().cpu().numpy()

    if not mutual_filter:
        return corres01_idx0, corres01_idx1

    nns10 = find_knn_gpu(feats1, feats0, nn_max_n=250, knn=1, return_distance=False)
    # corres10_idx1 = torch.arange(len(nns10)).long().squeeze()
    # corres10_idx0 = nns10.long().squeeze()
    # corres10_idx1 = (torch.arange(len(nns10)).long().squeeze()).detach().cpu().numpy()
    corres10_idx0 = (nns10.long().squeeze()).detach().cpu().numpy()
    # corres10_idx1 = corres10_idx1.detach().cpu().numpy()
    # corres10_idx0 = corres10_idx0.detach().cpu().numpy()

    mutual_filter = corres10_idx0[corres01_idx1] == corres01_idx0
    corres_idx0 = corres01_idx0[mutual_filter]
    corres_idx1 = corres01_idx1[mutual_filter]

    return corres_idx0, corres_idx1


if __name__ == "__main__":

    listener = PcListner()
    updater = Main(listener)
    rospy.spin()

