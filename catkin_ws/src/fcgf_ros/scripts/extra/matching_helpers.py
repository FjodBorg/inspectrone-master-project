#!/usr/bin/env python3.7
import os
import numpy as np
import open3d
import rospy
import sensor_msgs.point_cloud2 as pc2

# pytorch
import model as mdl
import torch
from util.misc import extract_features
from urllib.request import urlretrieve

# teaser
from core.knn import find_knn_gpu
from extra.helpers import get_teaser_solver, Rt2T

# custom
from extra import extensions

class MatcherStatics():
    def __init__(self, config):
        self.config = config
        self.voxel_size = config.voxel_size
        self.model, self.device = self._load_model(config)
        self.pcd_map = open3d.geometry.PointCloud()
        self.pcd_scan = open3d.geometry.PointCloud()

        self.pcd_map = self._load_static_ply(config)

    def _load_model(self, config):
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo("Using Cuda: " + str(torch.cuda.is_available()))
        #  model_file = downloads_path + "ResUNetBN2C-16feat-3conv.pth"
        rospy.loginfo("resnet location at: " + config.model)

        if not os.path.isfile(config.model):
            if not os.path.exists(config.paths.downloads):
                os.makedirs(config.path.downloads)
            
            #  TODO 32 features, download the one rune gave you. Its easier (3conv size)
            rospy.loginfo("Downloading weights to " + config.model)
            urlretrieve(
                "https://node1.chrischoy.org/data/publications/fcgf/2019-09-18_14-15-59.pth",
                config.paths.downloads + "ResUNetBN2C-16feat-3conv.pth",
            )

        checkpoint = torch.load(config.model)
        model = mdl.resunet.ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
        model.load_state_dict(checkpoint['state_dict'])
        model.eval()

        model = model.to(device)

        return model, device

    def _load_static_ply(self, config):
        pcd_map = open3d.io.read_point_cloud(config.static_ply)
        return pcd_map


class MatcherHelper():
    def __init__(self, listener):
        self.listener = listener

    def pcd2xyz(self, pcd):
        return np.asarray(pcd.points).T

    def ros_to_open3d(self, pc_ros):
        # convert to xyz point cloud
        pc_xyz = pc2.read_points(
            pc_ros, skip_nans=True, field_names=("x", "y", "z")
        )
        # convert to open3d point cloud
        return open3d.utility.Vector3dVector(pc_xyz)

    def _load_topic_ply_points(self):
        pcd_scan = self.ros_to_open3d(self.listener.pc)
        return pcd_scan

    def get_map(self):
        return self.pcd_map

    def get_scan(self):
        self.pcd_scan.points = self._load_topic_ply_points()
        return self.pcd_scan

    def get_xyz_features(self, point_cloud):
        xyz_down, feature = extract_features(
            self.model,
            xyz=np.array(point_cloud.points),
            voxel_size=self.voxel_size,
            device=self.device,
            skip_check=True)
        return xyz_down, feature


class Matcher(MatcherStatics, MatcherHelper):
    def __init__(self, config, listener):
        MatcherStatics.__init__(self, config)  # init parent attributes
        MatcherHelper.__init__(self, listener)  # init parent attributes
        self.pcd_map_down = open3d.geometry.PointCloud()
        self.pcd_scan_down = open3d.geometry.PointCloud()
        self.pcd_map_features = None

    def get_open3d_features(self, point_cloud):
        '''Same as get_xyz_features, just with pcd in open3d format'''
        if point_cloud == self.pcd_map:  # if same reference as map
            pcd_down = self.pcd_map_down  # get reference to map
            if self.pcd_map_features is None:
                # if no features has been calcualted yet
                xyz_down, self.pcd_map_features = self.get_xyz_features(point_cloud)
                pcd_down.points = open3d.utility.Vector3dVector(xyz_down)
                pcd_down.paint_uniform_color([1, 0.706, 0])
                
            # use features that was calculated
            features = self.pcd_map_features
        else:
            pcd_down = self.pcd_scan_down  # get reference to scan
            xyz_down, features = self.get_xyz_features(point_cloud)
            pcd_down.points = open3d.utility.Vector3dVector(xyz_down)
            pcd_down.paint_uniform_color([0, 0, 0])
        return pcd_down, features

    def find_correspondences(self, feats0, feats1, mutual_filter=True):
        nn_max_n = 25  # 250 # TODO if gpu memory is problem
        nns01 = find_knn_gpu(feats0, feats1, nn_max_n=nn_max_n, knn=1, return_distance=False)
        # corres01_idx0 = (torch.arange(len(nns01))
        # corres01_idx1 = (nns01.long().squeeze())
        corres01_idx0 = (torch.arange(len(nns01)).long().squeeze()).detach().cpu().numpy()
        corres01_idx1 = (nns01.long().squeeze()).detach().cpu().numpy()
        # corres01_idx0 = corres01_idx0.detach().cpu().numpy()
        # corres01_idx1 = corres01_idx1.detach().cpu().numpy()

        if not mutual_filter:
            return corres01_idx0, corres01_idx1

        nns10 = find_knn_gpu(feats1, feats0, nn_max_n=nn_max_n, knn=1, return_distance=False)
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

    def convert_correspondences(self, map_pcd, sensor_pcd, corrs_A, corrs_B):
        np_A_xyz = self.pcd2xyz(map_pcd)  # np array of size 3 by N
        np_B_xyz = self.pcd2xyz(sensor_pcd)  # np array of size 3 by M
        np_corrs_A = np_A_xyz[:, corrs_A]  # np array of size 3 by num_corrs
        np_corrs_B = np_B_xyz[:, corrs_B]  # np array of size 3 by num_corrs
        return np_corrs_A, np_corrs_B

    def draw_correspondences(self, A_corr, B_corr):
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
        return line_set

    def find_transform(self, np_corrs_A, np_corrs_B, NOISE_BOUND=0.01):
        # robust global registration using TEASER++
        # Noise_bound is roughly voxel_size
        # sometimes if downsample is extreme noisebound should probably be smaller
        teaser_solver = get_teaser_solver(NOISE_BOUND)
        teaser_solver.solve(np_corrs_A, np_corrs_B)
        solution = teaser_solver.getSolution()
        R_teaser = solution.rotation
        t_teaser = solution.translation
        T_teaser = Rt2T(R_teaser, t_teaser)
        return T_teaser


class MatcherRansac(Matcher):
    def __init__(self, config, listener):
        Matcher.__init__(self, config, listener)
        self.pcd_map_features_cpu = None
        self.pcd_scan_features_cpu = open3d.pipelines.registration.Feature()
        self.voxel_size = config.voxel_size

    def find_transform(self, source_down, target_down, source_fpfh, target_fpfh):
        if self.pcd_map_features_cpu is None:
            self.pcd_map_features_cpu = open3d.pipelines.registration.Feature()
            self.pcd_map_features_cpu.data = (target_fpfh.detach().cpu().numpy().T)#.astype(np.float64)

        self.pcd_scan_features_cpu = open3d.pipelines.registration.Feature()
        self.pcd_scan_features_cpu.data = (source_fpfh.detach().cpu().numpy().T)#.astype(np.float64)
        # print(self.pc, "\n", self.map_pc, "\n")
        # print(self.feat, "\n", self.map_feat, "\n\n")
        result_ransac = self.execute_global_registration(source_down, target_down, self.pcd_scan_features_cpu, self.pcd_map_features_cpu)

        return result_ransac.transformation

    def execute_global_registration(self, source_down, target_down, source_fpfh,
                                target_fpfh):
        distance_threshold = self.voxel_size * 0.4
        # print(":: RANSAC registration on downsampled point clouds.")
        # print("   Since the downsampling voxel size is %.3f," % voxel_size)
        # print("   we use the distance threshold %.3f." % distance_threshold)
        # print(type(source_down),type(target_down),type(source_fpfh),type(target_fpfh))
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
    

class MatcherVisualizer:
    def __init__(self, config, listener):
        if config.teaser is True:
            self.matcher = Matcher(config, listener)
        else:
            self.matcher = MatcherRansac(config, listener)

        self.metrics = extensions.PerformanceMetrics()
        self.pcd_map_features_cpu = None
        self.pcd_scan_features_cpu = open3d.pipelines.registration.Feature()
        self.voxel_size = config.voxel_size

    def find_transform(self, np_corrs_A, np_corrs_B, NOISE_BOUND=0.01):
        # robust global registration using TEASER++
        # Noise_bound is roughly voxel_size
        # sometimes if downsample is extreme noisebound should probably be smaller
        teaser_solver = get_teaser_solver(NOISE_BOUND)
        teaser_solver.solve(np_corrs_A, np_corrs_B)
        solution = teaser_solver.getSolution()
        R_teaser = solution.rotation
        t_teaser = solution.translation
        T_teaser = Rt2T(R_teaser, t_teaser)
        return T_teaser