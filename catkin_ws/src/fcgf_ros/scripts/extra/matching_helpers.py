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
import teaserpp_python
from core.knn import find_knn_gpu
#from extra.helpers import get_teaser_solver, Rt2T

# custom
from extra import extensions

# Faiss
import faiss 

class MatcherBase():
    def __init__(self, config):
        self._config = config
        self._voxel_size = config.voxel_size
        self._model, self.device = self._load_model(config)
        self._pcd_map = open3d.geometry.PointCloud()
        self._pcd_scan = open3d.geometry.PointCloud()

        self._pcd_map = self._load_static_ply(config)

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

    def _load_topic_ply_points(self):
        pcd_scan = self.ros_to_open3d(self._listener.pc)
        return pcd_scan


class MatcherHelper(MatcherBase):
    def __init__(self, config, listener):
        super().__init__(config)  # init parent attributes

        # init extra attributes for downsampling and features
        self._listener = listener 
        self._pcd_map_down = open3d.geometry.PointCloud()
        self._pcd_scan_down = open3d.geometry.PointCloud()
        self._pcd_map_features = None

    def pcd2xyz(self, pcd):
        return np.asarray(pcd.points).T

    def ros_to_open3d(self, pc_ros):
        # convert to xyz point cloud
        pc_xyz = pc2.read_points(
            pc_ros, skip_nans=True, field_names=("x", "y", "z")
        )
        # convert to open3d point cloud
        return open3d.utility.Vector3dVector(pc_xyz)

    def get_map(self):
        return self._pcd_map

    def get_scan(self):
        self._pcd_scan.points = self._load_topic_ply_points()
        return self._pcd_scan

    def get_xyz_features(self, point_cloud):
        xyz_down, feature = extract_features(
            self._model,
            xyz=np.array(point_cloud.points),
            voxel_size=self._voxel_size,
            device=self.device,
            skip_check=True)
        return xyz_down, feature

    def get_open3d_features(self, point_cloud):
        '''Same as get_xyz_features, just with pcd in open3d format'''
        if point_cloud == self._pcd_map:  # if same reference as map
            pcd_down = self._pcd_map_down  # get reference to map
            if self._pcd_map_features is None:
                # if no features has been calcualted yet
                xyz_down, self._pcd_map_features = self.get_xyz_features(point_cloud)
                pcd_down.points = open3d.utility.Vector3dVector(xyz_down)
                pcd_down.paint_uniform_color([1, 0.706, 0])
                
            # use features that was calculated
            features = self._pcd_map_features
        else:
            pcd_down = self._pcd_scan_down  # get reference to scan
            xyz_down, features = self.get_xyz_features(point_cloud)
            pcd_down.points = open3d.utility.Vector3dVector(xyz_down)
            pcd_down.paint_uniform_color([0, 0, 0])
        return pcd_down, features

    def apply_transform(self, source, T):
        return source.transform(T)

    def eval(self):
        print("add_metrics is set to:", self._config.add_metrics)
        pass
        #self.metrics.print_all_timings()

    def reset_eval(self):
        print("add_metrics is set to:", self._config.add_metrics)
        pass
        #self.metrics.reset()

    # TODO raise errors if functions aren't defined


class MatcherTeaser(MatcherHelper):  # parent __init__ is inherited
    def Rt2T(self, R,t):
        T = np.identity(4)
        T[:3,:3] = R
        T[:3,3] = t
        return T

    def get_teaser_solver(self, noise_bound):
        solver_params = teaserpp_python.RobustRegistrationSolver.Params()
        solver_params.cbar2 = 1.0
        solver_params.noise_bound = noise_bound
        solver_params.estimate_scaling = False
        solver_params.inlier_selection_mode = \
            teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.PMC_EXACT
        solver_params.rotation_tim_graph = \
            teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN
        solver_params.rotation_estimation_algorithm = \
            teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
        solver_params.rotation_gnc_factor = 1.4
        solver_params.rotation_max_iterations = 10000
        solver_params.rotation_cost_threshold = 1e-16
        solver = teaserpp_python.RobustRegistrationSolver(solver_params)
        return solver

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

    def calc_transform(self, np_corrs_A, np_corrs_B, NOISE_BOUND=0.01):
        # robust global registration using TEASER++
        # Noise_bound is roughly voxel_size
        # sometimes if downsample is extreme noisebound should probably be smaller
        teaser_solver = self.get_teaser_solver(NOISE_BOUND)
        teaser_solver.solve(np_corrs_A, np_corrs_B)
        solution = teaser_solver.getSolution()
        R_teaser = solution.rotation
        t_teaser = solution.translation
        T_teaser = self.Rt2T(R_teaser, t_teaser)
        return T_teaser

    def find_transform_generic(self, source_down, target_down, source_fpfh, target_fpfh):
        corrs_A, corrs_B = self.find_correspondences(
            source_fpfh, target_fpfh, mutual_filter=True
        )
        np_corrs_A, np_corrs_B = self.convert_correspondences(
            source_down, target_down, corrs_A, corrs_B
        )
        #line_set = self.draw_correspondences(np_corrs_A, np_corrs_B)
        return self.calc_transform(np_corrs_A, np_corrs_B, NOISE_BOUND=0.05)


class MatcherRansac(MatcherHelper):
    def __init__(self, config, listener):
        super().__init__(config, listener)  # init parent attributes

        # init extra attributes for ransac
        self._pcd_map_features_cpu = None  
        self._pcd_scan_features_cpu = open3d.pipelines.registration.Feature()
        self._voxel_size = config.voxel_size

    def find_transform(self, source_down, target_down, source_fpfh, target_fpfh):
        if self._pcd_map_features_cpu is None:
            self._pcd_map_features_cpu = open3d.pipelines.registration.Feature()
            self._pcd_map_features_cpu.data = (target_fpfh.detach().cpu().numpy().T)#.astype(np.float64)

        self._pcd_scan_features_cpu = open3d.pipelines.registration.Feature()
        self._pcd_scan_features_cpu.data = (source_fpfh.detach().cpu().numpy().T)#.astype(np.float64)
        # print(self.pc, "\n", self.map_pc, "\n")
        # print(self.feat, "\n", self.map_feat, "\n\n")
        result_ransac = self.execute_global_registration(source_down, target_down, self._pcd_scan_features_cpu, self._pcd_map_features_cpu)

        return result_ransac.transformation

    def execute_global_registration(self, source_down, target_down, source_fpfh,
                                target_fpfh):
        distance_threshold = self._voxel_size * 0.4
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

    def find_transform_generic(self, *args):
        return self.find_transform(*args)

# TODO skip additional initial intilizations possible wtih redifinition of __new__ with __init__
# append print statements to already defined functions from parent
class _MatcherUseFaiss(MatcherTeaser): 
    # def __new__(cls, config, listener):
    #     return object.__new__(cls)

    # def __init__(self, config, listener):
    #     super().__init__(config, listener)

    def __init__(self, config, listener):
        super().__init__(config, listener)

        faiss_dimensions = 16 #TODO make this config variable?
        self.res = faiss.StandardGpuResources()  # use a single GPU
        ## Using a flat index
        self.index_flat0 = faiss.IndexFlatL2(faiss_dimensions)  # build a flat (CPU) index
        self.index_flat1 = faiss.IndexFlatL2(faiss_dimensions)  # build a flat (CPU) index
        # make it a flat GPU index
        #gpu_index_flat0 = faiss.index_cpu_to_gpu(res, 0, index_flat0)
        #gpu_index_flat1 = faiss.index_cpu_to_gpu(res, 0, index_flat1)

    def find_correspondences(self, feats0_torch, feats1_torch, mutual_filter=True):
        #Detatch from GPU to CPU: 
        # TODO: Optimise faiss for use with Torch...
        feat1 = feats0_torch.detach().cpu().numpy()
        feat0 = feats1_torch.detach().cpu().numpy()
        k = 1                          # we want to see k (here 1) nearest neighbors

        print(feat0.shape)
        print(feat1.shape)
        gpu_index_flat0 = faiss.index_cpu_to_gpu(self.res, 0, self.index_flat0)
        gpu_index_flat1 = faiss.index_cpu_to_gpu(self.res, 0, self.index_flat1)
        gpu_index_flat0.add(feat0)         # add vectors to the index
        gpu_index_flat1.add(feat1)         # add vectors to the index

        D, I = gpu_index_flat0.search(feat1, k)  # actual search
        nns01 = np.squeeze(I).T

        D, I = gpu_index_flat1.search(feat0, k)  # actual search
        nns10 = np.squeeze(I).T

        corres01_idx0 = np.arange(len(nns01))
        corres01_idx1 = nns01

        if not mutual_filter:
            return corres01_idx0, corres01_idx1

        corres10_idx1 = np.arange(len(nns10))
        corres10_idx0 = nns10

        mutual_filter = (corres10_idx0[corres01_idx1] == corres01_idx0)
        corres_idx0 = corres01_idx0[mutual_filter]
        corres_idx1 = corres01_idx1[mutual_filter]

        return corres_idx0, corres_idx1

# append print statements to already defined functions from parent
class _MatcherAddMetrics(_MatcherUseFaiss, MatcherTeaser, MatcherRansac):
    def __init__(self, _parent, config, listener):
        # store chosen parent class
        self.matcher = _parent.__class__
        
        self.matcher.__init__(self, config, listener)

        #print(_parent._parent_class, "\n\n\n")
        # else:
        #     self.matcher.__init__(self, config, listener)
        self.metrics = extensions.PerformanceMetrics()

        # def __new__(cls, _parent, config, listener):
        #     self.matcher = _parent.__class__
        #     self.metrics = extensions.PerformanceMetrics()
        #     return object.__new__(cls)

        # def __init__(self, _parent, config, listener):
        #     self.matcher.__init__(self, config, listener)

    def find_transform_generic(self, *args, **kwargs):  # define new find_transform
        self.metrics.start_time("finding transform")
        # since find_transform_generic calls parent funtions it needs to be super() and not self
        T = self.matcher.find_transform_generic(super(), *args, **kwargs)  # call parent function
        #T = self.matcher.find_transform_generic(self, *args, **kwargs)  # call parent function
        self.metrics.stop_time("finding transform")
        return T

    def convert_correspondences(self, *args):
        self.metrics.start_time("converting correspondences")
        corrs = self.matcher.convert_correspondences(self, *args)
        self.metrics.stop_time("converting correspondences")
        return corrs

    def find_correspondences(self, *args, **kwargs):
        self.metrics.start_time("finding correspondences")
        corrs = self.matcher.find_correspondences(self, *args, **kwargs)
        #rospy.loginfo("correspondences: " + str(len(corrs_A)) + " " + str(len(corrs_B)))
        self.metrics.stop_time("finding correspondences")
        return corrs

    def get_open3d_features(self, *args):
        self.metrics.start_time("processing ply")
        extrac = self.matcher.get_open3d_features(self, *args)
        self.metrics.stop_time("processing ply")
        return extrac

    def apply_transform(self, *args):
        self.metrics.start_time("apply transform")
        transformed = self.matcher.apply_transform(self, *args)
        self.metrics.stop_time("apply transform")
        return transformed

    def eval(self):
        self.metrics.print_all_timings()

    def reset_eval(self):
        self.metrics.reset()




# Link to the correct Matcher class
class Matcher():
    def __new__(self, config, listener):
        
        # select current registration type:
        if config.teaser is True:
            if config.faiss is True:
                matcher = _MatcherUseFaiss(config, listener)
            else:
                matcher = MatcherTeaser(config, listener)
        else:
            matcher = MatcherRansac(config, listener)

        if config.add_metrics is True:
            matcher = _MatcherAddMetrics(matcher, config, listener) # Make child of current matcher

        return matcher

