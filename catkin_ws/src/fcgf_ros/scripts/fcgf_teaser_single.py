#!/usr/bin/env python3.7
import os
import glob
import open3d
import numpy as np
import time

from urllib.request import urlretrieve
from util.visualization import get_colored_point_cloud_feature # needed on jetson, otherwise it cant render images 

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
        #self.vis.create_window()
        #self.vis.add_geometry(self.open3d_map)
        #self.vis.update_geometry(self.open3d_map)

        # add point cloud to visualization
        #self.vis.add_geometry(self.open3d_pc)
        #self.vis.update_geometry(self.open3d_pc)
        #self.vis.poll_events()
        #self.vis.update_renderer()
        while not rospy.is_shutdown():
            if self.prev_red_n != self.listener.n:
                #self.prev_red_n = self.listener.n
                self.open3d_pc.points = self.matcher.ros_to_open3d(self.listener.pc)
                #teest = copy.deepcopy(self.open3d_pc).translate((1.3*self.prev_red_n, 0, 0))
                #rospy.loginfo("Calculating transform")
                
                transformation = self.matcher.find_transform(self.open3d_pc, self.open3d_map)
                #self.open3d_pc.transform(transformation)
                #self.vis.update_geometry(self.open3d_pc)
                rospy.loginfo("Rendering transformed pointcloud #{}".format(self.prev_red_n))
                # self.vis.add_geometry(self.open3d_pc)
            #self.vis.poll_events()
            #self.vis.update_renderer()

            #rospy.loginfo(self.listener.n)


class PcMatcher:
    def __init__(self, broadcaster, parent=None):
        rospy.loginfo("initialization Matching method")
        self.model, self.checkpoint, self.device = self.load_checkpoint()
        self.voxel_size = voxel_size
        self.map_pc = open3d.geometry.PointCloud()
        self.map_feat = open3d.pipelines.registration.Feature()
        self.pc = open3d.geometry.PointCloud()
        self.feat = open3d.pipelines.registration.Feature()
        self.broadcaster = broadcaster

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

    def find_transform_ransac(self, point_cloud, global_map):
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



    def find_transform(self, point_cloud, global_map):
        if len(self.map_pc.points) == 0:  # this depends if we have the whole global map or not
            metrics.start_time("loading tank")
            map_pc, self.map_feat = self.get_features(global_map)

            self.map_pc.points = open3d.utility.Vector3dVector(map_pc)
            self.map_pc.paint_uniform_color([1, 0.706, 0])
            #self.map_feat.data = (map_feat.detach().cpu().numpy().T)#.astype(np.float64)
            metrics.stop_time("loading tank")

        metrics.start_time("loading ply")
        pc, self.feat = self.get_features(point_cloud)

        self.pc.points = open3d.utility.Vector3dVector(pc)
        self.pc.paint_uniform_color([0, 0, 0])
        #self.feat.data = (feat.detach().cpu().numpy().T)#.astype(np.float64)
        metrics.stop_time("loading ply")






        metrics.start_time("teaser")
        
        corrs_A, corrs_B = self.find_correspondences(self.map_feat, self.feat, mutual_filter=True)
        A_xyz = self.pcd2xyz(self.map_pc)  # np array of size 3 by N
        B_xyz = self.pcd2xyz(self.pc)  # np array of size 3 by M
        A_corr = A_xyz[:, corrs_A]  # np array of size 3 by num_corrs
        B_corr = B_xyz[:, corrs_B]  # np array of size 3 by num_corrs

        # finding lines for correlation
        num_corrs = A_corr.shape[1]
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
        open3d.visualization.draw([self.map_pc, self.pc, line_set])
        
        #print(f"FCGF generates {num_corrs} putative correspondences.")

        # robust global registration using TEASER++
        NOISE_BOUND = 0.01  # config.voxel_size
        teaser_solver = get_teaser_solver(NOISE_BOUND)
        teaser_solver.solve(A_corr, B_corr)
        solution = teaser_solver.getSolution()
        R_teaser = solution.rotation
        t_teaser = solution.translation
        T_teaser = Rt2T(R_teaser, t_teaser)

        
        map_pcd_T_teaser = copy.deepcopy(self.map_pc).transform(T_teaser)
        metrics.stop_time("teaser")
        
        # Visualize the registration results
        open3d.visualization.draw([map_pcd_T_teaser, self.pc])
        



        metrics.print_time(["loading tank", "loading ply", "teaser"])

        self.broadcaster.T = T_teaser
        return T_teaser

    def find_correspondences(self, feats0, feats1, mutual_filter=True):
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
        #corres10_idx1 = (torch.arange(len(nns10)).long().squeeze()).detach().cpu().numpy()
        corres10_idx0 = (nns10.long().squeeze()).detach().cpu().numpy()
        # corres10_idx1 = corres10_idx1.detach().cpu().numpy()
        # corres10_idx0 = corres10_idx0.detach().cpu().numpy()

        mutual_filter = corres10_idx0[corres01_idx1] == corres01_idx0
        corres_idx0 = corres01_idx0[mutual_filter]
        corres_idx1 = corres01_idx1[mutual_filter]

        return corres_idx0, corres_idx1

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

    def pcd2xyz(self, pcd):
        return np.asarray(pcd.points).T

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


class PoseBroadcaster:
    def __init__(self):
        self.T = None
        self.p = geometry_msgs.msg.PoseStamped()
        self.t = geometry_msgs.msg.TransformStamped()
        self.br = tf2_ros.TransformBroadcaster()
        self.pub = rospy.Publisher('teaser_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tf_buffer)

    def publish_pose(self):
        time_now = rospy.Time.now()
        self.t.header.stamp = time_now  # data.header.stamp
        self.t.header.frame_id = "map"
        self.t.child_frame_id = "imu_link"  # data.header.frame_id
        self.t.transform.translation.x = self.T[0, 3]
        self.t.transform.translation.y = self.T[1, 3]
        self.t.transform.translation.z = self.T[2, 3]

        R = self.T[0:3, 0:3]
        q = tf_conversions.transformations.quaternion_from_matrix(R)
    
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        
        #while True:
        self.br.sendTransform(self.t)

        rospy.sleep(rospy.Duration(0.01)) # Wait a bit before trying for the lookup

        try:
            trans = self.tfBuffer.lookup_transform('map', 'imu_link', time_now)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("tf Exception..")
            return 0

        self.p.header.stamp = time_now
        self.p.header.frame_id = 'map'
        self.p.pose.position.x = trans.transform.translation.x
        self.p.pose.position.y = trans.transform.translation.y
        self.p.pose.position.z = trans.transform.translation.z
        # Make sure the quaternion is valid and normalized
        self.p.pose.orientation.x = trans.transform.rotation.x
        self.p.pose.orientation.y = trans.transform.rotation.y
        self.p.pose.orientation.z = trans.transform.rotation.z
        self.p.pose.orientation.w = trans.transform.rotation.w
        self.pub.publish(self.p)


if __name__ == "__main__":
    global metrics
    metrics = PerformanceMetric()

    listener = PcListner()
    broadcaster = PoseBroadcaster()
    matcher = PcMatcher(broadcaster)
    updater = Main(listener, matcher)
    rospy.spin()
