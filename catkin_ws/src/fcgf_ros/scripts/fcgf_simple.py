#!/usr/bin/env python3.7
import os
import numpy as np  # NUMPY MUST COME BEFORE open3d
import open3d
import time

import copy

# ros related
import rospy

# custom modules
from extra import extensions, ros_helper, matching_helpers, essentials
from util.visualization import get_colored_point_cloud_feature

os.system("export OMP_NUM_THREADS=12")

def demo():
    matcher.reset_eval()

    pcd_map = matcher.get_map()
    pcd_scan = matcher.get_scan()  # matcher.pcd_scan
    #pcd_scan.points = matcher.ros_to_open3d(listener.pc)
    # TODO find out why ballast_tank.ply is bad and ply_ballast_tank.ply is good
    #  pcd_map = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"pcl_ballast_tank.ply")

    pcd_map_down, map_features = matcher.get_open3d_features(pcd_map)

    pcd_scan_down, scan_features = matcher.get_open3d_features(pcd_scan)

    T = matcher.find_transform_generic(pcd_scan_down, pcd_map_down, scan_features, map_features)
    stamp = matcher.publish_pcd(pcd_scan_down, pcd_map_down)
    matcher.publish_pose(T, stamp)  # TODO add correct stamp 


    matcher.eval()
    reg_qual = matcher.eval_transform(pcd_scan_down, pcd_map_down, T)
    
    matcher.feature_distance[len(matcher.feature_distance)-1][0][0] = reg_qual.fitness
    
    #metrics.print_all_timings()
    # Visualize the registration results
    if (config.debug or config.super_debug):
        if config.teaser is True:
            
            corrs_A, corrs_B = matcher.find_correspondences(
                scan_features, map_features, mutual_filter=True
            )

            np_corrs_A, np_corrs_B = matcher.convert_correspondences(
                pcd_scan_down, pcd_map_down, corrs_A, corrs_B
            )
            line_set = matcher.draw_correspondences(np_corrs_A, np_corrs_B)
        
        if config.super_debug is True:
            get_colored_point_cloud_feature(pcd_map_down,
                                            map_features.detach().cpu().numpy(),
                                            config.voxel_size)
            open3d.visualization.draw([pcd_map_down])
            get_colored_point_cloud_feature(pcd_scan_down,
                                            scan_features.detach().cpu().numpy(),
                                            config.voxel_size)
            open3d.visualization.draw([pcd_scan_down])
            
        if config.teaser:
            open3d.visualization.draw([pcd_map_down, pcd_scan_down, line_set])
        pcd_scan_down_T = matcher.apply_transform(copy.deepcopy(pcd_scan_down), T)
        open3d.visualization.draw([pcd_map_down, pcd_scan_down_T])
        

if __name__ == "__main__":

    global metrics, matcher, config
    feature_size = 32
    config = essentials.Config(
        # model_name="ResUNetBN2C-{}feat-3conv.pth".format(feature_size),
        # model_name="/retrained_models/with_cross_scan_matching/checkpoint.pth",
        # model_name="/retrained_models/with_0.025_hit_ratio/checkpoint.pth",
        model_name="/retrained_models/with_0.025_hit_ratio/best_val_checkpoint.pth",
        
        # model_name="best_val_checkpoint.pth",
        # model_name="retrained_models/best_val_checkpoint.pth",
        repos_dir=os.getenv("HOME")+"/repos/inspectrone/",
        static_ply_name="ballast_tank.ply",
        #static_ply_name="pcl_ballast_tank.ply",
        )
    # Set attributes, might contain attributes not defined in Config
    setattr(config, "feature_size", feature_size)
    # setattr(config, "voxel_size", 0.06)
    # setattr(config, "voxel_size", 0.08)
    setattr(config, "voxel_size", 0.025) # try with this # can't do matching properly
    setattr(config, "voxel_size", 0.06) # try with this 
    setattr(config, "NOISE_BOUND", config.voxel_size)
    setattr(config, "topic_in_ply", "/points_in")
    setattr(config, "topic_ballast_ply", "/ballest_tank")
    setattr(config, "topic_scan_ply", "/scan_ply")
    setattr(config, "topic_pose", "/matcher_pose")
    setattr(config, "teaser", True)
    setattr(config, "faiss", True) # teaser false needs add_metrics false
    setattr(config, "add_metrics", True)  # might decrease performance by a fraction if true
    setattr(config, "debug", False)  # show matches
    setattr(config, "super_debug", False)  # VERY SLOW increase voxel_size for speed up



    metrics = extensions.PerformanceMetrics()
    pcd_listener = ros_helper.PCListener(config.topic_in_ply)
    pcd_broadcaster = ros_helper.PCBroadcaster(config.topic_ballast_ply, config.topic_scan_ply)
    # pcd_scan_broadcaster = ros_helper.PCBroadcaster(config.topic_ply)
    pose_broadcaster = ros_helper.PoseBroadcaster(config.topic_pose, frame_id="scan")
    #pose_broadcaster = ros_helper.PoseBroadcaster(config.topic_pose, frame_id="camera_base")
    ros_col = ros_helper.Collect(pcd_listener, pcd_broadcaster, pose_broadcaster)
    #ros_col = ros_helper.Collect(pcd_listener, pcd_broadcaster, pose_broadcaster)

    matcher = matching_helpers.Matcher(config, ros_col)

    #  updater = Main(listener)
    rospy.loginfo("start")
    while pcd_listener.pc is None:
        rospy.loginfo("No Publsihed Pointclouds Yet, trying again in 0.2 sec")
        rospy.sleep(0.2)

    prev_red_n = pcd_listener.n

    rospy.loginfo("rendering pointcloud #{}".format(prev_red_n))

    demo()

    prev_time = time.time()
    while not rospy.is_shutdown():
        if prev_red_n != pcd_listener.n:
            prev_red_n = pcd_listener.n
            demo()
            prev_time = time.time()
        
        if time.time() - prev_time > 2:
            if config.debug:
                with np.printoptions(precision=3, suppress=True, linewidth=160, threshold=16000):
                    print(np.vstack(matcher.feature_distance))
            rospy.loginfo("No Publsihed Pointclouds, trying again in 0.2 sec")
            rospy.sleep(0.2)

    rospy.spin()
