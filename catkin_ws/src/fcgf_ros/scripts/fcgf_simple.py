#!/usr/bin/env python3.7
import os
import numpy as np  # NUMPY MUST COME BEFORE open3d
import open3d
import time
import matplotlib.pyplot as plt

import copy

# ros related
import rospy

from extra.ros_helper import setBool 

# custom modules
from extra import extensions, ros_helper, matching_helpers, essentials
from util.visualization import get_colored_point_cloud_feature

os.system("export OMP_NUM_THREADS=12")


use_real_time = False

def movingaverage(interval, window_size):
    window = np.ones(int(window_size))/float(window_size)
    return np.convolve(interval, window, 'same')

def demo_func():
    # setBool(use_real_time)
    matcher.reset_eval()

    pcd_map = matcher.get_map()
    pcd_scan = matcher.get_scan()  # matcher.pcd_scan
    #pcd_scan.points = matcher.ros_to_open3d(listener.pc)
    # TODO find out why ballast_tank.ply is bad and ply_ballast_tank.ply is good
    #  pcd_map = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"pcl_ballast_tank.ply")

    pcd_map_down, map_features = matcher.get_open3d_features(pcd_map)

    pcd_scan_down, scan_features = matcher.get_open3d_features(pcd_scan)
    T = matcher.find_transform_generic(pcd_scan_down, pcd_map_down, scan_features, map_features)

    reg_qual = matcher.eval_transform(pcd_scan_down, pcd_map_down, T)
    # if cloud is fit enough
    if rospy.get_param("/fcgf/fitness_thr") < reg_qual.fitness:
        matcher.publish_pcds(pcd_scan_down, pcd_map_down)
        # setBool(False)
        matcher.publish_transform(T)  # TODO add correct stamp 
    # setBool(False)

    matcher.eval()
    
    #metrics.print_all_timings()
    # Visualize the registration results
    if (config.debug_viz or config.super_debug or config.debug_calc_feat_dist):
        if config.super_debug is True:
            get_colored_point_cloud_feature(pcd_map_down,
                                            map_features.detach().cpu().numpy(),
                                            config.voxel_size)
            open3d.visualization.draw([pcd_map_down])
            get_colored_point_cloud_feature(pcd_scan_down,
                                            scan_features.detach().cpu().numpy(),
                                            config.voxel_size)
            open3d.visualization.draw([pcd_scan_down])
         
        if config.teaser and config.debug_calc_feat_dist:
            matcher.avg_feat_dist[len(matcher.avg_feat_dist)-1][0][0] = reg_qual.fitness

        if config.teaser and config.debug_viz:
            
            corrs_A, corrs_B = matcher.find_correspondences(
                scan_features, map_features, mutual_filter=True
            )

            np_corrs_A, np_corrs_B = matcher.convert_correspondences(
                pcd_scan_down, pcd_map_down, corrs_A, corrs_B
            )
            line_set, feat_dists = matcher.draw_correspondences(np_corrs_A, np_corrs_B)
            open3d.visualization.draw([pcd_map_down, pcd_scan_down])
            open3d.visualization.draw([pcd_map_down, pcd_scan_down, line_set])
                
        pcd_scan_down_T = matcher.apply_transform(copy.deepcopy(pcd_scan_down), T)
        
        if config.teaser and config.debug_calc_feat_dist:

            plt.figure(1)
            plt.plot(feat_dists, "r.")
            plt.plot(feat_dists_T, "b.")
            x_av = movingaverage(feat_dists, 50)
            x_av_T = movingaverage(feat_dists_T, 50)
            plt.plot(x_av, "k-", linewidth=4)
            plt.plot(x_av_T, "y-", linewidth=4)
            plt.ylabel('Distance[m]')
            plt.xlabel('Correspondence Index')
            plt.legend(["Before", "After", "Avg Before", "Avg After"])
            
            plt.figure(2)
            plt.boxplot([feat_dists, feat_dists_T])
            plt.ylabel('Distance[m]')
            # plt.xlabel('Correspondence Index')
            my_xticks = ['Before','After']
            x = [1, 2]
            plt.xticks(x, my_xticks)
            #plt.legend(["Before", "After"])
            plt.show()

        if config.teaser and config.debug_viz:
            np_corrs_A, np_corrs_B = matcher.convert_correspondences(
                pcd_scan_down_T, pcd_map_down, corrs_A, corrs_B
            )
            line_set_T, feat_dists_T = matcher.draw_correspondences(np_corrs_A, np_corrs_B)

            open3d.visualization.draw([pcd_map_down, pcd_scan_down_T])
            open3d.visualization.draw([pcd_map_down, pcd_scan_down_T, line_set_T])

    
    
    
        
        

if __name__ == "__main__":

    global metrics, matcher, config
    feature_size = 32
    # model_name="best_val_checkpoint.pth",
    # model_name="/retrained_models/best_val_checkpoint.pth",
    model_name="ResUNetBN2C-{}feat-3conv.pth".format(feature_size)
    # model_name="/retrained_models/with_cross_scan_matching/checkpoint.pth",
    # model_name="/retrained_models/with_0.025_hit_ratio/checkpoint.pth",
    # model_name="/retrained_models/with_0.025_hit_ratio/best_val_checkpoint.pth"
    # model_name = "/retrained_models/with_cropping_0.025_hit_ratio/best_val_checkpoint.pth"
    # model_name = "/retrained_models/with_cropping_0.025_hit_ratio/checkpoint.pth"
    # model_name="/retrained_models/with_cropping_0.075_hit_ratio/checkpoint.pth", # Works well
    # model_name="/retrained_models/with_cropping_0.075_hit_ratio/best_val_checkpoint.pth",
    # model_name="/retrained_models/with_cropping_0.075_hit_ratio_100_crops/checkpoint.pth" # Works well
    # model_name="/retrained_models/with_cropping_0.075_hit_ratio_100_crops/best_val_checkpoint.pth",
    # model_name="/retrained_models/with_cropping_0.125_hit_ratio_100_square_crops/best_val_checkpoint.pth",
    # model_name="/retrained_models/with_cropping_0.125_hit_ratio_100_square_crops/checkpoint.pth",
    model_name="/retrained_models/with_cropping_0.075_hit_ratio_100_square_crops/checkpoint.pth" # WORks very well with new technique 
    model_name="/retrained_models/with_cropping_0.075_hit_ratio_100_square_crops_0.04_voxel/checkpoint.pth"
    # model_name="/retrained_models/with_cropping_0.075_hit_ratio_100_square_crops_0.04_voxel_16_feat/checkpoint.pth"
    # model_name="/retrained_models/with_cropping_0.075_hit_ratio_100_square_crops_0.04_voxel/best_val_checkpoint.pth"
    # model_name="/retrained_models/with_cropping_0.075_hit_ratio_100_square_crops_0.04_voxel_300_epochs/checkpoint.pth"
    # model_name="/retrained_models/with_cropping_0.075_hit_ratio_100_square_crops_0.04_voxel_300_epochs_from_scratch/checkpoint.pth"
    config = essentials.Config(
        repos_dir=os.getenv("HOME")+"/repos/inspectrone/",
        model_name=model_name,
        static_ply_name="ballast_tank.ply",
        #static_ply_name="pcl_ballast_tank.ply",
        )
    # Set attributes, might contain attributes not defined in Config
    setattr(config, "feature_size", feature_size)
    # setattr(config, "voxel_size", 0.06)
    # setattr(config, "voxel_size", 0.08)
    setattr(config, "voxel_size", 0.025) # try with this # can't do matching properly
    setattr(config, "voxel_size", 0.04) # try with this 
    setattr(config, "NOISE_BOUND", config.voxel_size * 5)
    setattr(config, "topic_in_ply", "/points_in")
    setattr(config, "topic_ballast_ply", "/ballest_tank")
    setattr(config, "topic_scan_ply", "/scan_ply")
    setattr(config, "topic_pose", "/matcher_pose")
    setattr(config, "teaser", True )
    setattr(config, "faiss", True)  # teaser false needs add_metrics false
    setattr(config, "add_metrics", True)  # might decrease performance by a fraction if true
    setattr(config, "super_debug", False)  # VERY SLOW increase voxel_size for speed up
    setattr(config, "debug_viz", False)  # visualized the match
    setattr(config, "debug_calc_feat_dist", False)  # calculates the distance of each feature correspondence (Visualized with debug_viz True)
    setattr(config, "limit_max_correspondences", 1000)  # <= 0 means don't limit
    setattr(config, "limit_corr_type", 0)  # <= 0 random, 1 best
    setattr(config, "covariance_type", 1)  # 0=Outlier_based, 1=Fitness_based (default)

    # metrics = extensions.PerformanceMetrics()
    ros_helper.Inits()
    pcd_listener = ros_helper.PCListener(config.topic_in_ply)
    pcd_broadcaster = ros_helper.PCBroadcaster(config.topic_ballast_ply, config.topic_scan_ply)
    pose_broadcaster = ros_helper.PoseBroadcaster(config.topic_pose, frame_id="scan")
    ros_col = ros_helper.Collect(pcd_listener, pcd_broadcaster, pose_broadcaster)
    #ros_col = ros_helper.Collect(pcd_listener, pcd_broadcaster, pose_broadcaster)

    matcher = matching_helpers.Matcher(config, ros_col)
    

    #  updater = Main(listener)
    rospy.loginfo("start")
    rospy.set_param('/fcgf/fitness_thr', 0)

    matcher.publish_inital_map()
    while pcd_listener.pc is None:
        rospy.loginfo("No Publsihed Pointclouds Yet, trying again in 0.05 sec")
        while(True):
            try:
                rospy.sleep(0.05)
                break
            except rospy.ROSTimeMovedBackwardsException:
                pass
            except rospy.ROSInterruptException:
                # if e.g ctrl + c
                break

    prev_red_n = pcd_listener.n

    rospy.loginfo("rendering pointcloud #{}".format(prev_red_n))

    demo_func()

    prev_time = time.time()
    while not rospy.is_shutdown():
        if prev_red_n != pcd_listener.n:
            prev_red_n = pcd_listener.n
            demo_func()
            prev_time = time.time()
        
        if time.time() - prev_time > 2:
            if config.debug_calc_feat_dist:
                with np.printoptions(precision=3, suppress=True, linewidth=160, threshold=16000):
                    print(np.vstack(matcher.avg_feat_dist))
            rospy.loginfo("No published Pointclouds, trying again in 0.05 sec")
            while(True):
                try:
                    rospy.sleep(0.05)
                    break
                except rospy.ROSTimeMovedBackwardsException:
                    pass
                except rospy.ROSInterruptException:
                    matcher.metrics.f.close() # hotfix for teaser metrics
                    # if e.g ctrl + c
                    break

    rospy.spin()
