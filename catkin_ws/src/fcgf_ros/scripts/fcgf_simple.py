#!/usr/bin/env python3.7
import os
import numpy as np  # NUMPY MUST COME BEFORE open3d
import open3d

import copy

# ros related
import rospy

# custom modules
from extra import extensions, ros_helper, matching_helpers, essentials


def demo():
    pcd_map = matcher.get_map()
    pcd_scan = matcher.get_scan()  # matcher.pcd_scan
    #pcd_scan.points = matcher.ros_to_open3d(listener.pc)
    # TODO find out why ballast_tank.ply is bad and ply_ballast_tank.ply is good
    #  pcd_map = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"pcl_ballast_tank.ply")

    matcher.reset_eval()
    pcd_map_down, map_features = matcher.get_open3d_features(pcd_map)

    pcd_scan_down, scan_features = matcher.get_open3d_features(pcd_scan)    

    # if config.teaser is True:
    #     corrs_A, corrs_B = matcher.find_correspondences(
    #         map_features, scan_features, mutual_filter=True
    #     )

    #     np_corrs_A, np_corrs_B = matcher.convert_correspondences(
    #         pcd_map_down, pcd_scan_down, corrs_A, corrs_B
    #     )

    #     # metrics.start_time("drawing correspondences")
    #     # line_set = matcher.draw_correspondences(np_corrs_A, np_corrs_B)
    #     # metrics.stop_time("drawing correspondences")

    #     #TODO swap around the map and scan transform order
    #     T = matcher.calc_transform(np_corrs_A, np_corrs_B, NOISE_BOUND=0.05)

    # else:
    #     T = matcher.find_transform(pcd_map_down, pcd_scan_down, map_features, scan_features)
    
    T = matcher.find_transform_generic(pcd_scan_down, pcd_map_down, scan_features, map_features)
    #matcher.publish_pose(T)
    matcher.publish_pcd(pcd_map_down)

    matcher.eval()
    #metrics.print_all_timings()
    # Visualize the registration results
    # if (config.add_metrics is True):
    #     # open3d.visualization.draw([pcd_map_down, pcd_scan_down, line_set])
    #     pcd_scan_down_T = matcher.apply_transform(copy.deepcopy(pcd_scan_down), T)
    #     open3d.visualization.draw([pcd_map_down, pcd_scan_down_T])

if __name__ == "__main__":

    global metrics, matcher, config
    config = essentials.Config()
    # Set attributes, might contain attributes not defined in Config
    setattr(config, "repos_dir", "repos/inspectrone/")
    setattr(config, "voxel_size", 0.05)
    setattr(config, "model_name", "ResUNetBN2C-16feat-3conv.pth")
    setattr(config, "static_ply_name", "pcl_ballast_tank.ply")
    setattr(config, "topic_ply", "/points_throttle")
    setattr(config, "topic_ballast_ply", "/ballest_tank")
    setattr(config, "topic_pose", "/matcher_pose")
    setattr(config, "teaser", True)
    setattr(config, "faiss", True)
    setattr(config, "add_metrics", True)  # might decrease performance by a fraction if true

    metrics = extensions.PerformanceMetrics()
    pcd_listener = ros_helper.PCListener(config.topic_ply)
    pcd_broadcaster = ros_helper.PCBroadcaster(config.topic_ballast_ply)
    pose_broadcaster = ros_helper.PoseBroadcaster(config.topic_pose)
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

    while not rospy.is_shutdown():
        if prev_red_n != pcd_listener.n:
            demo()

    rospy.spin()
