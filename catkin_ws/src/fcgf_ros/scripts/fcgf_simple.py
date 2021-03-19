#!/usr/bin/env python3.7
import os
import numpy as np  # NUMPY MUST COME BEFORE open3d
import open3d

import copy

# ros related
import rospy

# custom modules
from extra import extensions, listeners, matching_helpers, essentials


def demo():
    pcd_map = matcher.get_map()
    pcd_scan = matcher.get_scan()  # matcher.pcd_scan
    #pcd_scan.points = matcher.ros_to_open3d(listener.pc)
    # TODO find out why ballast_tank.ply is bad and ply_ballast_tank.ply is good
    #  pcd_map = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"pcl_ballast_tank.ply")

    metrics.start_time("processing tank")
    pcd_map_down, map_features = matcher.get_open3d_features(pcd_map)
    metrics.stop_time("processing tank")

    metrics.start_time("processing scan")
    pcd_scan_down, scan_features = matcher.get_open3d_features(pcd_scan)
    metrics.stop_time("processing scan")

    if config.teaser is True:
        metrics.start_time("finding correspondences")
        corrs_A, corrs_B = matcher.find_correspondences(
            map_features, scan_features, mutual_filter=True
        )
        rospy.loginfo("correspondences: " + str(len(corrs_A)) + " " + str(len(corrs_B)))
        metrics.stop_time("finding correspondences")

        metrics.start_time("converting correspondences")
        np_corrs_A, np_corrs_B = matcher.convert_correspondences(
            pcd_map_down, pcd_scan_down, corrs_A, corrs_B
        )
        metrics.stop_time("converting correspondences")

        # metrics.start_time("drawing correspondences")
        # line_set = matcher.draw_correspondences(np_corrs_A, np_corrs_B)
        # metrics.stop_time("drawing correspondences")

        #TODO swap around the map and scan transform order
        metrics.start_time("calculating transform")
        T_teaser = matcher.find_transform(np_corrs_A, np_corrs_B, NOISE_BOUND=0.05)
        metrics.stop_time("calculating transform")

    else:
        metrics.start_time("calculating transform")
        T_teaser = matcher.find_transform(pcd_map_down, pcd_scan_down, map_features, scan_features)
        metrics.stop_time("calculating transform")

    metrics.start_time("apply transform")
    map_pcd_down_T_teaser = copy.deepcopy(pcd_map_down).transform(T_teaser)
    metrics.stop_time("apply transform")

    metrics.print_all_timings()
    # Visualize the registration results
    if (config.visualize is True):
        # open3d.visualization.draw([pcd_map_down, pcd_scan_down, line_set])
        open3d.visualization.draw([map_pcd_down_T_teaser, pcd_scan_down])

if __name__ == "__main__":

    global metrics, matcher, config
    config = essentials.Config(
        repos_dir="repos/inspectrone/",
        voxel_size=0.05,
        model_name="ResUNetBN2C-16feat-3conv.pth",
        static_ply_name="pcl_ballast_tank.ply",  # pcl is incomplete
        topic_ply="/points_throttle",
        teaser=True,  # TODO try with ICP
        visualize=True
    )

    metrics = extensions.PerformanceMetrics()
    listener = listeners.PointCloudListener(config)
    # if config.visualize is True:
    #     matcher = matching_helpers.MatcherVisualizer(config, listener)
    # else:
    if config.teaser is True:
        matcher = matching_helpers.Matcher(config, listener)
    else:
        matcher = matching_helpers.MatcherRansac(config, listener)

    #  updater = Main(listener)
    rospy.loginfo("start")
    while listener.pc is None:
        rospy.loginfo("No Publsihed Pointclouds Yet, trying again in 0.2 sec")
        rospy.sleep(0.2)

    prev_red_n = listener.n

    rospy.loginfo("rendering pointcloud #{}".format(prev_red_n))

    demo()

    while not rospy.is_shutdown():
        if prev_red_n != listener.n:
            demo()

    rospy.spin()
