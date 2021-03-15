#!/usr/bin/env python3.7
import os
import numpy as np  # NUMPY MUST COME BEFORE open3d
import open3d

import copy

# ros related
import rospy

# custom modules
from extra import extensions, listeners, matching_helpers, essentials


def demo(listener):
    pcd_scan = matcher.pcd_scan
    pcd_scan.points = matcher.ros_to_open3d(listener.pc)
    metrics.start_time("loading tank")
    # TODO find out why ballast_tank.ply is bad and ply_ballast_tank.ply is good
    #  pcd_map = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"pcl_ballast_tank.ply")
    pcd_map = matcher.get_map()
    metrics.stop_time("loading tank")

    metrics.start_time("processing tank")
    xyz_down_map, feature_map = matcher.get_features(pcd_map)

    map_pcd = open3d.geometry.PointCloud()
    map_pcd.points = open3d.utility.Vector3dVector(xyz_down_map)
    map_pcd.paint_uniform_color([1, 0.706, 0])

    metrics.stop_time("processing tank")

    metrics.start_time("processing scan")
    # pcd = open3d.io.read_point_cloud(catkin_ws_path+'src/ply_publisher/cfg/'+"sensor_tank1.ply")
    xyz_down, feature = matcher.get_features(pcd_scan)
    sensor_pcd = open3d.geometry.PointCloud()
    sensor_pcd.points = open3d.utility.Vector3dVector(xyz_down)
    sensor_pcd.paint_uniform_color([0, 0, 0])
    metrics.stop_time("processing scan")

    metrics.start_time("finding correspondences")
    corrs_A, corrs_B = matcher.find_correspondences(
        feature_map, feature, mutual_filter=True
    )
    metrics.stop_time("finding correspondences")

    metrics.start_time("converting correspondences")
    np_corrs_A, np_corrs_B = matcher.convert_correspondences(
        map_pcd, sensor_pcd, corrs_A, corrs_B
    )
    metrics.stop_time("converting correspondences")

    metrics.start_time("drawing correspondences")
    line_set = matcher.draw_correspondences(np_corrs_A, np_corrs_B)
    metrics.stop_time("drawing correspondences")

    metrics.start_time("calculating transform")
    T_teaser = matcher.find_transform(np_corrs_A, np_corrs_B, NOISE_BOUND=0.05)
    metrics.stop_time("calculating transform")

    # Visualize the registration results
    metrics.print_all_timings()

    map_pcd_T_teaser = copy.deepcopy(map_pcd).transform(T_teaser)
    open3d.visualization.draw([map_pcd, sensor_pcd, line_set])
    open3d.visualization.draw([map_pcd_T_teaser, sensor_pcd])


if __name__ == "__main__":

    global metrics, matcher
    config = essentials.Config(
        repos_dir="repos/inspectrone/",
        voxel_size=0.05,
        model_name="ResUNetBN2C-16feat-3conv.pth",
        static_ply_name="pcl_ballast_tank.ply",
        topic_ply="/points_throttle",
    )

    metrics = extensions.PerformanceMetrics()
    listener = listeners.PointCloudListener(config)
    matcher = matching_helpers.Matcher(config)

    #  updater = Main(listener)
    rospy.loginfo("start")
    while listener.pc is None:
        rospy.loginfo("No Publsihed Pointclouds Yet, trying again in 0.2 sec")
        rospy.sleep(0.2)

    prev_red_n = listener.n

    rospy.loginfo("rendering pointcloud #{}".format(prev_red_n))

    demo(listener)

    while not rospy.is_shutdown():
        if prev_red_n != listener.n:
            demo(listener)

    rospy.spin()
