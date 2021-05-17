#!/usr/bin/env python3.7

import random

from extra import dataset_helpers

# new training:
"""
base_path="$HOME/repos/inspectrone"
dw_path="$base_path/catkin_ws/downloads"
python3.7 retrain.py \
--batch_size 4 \
--weights "" \
--voxel_size 0.04 \
--hit_ratio 0.075 \
--max_epoch 200 \
--threed_match_dir "$dw_path/datasets/ballast_tank/" \
--out_dir "$dw_path/retrained_models/" 
"""


def main():
    global config, ios, pcd_gen
    config = dataset_helpers.Config()
    # basic references, dirs and files
    setattr(config, "bag_dir", "/home/fjod/repos/inspectrone/catkin_ws/bags/")
    setattr(config, "bag_files", ["Kinect/groundtruth_imu_frame.bag"])
    setattr(config, "pc2_topics", ["/points2"])
    setattr(config, "odom_topics", ["/tagslam/odom/body_rig"])
    setattr(
        config,
        "dataset_dir",
        "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/",
    )
    setattr(
        config,
        "ply_dir",
        "/home/fjod/repos/inspectrone/catkin_ws/src/ply_publisher/cfg/",
    )
    setattr(config, "ply_files", ["ballast_tank.ply", "pcl_ballast_tank.ply"])
    setattr(config, "reference", config.ply_files[0])  # use

    # Matching Type
    setattr(config, "use_cross_match_scan", False)  # not tested yet
    setattr(config, "use_cross_match_tank", False)  # not tested yet
    # cropping
    setattr(config, "use_cropping", True)
    setattr(config, "use_cubic_crop", True)
    setattr(config, "max_random_crop_iterations", 100)
    # noise
    setattr(config, "use_noise", True)
    setattr(config, "max_noise_level", 0.1)  # max noise level pr meter
    setattr(config, "noise_origins", 4)  # How many origins (More noise the further away)
    only_use_noise = True  # if you don't want noiseless data

    # generation
    setattr(config, "sample_size", 8)
    setattr(config, "overlaps", [0.30, 0.50, 0.70])
    setattr(config, "overlaps_count", [0, 0, 0])
    setattr(config, "min_pcd_size", 5000)  # 5000 for voxel 0.025
    setattr(config, "voxel_size", 0.025)  # everything except 0.025 doesn't seem to work

    # misc
    setattr(config, "global_counter", 0)
    setattr(config, "skip_to_idx", 0)
    


    ios = dataset_helpers.IOS(config)
    pcd_gen = dataset_helpers.Generator_PCD(ios)
    txt_gen = dataset_helpers.Generator_txt(ios)
    match_gen = dataset_helpers.Generator_matcher(ios)
    noise_gen = dataset_helpers.Generator_noise(ios)

    ios.check_configured_path()
    pcd_gen.create_pointcloud_dataset()
    txt_gen.create_matching_file()
    if config.use_noise:
        noise_gen.create_noisy_files()
    
    match_gen.create_overlap_files()


if __name__ == "__main__":
    main()
