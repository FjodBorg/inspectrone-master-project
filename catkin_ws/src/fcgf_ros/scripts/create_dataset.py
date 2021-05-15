#!/usr/bin/env python3.7


import numpy as np
import os
import open3d as o3d
import random

from extra import dataset_helpers


# new training:
"""
base_path="$HOME/repos/inspectrone"
dw_path="$base_path/catkin_ws/downloads"
python3.7 retrain.py \
--threed_match_dir "$dw_path/datasets/ballast_tank/" \
--out_dir "$dw_path/retrained_models/" \
--batch_size 4 \
--weights "" \
--config.voxel_size 0.04 \
--hit_ratio 0.075 \
--max_epoch 200 # kunne evt være større
"""


def make_global_variables():
    global prev_T

    prev_T = None








    # print("appending", len(file_targets), "matches to", txt_file, "\n")

    # txt_name = "{}@{:05d}-{:05d}-{:0.2f}.txt".format(source_name, from_idx, to_idx, 0.5)






def create_overlap_files():
    for file in os.listdir(config.dataset_dir):
        if file.endswith(".txt"):
            number = file.split("-")[-1].split(".txt")[0]
            if float(number) < 1.0 and number != "00000":
                # if overlap file exists
                continue

            # write base file with all config.overlaps
            f = open(os.path.join(config.dataset_dir, file), "r")
            string = f.read()
            f.close()

            for i, overlap_thr in enumerate(
                config.overlaps
            ):  # iterate through overlap thresholds
                new_string = ""
                for line in string.split("\n"):
                    try:
                        overlap = float(line.split(" ")[-1])
                        # only write items that are above the threshold
                        if overlap > overlap_thr:
                            new_string = new_string + line + "\n"
                            config.overlaps_count[i] += 1
                    except ValueError:
                        pass
                # print(new_string)

                # write files with valid thresholds
                file_overlap = "{}-{:0.2f}.txt".format(file.split(".")[0], overlap_thr)
                # print("yee,", file_overlap, len(new_string.split("\n")))
                print(
                    "Generated file with {:03d} entries: {}".format(
                        len(new_string.split("\n")) - 1, file_overlap
                    )
                )
                f = open(os.path.join(config.dataset_dir, file_overlap), "w")
                f.write(new_string)
                f.close()
    print(
        "Overlap percentages {} has these occurences {}".format(
            config.overlaps, config.overlaps_count
        )
    )
    # for overlap in config.overlaps:


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
    add_noise = True
    noise_level = 0.3  # max noise in meters
    noise_origins = 5  # How many origins (More noise the further away)
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
    random.seed(19)


    make_global_variables()

    ios = dataset_helpers.IOS(config)
    pcd_gen = dataset_helpers.Generator_PCD(ios)
    txt_gen = dataset_helpers.Generator_txt(ios)

    if not os.path.exists(config.dataset_dir):
        print("creating path at", config.dataset_dir)
        os.mkdir(config.dataset_dir)

    pcd_gen.create_pointcloud_dataset()
    txt_gen.create_matching_file()
    create_overlap_files()
    # TODO, add noise to npz function here


if __name__ == "__main__":
    main()
