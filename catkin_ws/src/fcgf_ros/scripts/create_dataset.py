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

def generate_txt_name(batches, idxs):
    # get source name
    # print(batch)
    source_name = batches[0].split("@")[0]

    # select correct indecies
    # from_idx = idx
    # to_idx = (from_idx + len(batch) - 1)

    # full file name:
    # txt_name = "{}@{:05d}-{:05d}.txt".format(source_name, from_idx, to_idx)
    txt_name = "{}@batch_{:05d}-{:05d}.txt".format(
        source_name,
        int(idxs[1] / config.sample_size),
        int(idxs[0] / config.sample_size),
    )
    # txt_name = "{}@{:05d}.txt".format(source_name, int(idx/config.sample_size))

    # file to write to:
    file_abs = config.dataset_dir + txt_name

    return file_abs


def calc_overlap(file, file_target):
    pcd_source = o3d.geometry.PointCloud()
    pcd_target = o3d.geometry.PointCloud()
    npz_file1 = np.load(config.dataset_dir + file)
    npz_file2 = np.load(config.dataset_dir + file_target)
    xyz1 = npz_file1["pcd"]
    xyz2 = npz_file2["pcd"]
    pcd_source.points = o3d.utility.Vector3dVector(xyz1)
    pcd_target.points = o3d.utility.Vector3dVector(xyz2)
    pcd_source = pcd_source.voxel_down_sample(voxel_size=config.voxel_size)
    pcd_target = pcd_target.voxel_down_sample(voxel_size=config.voxel_size)

    pcd_combined = pcd_source + pcd_target

    pcd_merged = pcd_combined.voxel_down_sample(voxel_size=config.voxel_size)

    p_source = len(pcd_source.points)
    p_target = len(pcd_target.points)
    p_merged = len(pcd_merged.points)

    # p_rest is the number of overlapping points
    p_rest = p_source + p_target - p_merged
    p_overlap = p_rest / (p_merged)

    # if p_overlap > 0.3:
    #     print(file, file_target, p_overlap, p_source, p_target, p_merged, p_rest)

    #     o3d.visualization.draw([pcd_source.paint_uniform_color([0,0,1]), pcd_target])

    if p_source < config.min_pcd_size or p_target < config.min_pcd_size:
        print(
            "#points: ({} or {}) is less than config.min_pcd_size: {}".format(
                p_source, p_target, config.min_pcd_size
            )
        )
        return None

    # check point cloud
    min_pcd_overlap_size = int(config.min_pcd_size * 0.6)
    if p_rest < min_pcd_overlap_size:  # 5000 is probably too high
        print(
            "#points: ({}) is too few overlapping points for model training: {}".format(
                p_rest, min_pcd_overlap_size
            )
        )
        return None

    pcd_source.paint_uniform_color([1, 0, 0])
    pcd_target.paint_uniform_color([0, 1, 0])
    # o3d.visualization.draw([pcd_source, pcd_target])
    # o3d.visualization.draw([pcd_merged])

    # print("{} {} {} {} {:0.5f}".format(p_source, p_target, p_merged, p_rest, p_overlap))
    # TODO only consider it if tank to scan has a fitness above 10% maybe ?
    return p_overlap


def process_batch(choice, frs, idxs, batches):
    skip = False

    tank_batch, scan_batch = batches
    # seq = batch[i].split("@")[1]  # sequence + extension
    # idx = int(seq.split("_")[1].split(".")[0]) # idx

    # if (idx % config.sample_size) == 0:
    #     # when x cross_mathces has been found

    file_abs = generate_txt_name(scan_batch, idxs)

    # status = "{}/{}".format(idx+1, seq_count)  # remaining files

    skip, str_prefix, str_suffix = ios.generate_str_operation(file_abs, choice, frs)

    # TODO find overlap for tanks?

    # print(txt_path, skip,"\n\n\n")
    if not skip:
        string = ""
        for i, file in enumerate(scan_batch):
            # print(str_prefix + file + "\t", str_suffix)

            for file_target in tank_batch:
                overlap = calc_overlap(file, file_target)
                if overlap is not None:
                    # append to string
                    string = string + "{} {} {:0.6f}\n".format(
                        file, file_target, overlap
                    )
                # print("  overlap was:", overlap)

                if config.use_cross_match_tank:  # NOT TESTED
                    for j in range(i + 1, len(tank_batch)):
                        overlap = calc_overlap(file, tank_batch[j])
                        # print(i, j)
                        if overlap is not None:
                            # append to string
                            string = string + "{} {} {:0.6f}\n".format(
                                file, tank_batch[j], overlap
                            )

            if config.use_cross_match_scan:  # NOT TESTED
                for j in range(i + 1, len(scan_batch)):
                    overlap = calc_overlap(file, scan_batch[j])
                    # print(i, j)
                    if overlap is not None:
                        # append to string
                        string = string + "{} {} {:0.6f}\n".format(
                            file, scan_batch[j], overlap
                        )

        f = open(file_abs, "w")
        f.write(string)
        f.close()

    print(str_prefix + file_abs.split("/")[-1] + "\t", str_suffix)
    return skip

    # print("appending", len(file_targets), "matches to", txt_file, "\n")

    # txt_name = "{}@{:05d}-{:05d}-{:0.2f}.txt".format(source_name, from_idx, to_idx, 0.5)


def create_txtfiles(choice, frs):
    # tank files without extension
    tank_names = tuple(ply_file.split(".")[0] for ply_file in config.ply_files)

    scan_names = tuple(
        (bag_file.split("/")[-1]).split(".")[0] for bag_file in config.bag_files
    )

    # all scan files
    scan_files = [
        file
        for file in os.listdir(config.dataset_dir)
        if file.endswith(".npz") and file.startswith(scan_names)
    ]

    if config.use_cropping:
        # all tank files with and without cropping
        tank_files = [
            file
            for file in os.listdir(config.dataset_dir)
            if file.endswith(".npz") and file.startswith(tank_names)
        ]
    else:
        tank_files = [tank_file + ".npz" for tank_file in tank_names]

    # to be sure that it is seeded correctly:
    scan_files = sorted(scan_files)
    tank_files = sorted(tank_files)
    random.shuffle(tank_files)
    random.shuffle(scan_files)

    scan_len = len(scan_files)
    tank_len = len(tank_files)

    for j in range(0, scan_len, config.sample_size):
        if j < scan_len:
            scan_batch = scan_files[j : j + config.sample_size]
        else:
            scan_batch = scan_files[j:scan_len]

        for i in range(0, tank_len, config.sample_size):

            if i < tank_len:
                tank_batch = tank_files[i : i + config.sample_size]
            else:
                tank_batch = tank_files[i:tank_len]
            process_batch(choice, frs, (i, j), (tank_batch, scan_batch))


def create_matching_file():
    choice, frs = ios.get_choice(extension=".txt")
    if choice != frs[2]:  # if skip was  not selected
        if choice == frs[1]:
            for file in os.listdir(config.dataset_dir):
                if file.endswith(".txt"):
                    print("Deleting txt file:", file)
                    os.remove(config.dataset_dir + file)
        create_txtfiles(choice, frs)
        print("done with text generation")


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

    if not os.path.exists(config.dataset_dir):
        print("creating path at", config.dataset_dir)
        os.mkdir(config.dataset_dir)

    pcd_gen.create_pointcloud_dataset()
    create_matching_file()
    create_overlap_files()
    # TODO, add noise to npz function here


if __name__ == "__main__":
    main()
