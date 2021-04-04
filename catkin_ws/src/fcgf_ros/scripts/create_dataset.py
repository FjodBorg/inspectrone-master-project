#!/usr/bin/env python3.7

import rosbag
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import os
import open3d as o3d


bag_dir = "/home/fjod/repos/inspectrone/catkin_ws/bags/"
bag_files = ["Kinect/groundtruth_imu_frame.bag"]
pc2_topics = ["/points2"]
odom_topics = ["/tagslam/odom/body_rig"]
dataset_dir = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/"
ply_dir = "/home/fjod/repos/inspectrone/catkin_ws/src/ply_publisher/cfg/"
ply_files = ["pcl_ballast_tank.ply", "ballast_tank.ply"]
reference = ply_files[0]

print(
    "Please run this with rosrun. LZ4 is broken otherwise"
)


def make_global_reference():
    global pcd_ref
    xyz, _ = ply2xyz(ply_dir + reference)
    print(xyz)
    pcd_ref = o3d.geometry.PointCloud()
    pcd_ref.points = o3d.utility.Vector3dVector(xyz)


def ros2xyz(pc_ros):

    pcd_ros = pc2.read_points(
        pc_ros,
        skip_nans=True,
        field_names=("x", "y", "z")
        # pcd_ros, skip_nans=True, field_names=("x", "y", "z", "rgb")
    )
    pcd_np_xyz = np.array(tuple(pcd_ros))

    # pcd_np = np.array(tuple(pcd_ros))
    # pcd_np_xyz, pcd_np_color = np.hsplit(pcd_np, [3])

    # we don't care about color
    length = pcd_np_xyz.shape[0]
    pcd_np_color = np.array([[0, 0, 0]] * length)

    return pcd_np_xyz, pcd_np_color


def ply2xyz(ply):
    pcd_o3d = o3d.io.read_point_cloud(ply)
    pcd_np_xyz = np.asarray(pcd_o3d.points)

    # we don't care about color
    length = pcd_np_xyz.shape[0]
    pcd_np_color = np.array([[0, 0, 0]] * length)

    return pcd_np_xyz, pcd_np_color


def get_choice(extension=".npz"):
    frs = ["f", "r", "s"]
    choice = frs[0]

    if any(File.endswith(extension) for File in os.listdir(dataset_dir)):
        print("files with extension .npz exists")
        while True:
            # ask user for what to do
            print("Do you want to:")
            print("    {}: Fill out missing [Default]".format(frs[0]))
            print("    {}: Replace files".format(frs[1]))
            print("    {}: Skip to next step".format(frs[2]))
            choice = input().lower().replace(" ", "")
            if choice == "" or choice == frs[0]:
                choice = frs[0]
                break
            elif choice == frs[1] or choice == frs[2]:
                break
        print("[{}] was chosen\n".format(choice))
    else:
        print(
            "No {} files found in {} \nProceeding to {} generation".format(
                extension, dataset_dir, extension
            )
        )
    return choice, frs


def generate_str_operation(file_path, choice, frs, str_suffix=""):
    skip = False

    if os.path.exists(file_path):

        # if choice was to fill
        if choice == frs[0]:
            str_prefix = "Skipping file:  "
            str_suffix = ""
            skip = True

        # if replace was choice
        else:
            str_prefix = "Replaced file:  "
            str_suffix = str_suffix

    # if file does not exist
    else:
        str_prefix = "Wrote file:     "
        str_suffix = str_suffix
    return skip, str_prefix, str_suffix


def get_bag_info(bag_file, i):
    bag = rosbag.Bag(bag_dir + bag_file)  # load bag
    bag_path_split = bag_file.split("/")  # split
    bag_file = bag_path_split[len(bag_path_split) - 1]  # select file only
    bag_prefix = bag_file.split(".")[0]  # remove extension

    seq_count = bag.get_message_count(pc2_topics[i])  # get entry count

    return bag, bag_prefix, seq_count


def local_allignment(source, target, threshold, T_rough):
    max_iter = 20
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, pcd_ref, threshold, T_rough)
    print("Fitness before:", evaluation.fitness)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, T_rough,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter))
    print("Fitness after:", reg_p2p.fitness)

    return reg_p2p.transformation
    

def in_ref_frame(xyz_np):
    # method to fix all data samples to be in the same frame
    threshold = 0.025 # TODO define this somewhere nicer
    pcd_source = o3d.geometry.PointCloud()
    pcd_source.points = o3d.utility.Vector3dVector(xyz_np)
    pcd_source.paint_uniform_color([1,0,0])

    # TODO use imu data to find rough transform
    T_rough = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])


    T_full = local_allignment(pcd_source, pcd_ref, threshold, T_rough)
    
    pcd_source.transform(T_full)
    o3d.visualization.draw([pcd_source, pcd_ref])

    return np.asarray(pcd_source.points)



def process_ply(ply_file, choice, frs):
    source = ply_file.split(".")[0]  # remove extension
    fname = source + ".npz"

    # print(fname)
    skip = False

    # check if file is there
    if os.path.exists(dataset_dir + fname):

        # if choice was to fill
        if choice == frs[0]:
            str_prefix = "Skipping file:  "
            skip = True

        # if replace was choice
        else:
            str_prefix = "Replaced file:  "

    # if file does not exist
    else:
        str_prefix = "Wrote file:     "

    # if file wasen't skipped
    if not skip:
        pcd_np_xyz, pcd_np_color = ply2xyz(ply_dir + ply_file)

        # transform to correct frame
        pcd_np_xyz_trans = in_ref_frame(pcd_np_xyz)

        np.savez(dataset_dir + fname, pcd=pcd_np_xyz_trans, color=pcd_np_xyz)

    print(str_prefix + fname)


def process_bag(source, msg, idx, seq_count, choice, frs):
    seq_str = "@seq_{:05d}".format(idx)
    fname = source + seq_str + ".npz"

    skip = False

    # check if file is there
    file_path = dataset_dir + fname
    status = "{}/{}".format(idx + 1, seq_count)
    skip, str_prefix, str_suffix = generate_str_operation(
        file_path, choice, frs, status
    )

    # if file wasen't skipped
    if not skip:
        pcd_np_xyz, pcd_np_color = ros2xyz(msg)

        # transform to correct frame
        pcd_np_xyz_trans = in_ref_frame(pcd_np_xyz)

        np.savez(dataset_dir + fname, pcd=pcd_np_xyz_trans, color=pcd_np_xyz)

    print(str_prefix + fname + "\t", str_suffix)


def create_pointcloud_dataset():
    # check if file exists
    choice, frs = get_choice(extension=".npz")


    if choice != frs[2]:  # if skip was  not selected

        # process ply's
        for i, ply_file in enumerate(ply_files):
            process_ply(ply_file, choice, frs)

        # process bags
        for i, bag_file in enumerate(bag_files):
            rosbag, bag_prefix, seq_count = get_bag_info(bag_file, i)

            pc_bag = rosbag.read_messages(topics=[pc2_topics[i]])
            odom_bag = rosbag.read_messages(topics=[odom_topics[i]])
            
            # process each pc2 entry
            for k, (topic, msg, t) in enumerate(pc_bag):
                print("pc:  ", t)
                _, msg2, t2 = next(odom_bag)
                print("look at line 242, odom:", t2, "\n", msg2)
                # TODO find a way to get imu time to the corresponding ply
                process_bag(bag_prefix, msg, k, seq_count, choice, frs)
            rosbag.close()


def generate_txt_name(batch, idx, cross_matches):
    # get source name
    # print(batch)
    source_name = batch[idx % cross_matches].split("@")[0]

    # select correct indecies
    from_idx = idx
    to_idx = from_idx + len(batch) - 1

    # full file name:
    txt_name = "{}@{:05d}-{:05d}.txt".format(source_name, from_idx, to_idx)

    # file to write to:
    txt_path = dataset_dir + txt_name

    return txt_path


def process_batch(choice, frs, idx, batch, file_targets, cross_matches):
    file_exists = False
    skip = False

    # seq = batch[i].split("@")[1]  # sequence + extension
    # idx = int(seq.split("_")[1].split(".")[0]) # idx

    # if (idx % cross_matches) == 0:
    #     # when x cross_mathces has been found

    txt_path = generate_txt_name(batch, idx, cross_matches)

    # status = "{}/{}".format(idx+1, seq_count)  # remaining files

    skip, str_prefix, str_suffix = generate_str_operation(txt_path, choice, frs)

    # print(txt_path, skip,"\n\n\n")
    if not skip:
        string = ""
        for file in batch:
            # print(str_prefix + file + "\t", str_suffix)

            for file_target in file_targets:
                # print("processing", file, "with", file_target)
                string = string + file + " vs " + file_target + "\n"
                # TODO calculate some overlap between file and file_target with open3d
                overlap = "Nan"
                # print("  overlap was:", overlap)

        f = open(txt_path, "w")
        f.write(string)
        f.close()

    print(str_prefix + txt_path.split("/")[-1] + "\t", str_suffix)
    return skip

    # print("appending", len(file_targets), "matches to", txt_file, "\n")


    # txt_name = "{}@{:05d}-{:05d}-{:0.2f}.txt".format(source_name, from_idx, to_idx, 0.5)


def create_txtfiles(choice, frs):
    # files that are target
    file_targets = tuple(ply_file.split(".")[0] + ".npz" for ply_file in ply_files)

    cross_matches = 8  # cross matches pr text file
    npz_files = [
        file
        for file in os.listdir(dataset_dir)
        if file.endswith(".npz") and not file.startswith(file_targets)
    ]
    npz_files = sorted(npz_files)
    length = len(npz_files)

    for i in range(0, length, cross_matches):
        if i < length:
            batch = npz_files[i : i + cross_matches]
        else:
            batch = npz_files[i:length]
        process_batch(choice, frs, i, batch, file_targets, cross_matches)


def create_matching_file():
    choice, frs = get_choice(extension=".txt")
    if choice != frs[2]:  # if skip was  not selected
        create_txtfiles(choice, frs)


def main():

    if not os.path.exists(dataset_dir):
        print("creating path at", dataset_dir)
        os.mkdir(dataset_dir)
  
    make_global_reference()
    create_pointcloud_dataset()
    # create_matching_file()


if __name__ == "__main__":
    main()
