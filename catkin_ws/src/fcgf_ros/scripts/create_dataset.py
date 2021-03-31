#!/usr/bin/env python3.7

import rosbag
import time
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import os

topic_name = '/points2'
bag_dir = "/home/fjod/repos/inspectrone/catkin_ws/bags/"
bag_files = ["Kinect/groundtruth_imu_frame.bag"]
dataset_dir = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/"
ply_files = ["pcl_ballast_tank.ply", "ballast_tank.ply"]
ply_files = ["pcl_ballast_tank.ply"] # currently only one tank


def ros2xyz(pc_ros):
    # there probably is a better way :)
    
    pc_ros = pc2.read_points(
        pc_ros, skip_nans=True, field_names=("x", "y", "z", "rgb")
    )
    # pc_ros_color = pc2.read_points(
    #     pc_ros, skip_nans=True, field_names=("rgb")
    # )
    pcd_np = np.array(tuple(pc_ros))
    pcd_np_xyz, pcd_np_color = np.hsplit(pcd_np, [3])
    
    # we don't care about the color and the offset and datatype for rgb in the rosbag seems to be wrong
    length = pcd_np_color.shape[0]
    pcd_np_color = np.array([[0, 0, 0]] * length)

    return pcd_np_xyz, pcd_np_color
    



def get_choice(extension=".npz"):
    
    frs = ["f", "r", "s"]
    choice = frs[0]
    
    if any(File.endswith(extension) for File in os.listdir(dataset_dir)):
        print("files with extension .npz exists")
        while(True):
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
        print("No {} files found in {} \nProceeding to {} generation".format(extension, dataset_dir, extension))
    return choice, frs


def process_ply(static_pc, ply_prefix, i, choice, frs):
    # seq_str = "@seq_{:05d}".format(idx)
    # fname = source + seq_str + ".npz"
    return 


def get_bag_info(bag_file):
    bag = rosbag.Bag(bag_dir + bag_file)  # load bag
    bag_path_split = bag_file.split("/")  # split
    bag_file = bag_path_split[len(bag_path_split)-1]  # select file only
    bag_prefix = bag_file.split(".")[0]  # remove extension

    seq_count = bag.get_message_count(topic_name)  # get entry count

    return bag, bag_prefix, seq_count

def process_bag(source, msg, idx, seq_count, choice, frs):
    seq_str = "@seq_{:05d}".format(idx)
    fname = source + seq_str + ".npz"

    skip = False

    # check if file is there
    if os.path.exists(dataset_dir + fname):

        # if choice was to fill
        if choice == frs[0]:
            str_prefix = "Skipping file:  "
            str_suffix = ""
            skip = True

        # if replace was choice
        else:
            str_prefix = "Replaced file:  "
            str_suffix = "{}/{}".format(idx+1, seq_count)

    # if file does not exist
    else:
        str_prefix = "Wrote file:     "
        str_suffix = "{}/{}".format(idx+1, seq_count)

    # if file wasen't skipped
    if not skip:
        pcd_np_xyz, pcd_np_color = ros2xyz(msg)
        np.savez(dataset_dir + fname, pcd=pcd_np_xyz, color=pcd_np_xyz)

    print(str_prefix + fname + "\t", str_suffix)


def create_pointcloud_dataset():
    # check if file exists
    choice, frs = get_choice(extension=".npz")
    if choice != frs[2]:  # if skip was  not selected

        # process ply's
        for i, static_pc in enumerate(ply_files):
            ply_prefix = static_pc
            process_ply(static_pc, ply_prefix, i, choice, frs)

        # process bags
        for i, bag_file in enumerate(bag_files):
            rosbag, bag_prefix, seq_count = get_bag_info(bag_file)

            # process each pc2 entry
            for k, (topic, msg, t) in enumerate(rosbag.read_messages(topics=[topic_name])):
                process_bag(bag_prefix, msg, k, seq_count, choice, frs)
            rosbag.close()

    choice, frs = get_choice(extension=".txt")
    # if choice != frs[2]:  # if skip was selected
    #     # if choice was either to fill or replace
    #     bag = rosbag.Bag(bag_dir + bag_files)
    #     static_count = len(dataset_static)
    #     for i, static_pcl in enumerate(dataset_static):

    #         static_info = "{}/{}".format(i+1, static_count)
    #         static_pcl = static_pcl.split(".")[0]
    #         scan_count = bag.get_message_count(topic_name)
    #         for k, (topic, msg, t) in enumerate(bag.read_messages(topics=[topic_name])):

    #             process_msg()
    #             time.sleep(3)
    #     bag.close()

def main():

    if not os.path.exists(dataset_dir):
        print("creating path at", dataset_dir)
        os.mkdir(dataset_dir)

    create_pointcloud_dataset()
    #create_matching_file()

    

if __name__ == "__main__":
    main()