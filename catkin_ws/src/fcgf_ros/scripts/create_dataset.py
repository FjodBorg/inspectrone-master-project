#!/usr/bin/env python3.7

import rosbag
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import os
import open3d as o3d
import tf
import copy


bag_dir = "/home/fjod/repos/inspectrone/catkin_ws/bags/"
bag_files = ["Kinect/groundtruth_imu_frame.bag"]
pc2_topics = ["/points2"]
odom_topics = ["/tagslam/odom/body_rig"]
dataset_dir = "/home/fjod/repos/inspectrone/catkin_ws/downloads/datasets/ballast_tank/"
ply_dir = "/home/fjod/repos/inspectrone/catkin_ws/src/ply_publisher/cfg/"
ply_files = ["ballast_tank.ply", "pcl_ballast_tank.ply"]
reference = ply_files[0] # use
global_counter = 0



def configure_pcd(pcd):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd.voxel_down_sample(voxel_size=voxel_size)
    #cl, ind = pcd.remove_radius_outlier(nb_points=2, radius=voxel_size*2)
    #inliers = pcd.select_by_index(ind)
    # return pcd
    #outlier_cloud = pcd.select_by_index(ind, invert=True)
    #print(ind)

def make_global_variables():
    global voxel_size
    global pcd_ref
    global skip_to_idx
    global prev_T

    prev_T = None
    skip_to_idx = 0
    voxel_size = 0.025  # must be same as config.py
    xyz, _ = ply2xyz(ply_dir + reference)

    pcd_ref = o3d.geometry.PointCloud()
    pcd_ref.points = o3d.utility.Vector3dVector(xyz)

    configure_pcd(pcd_ref)


# TODO read up on https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
def weightedAverageQuaternions(Q, w):
    # Number of quaternions to average
    M = Q.shape[0]
    A = np.zeros(shape=(4, 4))
    weightSum = 0

    for i in range(0, M):
        q = Q[i, :]
        A = w[i] * np.outer(q, q) + A
        weightSum += w[i]

    # scale
    A = (1.0/weightSum) * A

    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)

    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:, 0])


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


def local_allignment(source, target, max_iter, threshold, T_rough):
    loss = o3d.pipelines.registration.TukeyLoss(k=threshold)
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, voxel_size, T_rough)
    print("Fitness, rms before:", evaluation.fitness, evaluation.inlier_rmse)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, T_rough,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter, relative_fitness=0.99))
    print("Fitness, rms after: ", reg_p2p.fitness, reg_p2p.inlier_rmse)

    return reg_p2p.transformation
    

def to_ref_frame(xyz_np, T_rough):
    # method to fix all data samples to be in the same frame
    threshold = voxel_size  # TODO define this somewhere nicer
    max_iter = 100
    pcd_source = o3d.geometry.PointCloud()
    pcd_source.points = o3d.utility.Vector3dVector(xyz_np)
    pcd_source.paint_uniform_color([1,0,0])
    configure_pcd(pcd_source)
    
    # TODO Source if noise is relevant http://www.open3d.org/docs/0.11.0/tutorial/pipelines/robust_kernels.html#Point-to-plane-ICP-using-Robust-Kernels
    T_fine = local_allignment(pcd_source, pcd_ref, max_iter, 1, T_rough)
    T_full = local_allignment(pcd_source, pcd_ref, int(max_iter/4), threshold, T_fine)
    
    evaluation = o3d.pipelines.registration.evaluate_registration(
        pcd_source, pcd_ref, threshold, T_full)
    #print("Fitness, rms before:", evaluation.fitness, evaluation.inlier_rmse)
    
    global global_counter
    if global_counter % 50 == 1 or evaluation.fitness < 0.7:
    #if evaluation.fitness < 0.9:
        pcd_source_inbetween = copy.deepcopy(pcd_source)
        pcd_source_inbetween.transform(T_rough)
        o3d.visualization.draw([pcd_source_inbetween, pcd_ref])
        # pcd_source_inbetween2 = copy.deepcopy(pcd_source)
        # pcd_source_inbetween2.transform(T_fine)
        # o3d.visualization.draw([pcd_source_inbetween2, pcd_ref])
        pcd_source.transform(T_full)
        o3d.visualization.draw([pcd_source, pcd_ref])
    

    global_counter += 1 
    global prev_T
    prev_T = T_full

    return np.asarray(pcd_source.points)



def make_transform_from_ros(ros_pose):
    q = ros_pose.orientation
    t = ros_pose.position
    #print(q)
    # quaternion = (q.x, q.y, q.z, q.w)
    # position = np.array([t.x+100, t.y, t.z])
    
    R2 = tf.transformations.euler_matrix(np.pi/2, 0, -np.pi/2)
    
    # R = tf.transformations.quaternion_matrix(quaternion)
    # T = np.eye(4)
    # T[0:3, 3] = position
    
    # print(np.matmul(R, T))

    # position = [-0.028, -0.703, 0.955]  # rig to map
    # quaternion = [0.088, 0.991, 0.039, 0.090]

    # position = [0.747, 0.895, 0.188] # cam0 to map
    # quaternion = [-0.497, 0.558, -0.491, -0.447]
    # #quaternion = [-0.497, 0.558, -0.491, 0.447]

    # position = [-0.214, -0.775, 0.880]
    # quaternion = [0.045, 0.998, 0.010, 0.044]

    # position = [-0.214, 0.880, -0.775] # north east down if thsi configuration is used
    # #R2 = tf.transformations.euler_matrix(0, np.pi/2, -np.pi/2)
    
    # # position = [0.188, 0.747, 0.895] # cam0 to map
    # # quaternion = [ -0.7242872, 0, 0, 0.6894984 ] # -90deg around x
    # # quaternion = [ -0.4993949, 0.5245919, 0.4993949, 0.4754081 ] # -90 around x and 90 z
    # # quaternion = [ -0.4993949, 0.4993949, -0.5245919, 0.4754081 ] # -90 around x and 90 y
    # R = tf.transformations.quaternion_matrix(quaternion)
    # T = np.eye(4)
    # T[0:3, 3] = position
    
    # T_rough = (np.matmul(np.matmul(R, T), R2))
    
    # # testing

    # quaternion = [-0.497, 0.558, -0.491, 0.447]

    # # WUN
    # position = [0.188, 0.747, -0.895] # cam0 to map
    # position = [-0.895,  -0.747, -0.188] # cam0 to map
    # #quaternion = [ -0.4993949, 0.4993949, -0.5245919, 0.4754081 ] # -90 around x and 90 y
    # R = tf.transformations.quaternion_matrix(quaternion)
    # T = np.eye(4)
    # T[0:3, 3] = position
    
    # T_rough = np.matmul(R, T)


    quaternion = (q.x, q.y, q.z, q.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    position = np.array([t.x, t.y, t.z])
    position = np.add(position, np.array([0.057, 0.021, 0.011]))


    #R = tf.transformations.quaternion_matrix(quaternion)
    R = tf.transformations.euler_matrix(*euler)
    
    T = np.eye(4)
    T[0:3, 3] = position

    T_rough = np.matmul(T, np.matmul(R, R2))

    # print(euler)
    # TODO there is a bug in imu data where it doesn't change for 1 second but 
    # still publishes. This is the fix for it 
    # simply take the average of previous and current rotation 
    if prev_T is not None:
        q1 = tf.transformations.quaternion_from_matrix(prev_T)
        q2 = tf.transformations.quaternion_from_matrix(T_rough)
        Q = np.array([q1, q2])
        weights = np.array([0.8, 0.2])
        q_avg = weightedAverageQuaternions(Q, weights)
        t_avg = np.add(weights[0]*prev_T[0:3, 3],
                       weights[1]*T_rough[0:3, 3])
        R_new = tf.transformations.quaternion_matrix(q_avg)
        
        # print(T_rough)
        #T_rough = copy.copy(prev_T)
        T_rough[0:3, 0:3] = R_new[0:3, 0:3]
        T_rough[0:3, 3] = t_avg
        
        # print(T_rough)

        #T_rough = (np.matmul(T, np.matmul(R, R2)))

        # print(q1)
        # print(q2)
        # print(q_avg)
        # print(tf.transformations.euler_from_quaternion(q1))
        # print(tf.transformations.euler_from_quaternion(q2))
        # print(tf.transformations.euler_from_quaternion(q_avg))
        #euler2 = np.add(euler, prev_euler_full)/2
    #     print(prev_euler_full)
    #     print(euler2)
    # print(tf.transformations.euler_from_matrix(T_rough))
        


    # EDN for cam
    return T_rough
    
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

        T = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        
        # transform to correct frame
        
        # Add this when other is fixed
        # pcd_np_xyz_trans = to_ref_frame(pcd_np_xyz, T)
        pcd_np_xyz_trans = pcd_np_xyz

        np.savez(dataset_dir + fname, pcd=pcd_np_xyz_trans, color=pcd_np_xyz)

    print(str_prefix + fname)


def process_bag(source, msg, t, idx, seq_count, choice, frs, odom_bag):
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
        #print("pc:  ", t.to_sec())
        while True: # do while
            _, msg2, t2 = next(odom_bag)
            #print("odom:", t2.to_sec())
            print("pc - odom time diff: ", t.to_sec() - t2.to_sec())
            if t.to_sec() - t2.to_sec() < 0.15: # sampling is roughly 5 Hz
                break

        T = make_transform_from_ros(msg2.pose.pose)

        pcd_np_xyz, pcd_np_color = ros2xyz(msg)

        # transform to correct frame
        pcd_np_xyz_trans = to_ref_frame(pcd_np_xyz, T)

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
                if k < skip_to_idx:
                    print("skipping", k, "out of", skip_to_idx)
                    continue
                
                # TODO find a way to get imu time to the corresponding ply
                process_bag(bag_prefix, msg, t, k, seq_count, choice, frs, odom_bag)
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
                # overlap is equal to target on source.
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
  
    make_global_variables()
    create_pointcloud_dataset()
    # create_matching_file()


if __name__ == "__main__":
    main()
