import os
import numpy as np
import open3d as o3d
import rosbag
import tf
import sensor_msgs.point_cloud2 as pc2
import copy
import random


class Config:
    def __init__(self):
        # every attribute is defined externally
        pass


class IOS(Config):
    def __init__(self, config):
        self.config = config

    def get_choice(self, extension=".npz"):
        frs = ["f", "r", "s"]
        choice = frs[0]

        if any(
            File.endswith(extension) for File in os.listdir(self.config.dataset_dir)
        ):
            print("files with extension {} exists".format(extension))
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
                    extension, self.config.dataset_dir, extension
                )
            )
        return choice, frs

    def ply2xyz(self, ply):
        pcd_o3d = o3d.io.read_point_cloud(ply)
        pcd_np_xyz = np.asarray(pcd_o3d.points)

        # we don't care about color
        length = pcd_np_xyz.shape[0]
        pcd_np_color = np.array([[0, 0, 0]] * length)

        return pcd_np_xyz, pcd_np_color

    def ros2xyz(self, pc_ros):

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

    def generate_str_operation(self, file_path, choice, frs, str_suffix=""):
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

    def get_bag_info(self, bag_file, i):
        bag = rosbag.Bag(self.config.bag_dir + bag_file)  # load bag
        bag_path_split = bag_file.split("/")  # split
        bag_file = bag_path_split[len(bag_path_split) - 1]  # select file only
        bag_prefix = bag_file.split(".")[0]  # remove extension

        seq_count = bag.get_message_count(self.config.pc2_topics[i])  # get entry count

        return bag, bag_prefix, seq_count


class Generator_PCD(IOS):
    def __init__(self, ios):
        self.ios = ios
        self.config = ios.config
        self.pcd_ref = o3d.geometry.PointCloud()
        self.load_pcd_ref()
        
    def load_pcd_ref(self):
        xyz, _ = self.ios.ply2xyz(self.config.ply_dir + self.config.reference)
        self.pcd_ref.points = o3d.utility.Vector3dVector(xyz)
        self.pcd_ref = self.configure_pcd(self.pcd_ref)

    def create_pointcloud_dataset(self):
        # check if file exists
        choice, frs = self.ios.get_choice(extension=".npz")
        global prev_T

        if choice != frs[2]:  # if skip was  not selected

            # process ply's
            for i, ply_file in enumerate(self.config.ply_files):
                prev_T = None
                self.process_ply(ply_file, choice, frs)

            # process bags
            for i, bag_file in enumerate(self.config.bag_files):
                prev_T = None
                rosbag, bag_prefix, seq_count = self.ios.get_bag_info(bag_file, i)

                pc_bag = rosbag.read_messages(topics=[self.config.pc2_topics[i]])
                odom_bag = rosbag.read_messages(topics=[self.config.odom_topics[i]])

                # process each pc2 entry
                for k, (topic, msg, t) in enumerate(pc_bag):
                    if k < self.config.skip_to_idx:
                        print("skipping", k, "out of", self.config.skip_to_idx)
                        continue

                    # TODO find a way to get imu time to the corresponding ply
                    self.process_bag(
                        bag_prefix, msg, t, k, seq_count, choice, frs, odom_bag
                    )
                rosbag.close()
            print("Done with dataset generation")

    def process_ply(self, ply_file, choice, frs):
        source = ply_file.split(".")[0]  # remove extension
        fname = source + ".npz"

        # print(fname)
        skip = False

        # check if file is there
        if os.path.exists(self.config.dataset_dir + fname):

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
        if not skip or self.config.use_cropping:
            pcd_np_xyz, pcd_np_color = self.ios.ply2xyz(self.config.ply_dir + ply_file)

            T = np.asarray(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )

            # transform to correct frame

            # Add this when other is fixed
            pcd_o3d_xyz_trans = self.to_ref_frame(pcd_np_xyz, T)
            pcd_np_xyz_trans = np.array(pcd_o3d_xyz_trans.points)
            # TODO should these be resampled here???
            # pcd_np_xyz_trans = pcd_np_xyz

            # always delete all noisy data
            for file in os.listdir(self.config.dataset_dir):
                if file.startswith("noisy") and source in file:
                    print("Deleting noisy file:", file)
                    os.remove(self.config.dataset_dir + file)

            np.savez(
                self.config.dataset_dir + fname,
                pcd=pcd_np_xyz_trans,
                color=pcd_np_color,
            )

            # always delete all crops
            for file in os.listdir(self.config.dataset_dir):
                if file.startswith(source + "[") and file.endswith("].npz"):
                    print("Deleting cropped file:", file)
                    os.remove(self.config.dataset_dir + file)

            if self.config.use_cropping:
                self.make_crops(pcd_o3d_xyz_trans, str_prefix, fname)

        print(str_prefix + fname)

    def process_bag(self, source, msg, t, idx, seq_count, choice, frs, odom_bag):
        seq_str = "@seq_{:05d}".format(idx)
        fname = source + seq_str + ".npz"
        global prev_msg_t
        skip = False

        # check if file is there
        file_path = self.config.dataset_dir + fname
        status = "{}/{}".format(idx + 1, seq_count)
        skip, str_prefix, str_suffix = self.ios.generate_str_operation(
            file_path, choice, frs, status
        )

        # if file wasen't skipped
        if not skip:

            # print("pc:  ", t.to_sec())
            while True:
                # exception if next can't get new element
                try:
                    _, msg2, t2 = next(odom_bag)
                except StopIteration:  # this can be made in a more smart way, but i'm lazy
                    msg2, t2 = prev_msg_t
                    break

                # print("odom:", t2.to_sec())
                print("pc - odom time diff: ", t.to_sec() - t2.to_sec())
                if t.to_sec() - t2.to_sec() < 0.15:  # sampling is roughly 5 Hz

                    prev_msg_t = msg2, t2
                    break

            T = self.make_transform_from_ros(msg2.pose.pose)

            pcd_np_xyz, pcd_np_color = self.ios.ros2xyz(msg)

            # TODO should these be resampled here???
            # transform to correct frame
            pcd_o3d_xyz_trans = self.to_ref_frame(pcd_np_xyz, T)
            pcd_np_xyz_trans = np.array(pcd_o3d_xyz_trans.points)

            np.savez(
                self.config.dataset_dir + fname,
                pcd=pcd_np_xyz_trans,
                color=pcd_np_color,
            )

        print(str_prefix + fname + "\t", str_suffix)

    def to_ref_frame(self, xyz_np, T_roughest):
        # method to fix all data samples to be in the same frame
        threshold = self.config.voxel_size  # TODO define this somewhere nicer
        max_iter = 100
        pcd_source = o3d.geometry.PointCloud()
        pcd_source.points = o3d.utility.Vector3dVector(xyz_np)
        pcd_source = pcd_source.voxel_down_sample(voxel_size=self.config.voxel_size)
        pcd_source.paint_uniform_color([1, 0, 0])
        pcd_source = self.configure_pcd(pcd_source)

        # TODO Source if noise is relevant http://www.open3d.org/docs/0.11.0/tutorial/pipelines/robust_kernels.html#Point-to-plane-ICP-using-Robust-Kernels
        T_rough = self.local_allignment(
            pcd_source, self.pcd_ref, max_iter, 1, T_roughest
        )
        T_fine = self.local_allignment(
            pcd_source, self.pcd_ref, max_iter, 0.25, T_rough
        )
        T_full = self.local_allignment(
            pcd_source, self.pcd_ref, int(max_iter / 4), threshold, T_fine
        )

        evaluation = o3d.pipelines.registration.evaluate_registration(
            pcd_source, self.pcd_ref, threshold, T_full
        )
        # print("Fitness, rms before:", evaluation.fitness, evaluation.inlier_rmse)

        # global self.config.global_counter
        # if self.config.global_counter % 200 == 1 or evaluation.fitness < 0.7:
        # #if evaluation.fitness < 0.9:
        #     pcd_source_inbetween = copy.deepcopy(pcd_source)
        #     pcd_source_inbetween.transform(T_rough)
        #     o3d.visualization.draw([pcd_source_inbetween, self.pcd_ref])
        #     pcd_source_inbetween2 = copy.deepcopy(pcd_source)
        #     pcd_source_inbetween2.transform(T_full)
        #     o3d.visualization.draw([pcd_source_inbetween2, self.pcd_ref])
        #     #o3d.visualization.draw([pcd_source, self.pcd_ref])
        # self.config.global_counter += 1

        pcd_source.transform(T_full)

        global prev_T
        prev_T = T_full

        return pcd_source

    def make_crops(self, pcd_o3d_xyz_trans, str_prefix, fname):
        aabb = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
            pcd_o3d_xyz_trans.points
        )

        min_b = aabb.get_min_bound()
        max_b = aabb.get_max_bound()

        # just to make it more pretty when making arrays
        mix, miy, miz = min_b
        mxx, may, maz = max_b

        # bounding box set
        b1 = (np.array([mix, miy, miz]), np.array([2.6, may, maz]))
        b2 = (np.array([2.4, miy, miz]), np.array([mxx, may, maz]))
        b3 = (np.array([mix, miy, miz]), np.array([mxx, 1.6, maz]))
        b4 = (np.array([mix, 1.4, miz]), np.array([mxx, may, maz]))

        # # create split crops
        b_sets = [b3, b4, b1, b2]
        for bb1, bb2 in b_sets:
            aabb = o3d.geometry.AxisAlignedBoundingBox(bb1, bb2)
            self.crop_n_save_pcd(pcd_o3d_xyz_trans, aabb, fname)

        for i in range(self.config.max_random_crop_iterations):
            aabb = self.get_random_samples(min_b, max_b)
            if not self.crop_n_save_pcd(pcd_o3d_xyz_trans, aabb, fname):
                # it failed thus try again with new sample
                i -= 1
                continue

    def make_transform_from_ros(self, ros_pose):
        q = ros_pose.orientation
        t = ros_pose.position

        # rotation from imu too cam0 frame?
        R2 = tf.transformations.euler_matrix(np.pi / 2, 0, -np.pi / 2)

        quaternion = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        position = np.array([t.x, t.y, t.z])
        position = np.add(position, np.array([0.057, 0.021, 0.011]))

        # R = tf.transformations.quaternion_matrix(quaternion)
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
            weights = np.array([0.9, 0.1])
            q_avg = self.weightedAverageQuaternions(Q, weights)
            t_avg = np.add(weights[0] * prev_T[0:3, 3], weights[1] * T_rough[0:3, 3])
            R_new = tf.transformations.quaternion_matrix(q_avg)

            T_rough[0:3, 0:3] = R_new[0:3, 0:3]
            T_rough[0:3, 3] = t_avg

        # EDN for cam
        return T_rough

    def configure_pcd(self, pcd):
        # print(pcd)
        pcd = pcd.voxel_down_sample(voxel_size=self.config.voxel_size)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        # print(pcd)
        return pcd
        # cl, ind = pcd.remove_radius_outlier(nb_points=2, radius=config.voxel_size*2)
        # inliers = pcd.select_by_index(ind)
        # return pcd
        # outlier_cloud = pcd.select_by_index(ind, invert=True)
        # print(ind)

    def local_allignment(self, source, target, max_iter, threshold, T):
        loss = o3d.pipelines.registration.TukeyLoss(k=threshold)
        evaluation = o3d.pipelines.registration.evaluate_registration(
            source, target, threshold, T
        )
        print(
            "Before:  Fitness: {:0.5f}  rms: {:0.5f}  threshold: {}".format(
                evaluation.fitness, evaluation.inlier_rmse, threshold
            )
        )
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source,
            target,
            threshold,
            T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=max_iter, relative_fitness=0.99
            ),
        )
        print(
            "After    Fitness: {:0.5f}  rms: {:0.5f}  threshold: {}".format(
                reg_p2p.fitness, reg_p2p.inlier_rmse, threshold
            )
        )

        return reg_p2p.transformation

    def crop_n_save_pcd(self, pcd_o3d_xyz_trans, aabb, fname):
        # TODO fix voxel size
        # print(aabb)
        pcd_croped = copy.deepcopy(pcd_o3d_xyz_trans)

        try:
            pcd_croped = pcd_croped.crop(aabb)

            length = len(pcd_croped.points)
            # print(length, config.min_pcd_size*2)
            if (
                length < self.config.min_pcd_size * 2
            ):  # verify that the cloud is large enough
                print("too small pcd: ", pcd_croped)
                # o3d.visualization.draw([pcd_croped.voxel_down_sample(config.voxel_size=0.05), pcd_ref])
                return None

            # o3d.visualization.draw([pcd_croped.voxel_down_sample(config.voxel_size=0.05), pcd_ref])

            pcd_np_color = np.array([[0, 0, 0]] * length)
            pcd_np_croped = np.array(pcd_croped.points)

            x1, y1, z1 = aabb.get_min_bound()
            x2, y2, z2 = aabb.get_max_bound()

            new_fname = (
                fname[0:-4] + "[{:0.2f}_{:0.2f}_{:0.2f}][{:0.2f}_{:0.2f}_{:0.2f}].npz"
            )
            new_fname = new_fname.format(x1, y1, z1, x2, y2, z2)

            np.savez(
                self.config.dataset_dir + new_fname,
                pcd=pcd_np_croped,
                color=pcd_np_color,
            )
            print("Wrote file:     " + new_fname)

            return True

        except RuntimeError:
            print("error occured, probably tried to crop section without voxels: ")
            return None

        # if something we don't know happend
        return None

    def get_random_samples(self, min_b, max_b):
        if self.config.use_cubic_crop:
            max_size_dim = np.array([3, 3, 3])
            min_size_dim = np.array([1.5, 1.5, 1.5])

            size_dim = np.array([random.uniform(min_size_dim[0], max_size_dim[0])] * 3)

        else:
            max_size_dim = max_b - min_b
            min_size_dim = np.array([1, 1, 1])
            # find random size
            min_size = self.get_size(np.array([2, 2, 2]))

            while True:
                size_dim = np.array(
                    [
                        random.uniform(min_size_dim[0], max_size_dim[0]),
                        random.uniform(min_size_dim[1], max_size_dim[1]),
                        random.uniform(min_size_dim[2], max_size_dim[2]),
                    ]
                )
                # try until a suitable size is found
                if min_size <= self.get_size(size_dim):
                    print(size_dim, ":", self.get_size(size_dim), ">=", min_size)
                    print(self.get_size(size_dim), min_size)
                    break

        # print(min_size, size, max_size)

        # calculated posible position within cloud with given size
        min_pos = min_b + size_dim / 2
        max_pos = max_b - size_dim / 2

        # generate random center within cloud
        center = np.array(
            [
                random.uniform(min_pos[0], max_pos[0]),
                random.uniform(min_pos[1], max_pos[1]),
                random.uniform(min_pos[2], max_pos[2]),
            ]
        )
        # print(center)
        # print(min_b, max_b)

        bb1 = np.round(center - size_dim / 2, 2)
        bb2 = np.round(center + size_dim / 2, 2)

        # print(bb1, bb2)
        # make allignedbox
        aabb = o3d.geometry.AxisAlignedBoundingBox(bb1, bb2)
        return aabb

    # TODO read up on https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    def weightedAverageQuaternions(self, Q, w):
        # Number of quaternions to average
        M = Q.shape[0]
        A = np.zeros(shape=(4, 4))
        weightSum = 0

        for i in range(0, M):
            q = Q[i, :]
            A = w[i] * np.outer(q, q) + A
            weightSum += w[i]

        # scale
        A = (1.0 / weightSum) * A

        # compute eigenvalues and -vectors
        eigenValues, eigenVectors = np.linalg.eig(A)

        # Sort by largest eigenvalue
        eigenVectors = eigenVectors[:, eigenValues.argsort()[::-1]]

        # return the real part of the largest eigenvector (has only real part)
        return np.real(eigenVectors[:, 0])

    def get_size(self, size_array):
        size = np.sum(size_array)  # sum size
        # print(size)
        # size = np.linalg.norm(size_array)  # diagonal size
        # print(size)
        return size


class Generator_txt(IOS):
    def __init__(self, ios):
        self.ios = ios
        self.config = ios.config

    def create_matching_file(self):
        choice, frs = self.ios.get_choice(extension=".txt")
        if choice != frs[2]:  # if skip was  not selected
            if choice == frs[1]:
                for file in os.listdir(self.config.dataset_dir):
                    if file.endswith(".txt"):
                        print("Deleting txt file:", file)
                        os.remove(self.config.dataset_dir + file)
            self.create_txtfiles(choice, frs)
            print("done with text generation")

    def create_txtfiles(self, choice, frs):
        # tank files without extension
        tank_names = tuple(ply_file.split(".")[0] for ply_file in self.config.ply_files)

        scan_names = tuple(
            (bag_file.split("/")[-1]).split(".")[0] for bag_file in self.config.bag_files
        )

        # all scan files
        scan_files = [
            file
            for file in os.listdir(self.config.dataset_dir)
            if file.endswith(".npz") and file.startswith(scan_names)
        ]

        if self.config.use_cropping:
            # all tank files with and without cropping
            tank_files = [
                file
                for file in os.listdir(self.config.dataset_dir)
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

        for j in range(0, scan_len, self.config.sample_size):
            if j < scan_len:
                scan_batch = scan_files[j : j + self.config.sample_size]
            else:
                scan_batch = scan_files[j:scan_len]

            for i in range(0, tank_len, self.config.sample_size):

                if i < tank_len:
                    tank_batch = tank_files[i : i + self.config.sample_size]
                else:
                    tank_batch = tank_files[i:tank_len]
                self.process_batch(choice, frs, (i, j), (tank_batch, scan_batch))

    def process_batch(self, choice, frs, idxs, batches):
        skip = False

        tank_batch, scan_batch = batches
        # seq = batch[i].split("@")[1]  # sequence + extension
        # idx = int(seq.split("_")[1].split(".")[0]) # idx

        # if (idx % config.sample_size) == 0:
        #     # when x cross_mathces has been found

        file_abs = self.generate_txt_name(scan_batch, idxs)

        # status = "{}/{}".format(idx+1, seq_count)  # remaining files

        skip, str_prefix, str_suffix = self.ios.generate_str_operation(file_abs, choice, frs)

        # TODO find overlap for tanks?

        # print(txt_path, skip,"\n\n\n")
        if not skip:
            string = ""
            for i, file in enumerate(scan_batch):
                # print(str_prefix + file + "\t", str_suffix)

                for file_target in tank_batch:
                    overlap = self.calc_overlap(file, file_target)
                    if overlap is not None:
                        # append to string
                        string = string + "{} {} {:0.6f}\n".format(
                            file, file_target, overlap
                        )
                    # print("  overlap was:", overlap)

                    if self.config.use_cross_match_tank:  # NOT TESTED
                        for j in range(i + 1, len(tank_batch)):
                            overlap = self.calc_overlap(file, tank_batch[j])
                            # print(i, j)
                            if overlap is not None:
                                # append to string
                                string = string + "{} {} {:0.6f}\n".format(
                                    file, tank_batch[j], overlap
                                )

                if self.config.use_cross_match_scan:  # NOT TESTED
                    for j in range(i + 1, len(scan_batch)):
                        overlap = self.calc_overlap(file, scan_batch[j])
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

    def generate_txt_name(self, batches, idxs):
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
            int(idxs[1] / self.config.sample_size),
            int(idxs[0] / self.config.sample_size),
        )
        # txt_name = "{}@{:05d}.txt".format(source_name, int(idx/config.sample_size))

        # file to write to:
        file_abs = self.config.dataset_dir + txt_name

        return file_abs

    def calc_overlap(self, file, file_target):
        pcd_source = o3d.geometry.PointCloud()
        pcd_target = o3d.geometry.PointCloud()
        npz_file1 = np.load(self.config.dataset_dir + file)
        npz_file2 = np.load(self.config.dataset_dir + file_target)
        xyz1 = npz_file1["pcd"]
        xyz2 = npz_file2["pcd"]
        pcd_source.points = o3d.utility.Vector3dVector(xyz1)
        pcd_target.points = o3d.utility.Vector3dVector(xyz2)
        pcd_source = pcd_source.voxel_down_sample(voxel_size=self.config.voxel_size)
        pcd_target = pcd_target.voxel_down_sample(voxel_size=self.config.voxel_size)

        pcd_combined = pcd_source + pcd_target

        pcd_merged = pcd_combined.voxel_down_sample(voxel_size=self.config.voxel_size)

        p_source = len(pcd_source.points)
        p_target = len(pcd_target.points)
        p_merged = len(pcd_merged.points)

        # p_rest is the number of overlapping points
        p_rest = p_source + p_target - p_merged
        p_overlap = p_rest / (p_merged)

        # if p_overlap > 0.3:
        #     print(file, file_target, p_overlap, p_source, p_target, p_merged, p_rest)

        #     o3d.visualization.draw([pcd_source.paint_uniform_color([0,0,1]), pcd_target])

        if p_source < self.config.min_pcd_size or p_target < self.config.min_pcd_size:
            print(
                "#points: ({} or {}) is less than config.min_pcd_size: {}".format(
                    p_source, p_target, self.config.min_pcd_size
                )
            )
            return None

        # check point cloud
        min_pcd_overlap_size = int(self.config.min_pcd_size * 0.6)
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


class Generator_matcher(IOS):
    pass
