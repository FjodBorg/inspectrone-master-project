#!/usr/bin/env python3.7
import os
import numpy as np
import argparse
import copy
import open3d as o3d
import time

from urllib.request import urlretrieve
from util.visualization import get_colored_point_cloud_feature
from util.misc import extract_features
from core.knn import find_knn_gpu

from extra.helpers import *

from model.resunet import ResUNetBN2C

import torch
import teaserpp_python

HOME_path = os.getenv("HOME")
catkin_ws_path = HOME_path + "/repos/inspectrone/catkin_ws/"
downloads_path = catkin_ws_path + "downloads/"
print(downloads_path + "ResUNetBN2C-16feat-3conv.pth")

if not os.path.isfile(downloads_path + "ResUNetBN2C-16feat-3conv.pth"):
    print("Downloading weights to ", downloads_path + "ResUNetBN2C-16feat-3conv.pth")
    urlretrieve(
        "https://node1.chrischoy.org/data/publications/fcgf/2019-09-18_14-15-59.pth",
        downloads_path + "ResUNetBN2C-16feat-3conv.pth",
    )


def pcd2xyz(pcd):
    return np.asarray(pcd.points).T


def find_correspondences(feats0, feats1, mutual_filter=True):
    nns01 = find_knn_gpu(feats0, feats1, nn_max_n=250, knn=1, return_distance=False)
    # corres01_idx0 = (torch.arange(len(nns01))
    # corres01_idx1 = (nns01.long().squeeze())
    corres01_idx0 = (torch.arange(len(nns01)).long().squeeze()).detach().cpu().numpy()
    corres01_idx1 = (nns01.long().squeeze()).detach().cpu().numpy()
    # corres01_idx0 = corres01_idx0.detach().cpu().numpy()
    # corres01_idx1 = corres01_idx1.detach().cpu().numpy()

    if not mutual_filter:
        return corres01_idx0, corres01_idx1

    nns10 = find_knn_gpu(feats1, feats0, nn_max_n=250, knn=1, return_distance=False)
    # corres10_idx1 = torch.arange(len(nns10)).long().squeeze()
    # corres10_idx0 = nns10.long().squeeze()
    corres10_idx1 = (torch.arange(len(nns10)).long().squeeze()).detach().cpu().numpy()
    corres10_idx0 = (nns10.long().squeeze()).detach().cpu().numpy()
    # corres10_idx1 = corres10_idx1.detach().cpu().numpy()
    # corres10_idx0 = corres10_idx0.detach().cpu().numpy()

    mutual_filter = corres10_idx0[corres01_idx1] == corres01_idx0
    corres_idx0 = corres01_idx0[mutual_filter]
    corres_idx1 = corres01_idx1[mutual_filter]

    return corres_idx0, corres_idx1


def demo(config):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("cuda active: ", torch.cuda.is_available())

    print(config.model)
    checkpoint = torch.load(config.model)
    model = ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()

    model = model.to(device)

    pcd = o3d.io.read_point_cloud(config.input)
    pcd_map = o3d.io.read_point_cloud(downloads_path + "cloud0.ply")

    xyz_down, feature = extract_features(
        model,
        xyz=np.array(pcd.points),
        voxel_size=config.voxel_size,
        device=device,
        skip_check=True,
    )

    xyz_down_map, feature_map = extract_features(
        model,
        xyz=np.array(pcd_map.points),
        voxel_size=config.voxel_size,
        device=device,
        skip_check=True,
    )

    map_pcd = o3d.geometry.PointCloud()
    map_pcd.points = o3d.utility.Vector3dVector(xyz_down_map)
    map_pcd.paint_uniform_color([1, 0.706, 0])
    sensor_pcd = o3d.geometry.PointCloud()
    sensor_pcd.points = o3d.utility.Vector3dVector(xyz_down)
    sensor_pcd.paint_uniform_color([0, 0, 0])

    ref = o3d.pipelines.registration.Feature()
    ref.data = feature_map.detach().cpu().numpy().T  # .astype(np.float64)
    test = o3d.pipelines.registration.Feature()
    test.data = feature.detach().cpu().numpy().T  # .astype(np.float64)

    corrs_A, corrs_B = find_correspondences(feature_map, feature, mutual_filter=True)
    A_xyz = pcd2xyz(map_pcd)  # np array of size 3 by N
    B_xyz = pcd2xyz(sensor_pcd)  # np array of size 3 by M
    A_corr = A_xyz[:, corrs_A]  # np array of size 3 by num_corrs
    B_corr = B_xyz[:, corrs_B]  # np array of size 3 by num_corrs

    num_corrs = A_corr.shape[1]
    print(f"FCGF generates {num_corrs} putative correspondences.")

    # visualize the point clouds together with feature correspondences
    points = np.concatenate((A_corr.T, B_corr.T), axis=0)
    lines = []
    for i in range(num_corrs):
        lines.append([i, i + num_corrs])
    colors = [[0, 1, 0] for i in range(len(lines))]  # lines are shown in green
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([map_pcd, sensor_pcd, line_set])

    # robust global registration using TEASER++
    NOISE_BOUND = 0.01  # config.voxel_size
    teaser_solver = get_teaser_solver(NOISE_BOUND)
    teaser_solver.solve(A_corr, B_corr)
    solution = teaser_solver.getSolution()
    R_teaser = solution.rotation
    t_teaser = solution.translation
    T_teaser = Rt2T(R_teaser, t_teaser)

    # Visualize the registration results
    map_pcd_T_teaser = copy.deepcopy(map_pcd).transform(T_teaser)
    o3d.visualization.draw_geometries([map_pcd_T_teaser, sensor_pcd])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i",
        "--input",
        default=downloads_path + "cloud1.ply",  #'sensor.ply',
        type=str,
        help="path to a pointcloud file",
    )
    parser.add_argument(
        "-m",
        "--model",
        default=downloads_path + "ResUNetBN2C-16feat-3conv.pth",
        type=str,
        help="path to latest checkpoint (default: None)",
    )
    parser.add_argument(
        "--voxel_size",
        default=0.03,
        type=float,
        help="voxel size to preprocess point cloud",
    )

    config = parser.parse_args()
    demo(config)
