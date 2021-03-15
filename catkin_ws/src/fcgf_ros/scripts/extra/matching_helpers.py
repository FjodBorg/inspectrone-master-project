#!/usr/bin/env python3.7
import os
import numpy as np
import open3d
import rospy

# pytorch
import model as mdl
import torch
from util.misc import extract_features

# teaser
from core.knn import find_knn_gpu

downloads_path = ""
voxel_size = 0


class Matcher():
    def __init__(self, config):
        self.config = config
        self.voxel_size = config.voxel_size
        
        self.pcd_map = self._load_static_ply(config)
        self.model, self.device = self._load_model(config)

    def _load_model(self, config):
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo("Using Cuda: " + str(torch.cuda.is_available()))
        #  model_file = downloads_path + "ResUNetBN2C-16feat-3conv.pth"
        rospy.loginfo("resnet location at: " + config.model)

        if not os.path.isfile(config.model):
            if not os.path.exists(config.path.do):
                os.makedirs(downloads_path)
            
            rospy.loginfo("Downloading weights to ", config.model)
            urlretrieve(
                "https://node1.chrischoy.org/data/publications/fcgf/2019-09-18_14-15-59.pth",
                downloads_path + "ResUNetBN2C-16feat-3conv.pth",
            )

        checkpoint = torch.load(config.model)
        model = mdl.resunet.ResUNetBN2C(1, 16, normalize_feature=True, conv1_kernel_size=3, D=3)
        model.load_state_dict(checkpoint['state_dict'])
        model.eval()

        model = model.to(device)

        return model, device

    def _load_static_ply(self, config):
        pcd_map = open3d.io.read_point_cloud(config.static_ply)
        return pcd_map

    def get_map(self):
        return self.pcd_map

    #def load_ply(self, config)


    def get_features(self, point_cloud):
        xyz_down, feature = extract_features(
            self.model,
            xyz=np.array(point_cloud.points),
            voxel_size=self.voxel_size,
            device=self.device,
            skip_check=True)
        return xyz_down, feature

    def find_correspondences(self, feats0, feats1, mutual_filter=True):
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
        # corres10_idx1 = (torch.arange(len(nns10)).long().squeeze()).detach().cpu().numpy()
        corres10_idx0 = (nns10.long().squeeze()).detach().cpu().numpy()
        # corres10_idx1 = corres10_idx1.detach().cpu().numpy()
        # corres10_idx0 = corres10_idx0.detach().cpu().numpy()

        mutual_filter = corres10_idx0[corres01_idx1] == corres01_idx0
        corres_idx0 = corres01_idx0[mutual_filter]
        corres_idx1 = corres01_idx1[mutual_filter]

        return corres_idx0, corres_idx1
