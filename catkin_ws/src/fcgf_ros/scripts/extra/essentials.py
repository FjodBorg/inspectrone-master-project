#!/usr/bin/env python3.7
import os
import rospy


class Config:
    def __init__(self, repos_dir="repos/inspectrone/", voxel_size=0.05, model_name="ResUNetBN2C-16feat-3conv.pth", static_ply_name="pcl_ballast_tank.ply", topic_ply="/points_throttle", teaser=True, faiss=True, add_metrics=True):
        """A simple config function"""
        '''
        
        Atrributes
        ----------
        repos_dir : str
            A relative path from HOME to the the inspectrone folder
        voxel_size : double
            the voxel_size after feature extraction
        model_name : str
            name of the model inside <inspectrone>/catkin_ws/downloads
        static_ply_name : str
            name of the .ply file inside <inspectrone>/catkin_ws/src/ply_publisher/cfg/
        topic_ply : str
            name of the .ply file inside <inspectrone>/catkin_ws/src/ply_publisher/cfg/
        '''

        self.paths = self._Paths(repos_dir)
        self.voxel_size = voxel_size
        self.model_name = model_name
        self.model = self.paths.downloads + self.model_name
        self.static_ply = self.paths.plys + static_ply_name
        self.topic_ply = topic_ply
        self.add_metrics = add_metrics
        self.teaser = teaser
        self.faiss = faiss

        os.system("export OMP_NUM_THREADS=12")
        #os.environ["OMP_NUM_THREADS"] = "12"
        #print(os.environ["OMP_NUM_THREADS"])
        #os.system("echo $OMP_NUM_THREADS")
        #exit()

    class _Paths:
        def __init__(self, repos_dir):
            self.catkin_ws = repos_dir + "catkin_ws/"
            self.downloads = self.catkin_ws + "downloads/"
            self.plys = self.catkin_ws + "src/ply_publisher/cfg/"
            self._verify_path(repos_dir)
            self._verify_path(self.catkin_ws)
            self._verify_path(self.downloads)

        def _verify_path(self, path):
            if not os.path.exists(path):
                shutdown_reason = "Terminating node:  Path " + path + " does not exist. Look into config file to specify directories"
                colored_reason = "\n\n\x1b[1;37;41m" + shutdown_reason + "\x1b[0m\n\n"
                rospy.signal_shutdown(shutdown_reason) #TODO find propper way to do this
                exit(colored_reason)
                
