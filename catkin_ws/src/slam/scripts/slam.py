#!/usr/bin/env python3.7
import numpy as np 
# import g2oclasses
import g2o
import tf 

from g2oclasses import PoseGraphOptimization

import rospy
import geometry_msgs.msg
import nav_msgs.msg

# https://github.com/UditSinghParihar/g2o_tutorial
# https://github.com/uoip/g2opy/blob/master/python/examples/ba_demo.py
# https://github.com/uoip/g2opy/blob/master/g2o/what_is_in_these_directories.txt
# https://github.com/uoip/g2opy/blob/master/g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp
# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/plane_slam/simulator_3d_plane.cpp

# https://github.com/RainerKuemmerle/g2o/blob/master/doc/g2o.pdf

class OdomInfo():
    def __init__(self) -> None:
        self.pos = None
        self.quad = None
        self.covar = None
    
    def read_values():
        pass

class Slam():
    def __init__(self) -> None:
        self.time_stamps = []
        self.start_time = None
        self.index = 0
        self.graph = PoseGraphOptimization()

        self.odom = OdomInfo()

    def odomCB(self, data):
        # rospy.loginfo("I heard %s", data.pose.pose)
        # rospy.loginfo("I heard %s", data.header.stamp)
        pos = data.pose.pose.position
        quad = data.pose.pose.orientation
        abs_time_stamp = (data.header.stamp)
        if self.start_time is None:
            self.start_time = abs_time_stamp
        # Index is the time stamp multipled with 1000 (needs to be int)
        time_stamp = int((abs_time_stamp - self.start_time).to_sec() * 1000)
        # print(start_time)
        print(time_stamp)
        self.time_stamps.append(time_stamp)
        covar = np.array(data.pose.covariance).reshape(6,6)

        # pose_np = np.array([pos.x, pos.y, pos.z, quad.x, quad.y, quad.z, quad.w])
        # pose = g2o.SE3Quat(pose_np)
        # pose_np = np.array([pos.x, pos.y, pos.z, quad.x, quad.y, quad.z, quad.w])
        # pose = g2o.SE3Quat(pose_np)

        pos_np = [pos.x, pos.y, pos.z]
        quad_np = [quad.x, quad.y, quad.z, quad.w]
        rot = tf.transformations.quaternion_matrix(quad_np)
        tran = tf.transformations.translation_matrix(pos_np)
        T = np.matmul(tran, rot)
        pose = g2o.Isometry3d(T)
        # print(pose.to_vector())
        if self.index < 1:
            self.graph.add_pose(time_stamp, pose, True)
        else:
            self.graph.add_pose(time_stamp, pose)
            
        # https://github.com/RainerKuemmerle/g2o/blob/master/g2o/types/slam3d/isometry3d_mappings.h
        print(time_stamp, self.graph.get_pose(time_stamp).translation()) # eigen type

        # add vertex 
        if len(self.graph.vertices()) > 1:
            ver = self.graph.vertices()[self.time_stamps[self.index]]
            ver_prev = self.graph.vertices()[self.time_stamps[self.index - 1]]
        
            # we have o->ver_prev and o->ver
            # we want ver_prev->ver:
            meas_eig = ver_prev.estimate().inverse() * ver.estimate()
            print(meas_eig.translation())
            meas = g2o.Isometry3d(meas_eig)

            # trans_np = pos_np - ver_prev.estimate().translation()
            # trans_np = ver.estimate().translation() - ver_prev.estimate().translation()
            # from ver_prev to ver
            # print("new ")
            # print(trans_np)
            # print((ver_prev.estimate() * ver.estimate().inverse()).translation())
            # print((ver.estimate().inverse() * ver_prev.estimate()).translation())
            # print("new end")
            # TODO Use odometry Twist as measurement!
            # pyquaternion.Quaternion.absolute_distance(q0, q1)
            # print(meas)
            self.graph.add_edge([ver, ver_prev], meas, covar)
            # print(ver.to_vector())
            # print(graph.get_pose(index).to_vector())
        


        # if index == 5:
        #     ver = graph.vertices()[index]
        #     ver_prev = graph.vertices()[0]
        #     meas_np = pos_np - ver_prev.estimate().translation()
        #     meas = g2o.Isometry3d(np.identity(3), meas_np)
        #     graph.add_edge([ver, ver_prev], meas)
        self.graph.save("yes.g2o")
            # graph.add_edge()
        # print(graph.vertices())
        # vertices = [graph.vertex(0), graph.vertex(index)]

        # add constraints
        # meas = g2o.EdgeSE3()
        # meas.pos0 = pt0
        # meas.pos1 = pt1
        # meas.normal0 = nm0
        # meas.normal1 = nm1
        # graph.add_edge(vertices, 2)
        self.index += 1

        # add landmark

    # def poseCB(self, data):
    #     global index, start_time
    #     # rospy.loginfo("I heard %s", data.pose.pose)
    #     # rospy.loginfo("I heard %s", data.header.stamp)
    #     pos = data.pose.pose.position
    #     quad = data.pose.pose.orientation
    #     abs_time_stamp = (data.header.stamp)
    #     if start_time is None:
    #         start_time = abs_time_stamp
    #     # Index is the time stamp multipled with 1000 (needs to be int)
    #     time_stamp = int((abs_time_stamp - start_time).to_sec() * 1000)
    #     # print(start_time)
    #     # print(time_stamp)
    #     time_stamps.append(time_stamp)

    def main_loop(self):
        pass

if __name__ == "__main__":
    # 
    # rospy.init_node('slam')
    # rospy.Subscriber("/matcher_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback)
    slam = Slam()
    rospy.init_node('slam')
    rospy.Subscriber("/rovio/odometry", nav_msgs.msg.Odometry, slam.odomCB)
    # rospy.Subscriber("/matcher_pose", geometry_msgs.msg.PoseWithCovarianceStamped, slam.poseCB)
    slam.main_loop()
    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()





