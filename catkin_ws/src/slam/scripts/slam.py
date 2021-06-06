#!/usr/bin/env python3.7
import numpy as np 
# import g2oclasses
import g2o
import tf 

from g2oclasses import PoseGraphOptimization

import rospy
import geometry_msgs.msg
import nav_msgs.msg
import math

# https://github.com/UditSinghParihar/g2o_tutorial
# https://github.com/uoip/g2opy/blob/master/python/examples/ba_demo.py
# https://github.com/uoip/g2opy/blob/master/g2o/what_is_in_these_directories.txt
# https://github.com/uoip/g2opy/blob/master/g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp
# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/plane_slam/simulator_3d_plane.cpp

# https://github.com/RainerKuemmerle/g2o/blob/master/doc/g2o.pdf

# BE CAREFUL HERE! Don't read and write to the same variable at the same time

class OdomInfo():
    def __init__(self) -> None:
        self.pos = None
        self.quad = None
        self.infomation = None
        self.start_time = None
        self.time_stamp = None
        self.lock = False
        self.got_new = False
    
    def get_new_values(self, data):
        self.got_new = False  # ensure no one reads it while being written
        abs_time_stamp = data.header.stamp
        # Index is the time stamp multipled with 1000 (needs to be int)
        self.time_stamp = int((abs_time_stamp - self.start_time).to_sec() * 1000) * 10 + 0  # poses end with 0
        self.pos = data.pose.pose.position
        self.quad = data.pose.pose.orientation
        covar = np.array(data.pose.covariance).reshape(6,6)
        self.infomation = np.linalg.pinv(covar)
        self.got_new = True  # ensure no one reads it while being written

    def set_start_time(self, start_time):
        self.start_time = start_time

class PoseInfo():
    def __init__(self) -> None:
        self.pos = None
        self.quad = None
        self.infomation = None
        self.start_time = None
        self.time_stamp = None
        self.lock = False
        self.got_new = False
    
    def get_new_values(self, data):
        self.got_new = False  # ensure no one reads it while being written
        abs_time_stamp = data.header.stamp
        # Index is the time stamp multipled with 1000 (needs to be int)
        self.time_stamp = int((abs_time_stamp - self.start_time).to_sec() * 1000) * 10 + 1  # poses end with 1
        self.pos = data.pose.pose.position
        self.quad = data.pose.pose.orientation
        self.covar = np.array(data.pose.covariance).reshape(6,6)
        self.infomation = np.linalg.pinv(self.covar)
        self.got_new = True  # ensure no one reads it while being written

    def set_start_time(self, start_time):
        self.start_time = start_time

class Slam():
    def __init__(self) -> None:
        self.ids = []
        self.start_time = None
        self.index = 0
        self.graph = PoseGraphOptimization()

        self.odom = OdomInfo()
        self.pose = PoseInfo()

    def set_start_time(self):
        self.odom.set_start_time(self.start_time)
        self.pose.set_start_time(self.start_time)

    def odomCB(self, data):
        # on first run set current time as 0 (easier to read indicies)
        if not self.odom.lock:
            if self.start_time is None:
                self.start_time = data.header.stamp
                self.set_start_time()

            self.odom.get_new_values(data)
            
            # self.time_stamps.append(self.odom.time_stamp)
        else:
            print("Odometry is locked, probably due to another process reading it")
        # rospy.loginfo("I heard %s", data.pose.pose)
        # rospy.loginfo("I heard %s", data.header.stamp)

        # print(start_time)

    def poseCB(self, data):
        # on first run set current time as 0 (easier to read indicies)
        if not self.pose.lock and self.start_time is not None:
            # if self.start_time is None:
            #     self.start_time = data.header.stamp
            #     self.set_start_time()

            self.pose.get_new_values(data)
            
            # self.time_stamps.append(self.odom.time_stamp)
        else:
            print("Pose is locked, probably due to another process reading it")
        # rospy.loginfo("I heard %s", data.pose.pose)
        # rospy.loginfo("I heard %s", data.header.stamp)

        # print(start_time)

    def add_edge_to_best_odom(self, pose_vertex, pose_time):
        best_dif_time = math.inf
        best_index = math.inf
        
        # Find the id for closest time between pose and odom 
        for index, odom_time in reversed(list(enumerate(self.ids))):
            dif_time = pose_time - odom_time
            if best_dif_time > abs(dif_time):
                best_index = index
                best_dif_time = dif_time
            elif dif_time < 0:  # we passed pose_time
                break

        meas_eig = np.identity(4)
        infomation = np.zeros((6,6))
        meas = g2o.Isometry3d(meas_eig)

        odom_vertex = self.graph.vertex(best_index)

        self.graph.add_edge([odom_vertex, pose_vertex], meas, infomation)

    def process_pose(self):
        time_stamp = self.pose.time_stamp

        pos = [self.pose.pos.x, self.pose.pos.y, self.pose.pos.z]
        quad = [self.pose.quad.x, self.pose.quad.y, self.pose.quad.z, self.pose.quad.w]
        infomation = self.pose.infomation

        print("New Pose", time_stamp)
        self.ids.append(time_stamp)
            
        # -90 degrees in roll 90 degs in yaw
        # rot_fix = tf.transformations.euler_matrix([-math.pi / 2.0, 0, math.pi / 2.0])
        rot = tf.transformations.quaternion_matrix(quad)
        tran = tf.transformations.translation_matrix(pos)
        T = np.matmul(tran, rot)
        # T = np.matmul(T, rot_fix)
        pose = g2o.Isometry3d(T)
        # print(pose.to_vector())
        self.graph.add_pose(time_stamp, pose, True)
            
        # https://github.com/RainerKuemmerle/g2o/blob/master/g2o/types/slam3d/isometry3d_mappings.h
        print(time_stamp, self.graph.get_pose(time_stamp).translation()) # eigen type

        # add edge
        if len(self.graph.vertices()) > 1:
            if any(self.pose.covar[0] > 0.5):
                print("covariance was too large")
                return
            ver = self.graph.vertices()[self.ids[self.index]]
            ver_prev = self.graph.vertices()[self.ids[self.index - 1]]
        
            # we have o->ver_prev and o->ver
            # we want ver_prev->ver:
            # see line 86 on:
            # https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/line_slam/simulator_3d_line.cpp 
            meas_eig = ver_prev.estimate().inverse() * ver.estimate()

            meas = g2o.Isometry3d(meas_eig)
            self.graph.add_edge([ver, ver_prev], meas, infomation)
            # add edge between teaser and rovio
            self.add_edge_to_best_odom(ver, time_stamp)

    def process_odom(self):
        time_stamp = self.odom.time_stamp

        pos = [self.odom.pos.x, self.odom.pos.y, self.odom.pos.z]
        quad = [self.odom.quad.x, self.odom.quad.y, self.odom.quad.z, self.odom.quad.w]
        infomation = self.odom.infomation

        print("New Odom", time_stamp)
        self.ids.append(time_stamp) 
            
        rot = tf.transformations.quaternion_matrix(quad)
        tran = tf.transformations.translation_matrix(pos)
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
            ver = self.graph.vertices()[self.ids[self.index]]
            ver_prev = self.graph.vertices()[self.ids[self.index - 1]]
        
            # we have o->ver_prev and o->ver
            # we want ver_prev->ver:
            meas_eig = ver_prev.estimate().inverse() * ver.estimate()
            # print(meas_eig.translation())
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
            self.graph.add_edge([ver, ver_prev], meas, infomation)
            # print(ver.to_vector())
            # print(graph.get_pose(index).to_vector())
        


        # if index == 5:
        #     ver = graph.vertices()[index]
        #     ver_prev = graph.vertices()[0]
        #     meas_np = pos_np - ver_prev.estimate().translation()
        #     meas = g2o.Isometry3d(np.identity(3), meas_np)
        #     graph.add_edge([ver, ver_prev], meas)

    def main_loop(self):


        
        if self.odom.got_new:
            self.odom.lock = True
            self.odom.got_new = False
            self.process_odom()
            # print("new2")
            
            self.index += 1
            self.odom.lock = False
        
        # print(self.odom.time_stamp)
        if self.pose.got_new and self.pose.time_stamp >= 0:
            self.pose.lock = True
            self.pose.got_new = False
            self.process_pose()
            # print("new2")
            
            self.index += 1
            self.pose.lock = False
        
        

        # pose_np = np.array([pos.x, pos.y, pos.z, quad.x, quad.y, quad.z, quad.w])
        # pose = g2o.SE3Quat(pose_np)
        # pose_np = np.array([pos.x, pos.y, pos.z, quad.x, quad.y, quad.z, quad.w])
        # pose = g2o.SE3Quat(pose_np)

        
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
        # self.index += 1

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


if __name__ == "__main__":
    # 
    # rospy.init_node('slam')
    # rospy.Subscriber("/matcher_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback)
    slam = Slam()
    rospy.init_node('slam')
    rospy.Subscriber("/rovio/odometry", nav_msgs.msg.Odometry, slam.odomCB)
    rospy.Subscriber("/matcher_pose", geometry_msgs.msg.PoseWithCovarianceStamped, slam.poseCB)
    while(True):
        slam.main_loop()

        try:
            rospy.sleep(0.1)
        except rospy.ROSTimeMovedBackwardsException:
            # Sometimes sleep throws this exception when running it on a bag file...
            pass

        except rospy.ROSInterruptException:
            # if e.g ctrl + c
            break

    rospy.spin()





