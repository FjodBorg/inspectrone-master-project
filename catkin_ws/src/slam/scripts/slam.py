#!/usr/bin/env python3.7
import numpy as np 
# import g2oclasses
import g2o
from numpy.core.arrayprint import IntegerFormat
import tf 

from g2oclasses import PoseGraphOptimization

import rospy
import visualization_msgs.msg 
import geometry_msgs.msg
import nav_msgs.msg
import math
import tf2_ros

# https://github.com/UditSinghParihar/g2o_tutorial
# https://github.com/uoip/g2opy/blob/master/python/examples/ba_demo.py
# https://github.com/uoip/g2opy/blob/master/g2o/what_is_in_these_directories.txt
# https://github.com/uoip/g2opy/blob/master/g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp
# https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/plane_slam/simulator_3d_plane.cpp

# https://github.com/RainerKuemmerle/g2o/blob/master/doc/g2o.pdf

# BE CAREFUL HERE! Don't read and write to the same variable at the same time

class Markers():
    def __init__(self) -> None:
        self.marker_idx = 0
        self.marker_array_msg = visualization_msgs.msg.MarkerArray()
        self.marker_array_pub = rospy.Publisher('/graph', visualization_msgs.msg.MarkerArray, queue_size=10)

    def add_marker(self, pose):
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = pose.time_stamp
        marker.type = marker.SPHERE
        # marker.frame_locked = False
        # print(pose, type(pose), isinstance(pose, OdomInfo))
        if isinstance(pose, OdomInfo):
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1
            marker.color.a = 1.0
        elif isinstance(pose, PoseInfo):
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1
            marker.color.a = 1.0
        # marker.action = 2
        marker.pose = geometry_msgs.msg.Pose()
        marker.pose.position = pose.position
        marker.pose.orientation = pose.orientation
        # marker.lifetime.secs = 1000
        # print(marker)
        # print(pose.position)
        # print(pose.orientation)
        # self.marker.ns = "Goal-%u"%i
        self.marker_array_msg.markers.append(marker)
        self.marker_idx += 1
        self.marker_array_pub.publish(self.marker_array_msg)

    def add_marker_edge(self, graph, idx_from, idx_to, rgb=[1,0,0]):
        print(idx_from, idx_to)
        pos_from = graph.get_pose(idx_from).translation()
        pos_to = graph.get_pose(idx_to).translation()
        print("yee", pos_from, pos_to)

        marker2 = visualization_msgs.msg.Marker() 
        marker2.header.frame_id = "/map" 
        marker2.header.stamp = rospy.Time.now()
        marker2.id = -len(self.marker_array_msg.markers)
        marker2.type = marker2.LINE_STRIP

        marker2.action = marker2.ADD
        marker2.scale.x = 0.005
        marker2.scale.y = 0.005
        marker2.scale.z = 0.005
        marker2.color.r = rgb[0]
        marker2.color.g = rgb[1]
        marker2.color.b = rgb[2]
        marker2.color.a = 1.0

        # marker2.pose.position.x = pos_from[0]
        # marker2.pose.position.y = pos_from[1]
        # marker2.pose.position.z = pos_from[2]
        marker2.pose.orientation.w = 1.0
        p1 = geometry_msgs.msg.Point()
        p1.x, p1.y, p1.z = pos_from
        p2 = geometry_msgs.msg.Point()
        p2.x, p2.y, p2.z = pos_to
        
        # if rgb[0] == 1 and rgb[1] == 0:
        #     print("breaking news")
        #     print(p1, p2)
        #     print(idx_from, idx_to)
        
        marker2.points.append(p1)
        marker2.points.append(p2)

        self.marker_array_msg.markers.append(marker2)  # add linestrip to markerArray
        self.marker_array_pub.publish(self.marker_array_msg)
        
        # marker = visualization_msgs.msg.Marker()
        # marker.header.frame_id = "map"
        # marker.header.stamp = rospy.Time.now()
        # marker.id = pose.time_stamp
        # # marker.frame_locked = False
        # # print(pose, type(pose), isinstance(pose, OdomInfo))
        # marker.scale.x = 0.1
        # marker.scale.y = 0.1
        # marker.scale.z = 0.1
        # marker.color.r = 1.0
        # marker.color.g = 0.0
        # marker.color.b = 1
        # marker.color.a = 1.0
        # marker.type = 2
        # # marker.action = 2
        # marker.pose = geometry_msgs.msg.Pose()
        # marker.pose.position = pose.position
        # marker.pose.orientation = pose.orientation
        # marker.lifetime.secs = 1000
        # # print(marker)
        # # print(pose.position)
        # # print(pose.orientation)
        # # self.marker.ns = "Goal-%u"%i
        # self.marker_array_msg.markers.append(marker)
        # self.marker_idx += 1
        # self.marker_array_pub.publish(self.marker_array_msg)
        

class OdomInfo():
    def __init__(self) -> None:
        self.position = None
        self.orientation = None
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
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation
        covar = np.array(data.pose.covariance).reshape(6,6)
        self.infomation = np.linalg.pinv(covar)
        self.got_new = True  # ensure no one reads it while being written

    def set_start_time(self, start_time):
        self.start_time = start_time

class PoseInfo():
    def __init__(self) -> None:
        self.position = None
        self.orientation = None
        self.infomation = None
        self.prev_infomation = None
        self.first_fixed = False
        self.start_time = None
        self.time_stamp = None
        self.lock = False
        self.got_new = False
    
    def get_new_values(self, data):
        self.got_new = False  # ensure no one reads it while being written
        abs_time_stamp = data.header.stamp
        # Index is the time stamp multipled with 1000 (needs to be int)
        self.time_stamp = int((abs_time_stamp - self.start_time).to_sec() * 1000) * 10 + 1  # poses end with 1
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation
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
        self.markerArray = Markers()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.world_offset = None

    def set_start_time(self):
        self.odom.set_start_time(self.start_time)
        self.pose.set_start_time(self.start_time)

    def odomCB(self, data):
        # on first run set current time as 0 (easier to read indicies)
        if self.world_offset is None:
            try:
                trans = self.tf_buffer.lookup_transform('map', "world_offset", rospy.Time.now())
                quad = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
                pos = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
                rot = tf.transformations.quaternion_matrix(quad)
                tran = tf.transformations.translation_matrix(pos)
                self.world_offset = np.matmul(tran, rot)

            except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.LookupException):
                print("tf Exception..")
                return 0

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

    def add_edge_to_best_odom(self, pose_vertex, pose_time, infomation_now):
        best_dif_time = math.inf
        best_id = math.inf
        
        # Find the id for closest time between pose and odom 
        for odom_time in reversed(self.ids):
            if odom_time % 2 == 1:
                # print("skipping: ", pose_time, odom_time, best_dif_time, best_id)
                # it is another pose_type and not odom
                continue
            # print("candidates: ", pose_time, odom_time, best_dif_time, best_id)

            dif_time = odom_time - pose_time
            if best_dif_time > abs(dif_time):
                best_id = odom_time
                best_dif_time = dif_time
            elif dif_time < 0:  # we passed pose_time
                break

        meas_eig = np.identity(4)
        infomation = infomation_now
        # TODO, this might break the optimization e.g. if pose pose is wrong and it has 0 info to odom pose, what would happen? Perhaps use info_in  instead?
        # infomation = np.zeros((6, 6))
        meas = g2o.Isometry3d(meas_eig)

        odom_vertex = self.graph.vertex(best_id)

        self.graph.add_edge([odom_vertex, pose_vertex], meas, infomation)
        self.markerArray.add_marker_edge(self.graph, best_id, self.ids[self.index], [1,1,0])

    def process_pose(self):
        time_stamp = self.pose.time_stamp

        pos = [self.pose.position.x, self.pose.position.y, self.pose.position.z]
        quad = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]


        print("\nNew Pose", time_stamp)
        if any(self.pose.covar[0] > 0.1):
            print("covariance was too large, no edge added")
            return -1

        
        self.ids.append(time_stamp)


        # -90 degrees in roll 90 degs in yaw
        # rot_fix = tf.transformations.euler_matrix([-math.pi / 2.0, 0, math.pi / 2.0])
        rot = tf.transformations.quaternion_matrix(quad)
        tran = tf.transformations.translation_matrix(pos)
        T = np.matmul(tran, rot)
        # T = np.matmul(T, rot_fix)
        pose = g2o.Isometry3d(T)
        # print(pose.to_vector())
        # TODO try with 1 fixed and with all fixed (infomation is now fixed)
        if not self.pose.first_fixed:
            self.graph.add_pose(time_stamp, pose, True)
        else:
            self.graph.add_pose(time_stamp, pose)
        self.markerArray.add_marker(self.pose)
            
        # https://github.com/RainerKuemmerle/g2o/blob/master/g2o/types/slam3d/isometry3d_mappings.h
        print(time_stamp, self.graph.get_pose(time_stamp).translation()) # eigen type

        # add edge
        if len(self.graph.vertices()) > 1:

            ver = self.graph.vertices()[self.ids[self.index]]
            
            i = 1
            while True:
                id = self.ids[self.index - i]
                # make sure it is another odom measurement
                if id % 2 == 1:
                    ver_prev = self.graph.vertices()[self.ids[self.index - i]]
                    break
                i += 1

            # we have o->ver_prev and o->ver
            # we want ver_prev->ver:
            # see line 86 on:
            # https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/line_slam/simulator_3d_line.cpp 
            meas_eig = ver_prev.estimate().inverse() * ver.estimate()

            meas = g2o.Isometry3d(meas_eig)
            # Use the largest covariance from the current and last pose
                
            if self.pose.prev_infomation is not None and np.sum(self.pose.prev_infomation) < np.sum(self.pose.infomation):
                # covariance: prev > now    
                # infomation: prev < now
                # if last last pose was the worst:
                infomation = self.pose.prev_infomation
            else:
                # if the current pose is the worst
                infomation = self.pose.infomation

            self.graph.add_edge([ver_prev, ver], meas, infomation)
            self.markerArray.add_marker_edge(self.graph, self.ids[self.index - i], self.ids[self.index], [1,0,0])

            # print(self.graph.edges())
            # print(dir(self.graph.edges()))
            # print(type(self.graph.edges()))
            # # print(dir(self.graph))
            # print(dir(list(self.graph.edges())[0]))
            # print(list(self.graph.edges())[0].id())
            # print(list(self.graph.edges())[0].vertices())
            # # print(dir(list(self.graph.edges())))
            # print(list(self.graph.edges())[0].information())
            # print(dir((self.graph)))
            # print(dir(list(self.graph.edges())))
            # print(locals(self.graph.edges()))
            # print(globals(self.graph.edges()))

            # add edge between teaser and rovio
            # use self.pose.infomation, since we're link from the current pose to the matching odom and not the previous pose 
            self.add_edge_to_best_odom(ver, time_stamp, self.pose.infomation)

        self.pose.prev_infomation = self.pose.infomation

        return 0

    def process_odom(self):
        time_stamp = self.odom.time_stamp

        pos = [self.odom.position.x, self.odom.position.y, self.odom.position.z]
        quad = [self.odom.orientation.x, self.odom.orientation.y, self.odom.orientation.z, self.odom.orientation.w]
        infomation = self.odom.infomation

        # TODO maybe add multiplication to odom infomation
        infomation *= 100000.0

        print("New Odom", time_stamp)
        self.ids.append(time_stamp) 
            
        rot = tf.transformations.quaternion_matrix(quad)
        tran = tf.transformations.translation_matrix(pos)
        T = np.matmul(tran, rot)
        # print(T)
        # Fix so the correct measurement is put into g2o
        # T = np.matmul(self.world_offset, T)
        # print(T)
        pose = g2o.Isometry3d(T)
        # print(pose.to_vector())
        self.graph.add_pose(time_stamp, pose)
        self.markerArray.add_marker(self.odom)
            
        # https://github.com/RainerKuemmerle/g2o/blob/master/g2o/types/slam3d/isometry3d_mappings.h
        print(time_stamp, self.graph.get_pose(time_stamp).translation()) # eigen type

        # add vertex 
        if len(self.graph.vertices()) > 1:
            # print("\n", self.index)
            # print(self.graph.vertices())
            ver = self.graph.vertices()[self.ids[self.index]]

            i = 1 
            while True :
                id = self.ids[self.index - i]
                # make sure it is another odom measurement
                if id % 2 == 0:
                    ver_prev = self.graph.vertices()[self.ids[self.index - i]]
                    break
                i += 1
        
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
            self.graph.add_edge([ver_prev, ver], meas, infomation)
            self.markerArray.add_marker_edge(self.graph, self.ids[self.index - i], self.ids[self.index], [0,1,0])
            # print(ver.to_vector())
            # print(graph.get_pose(index).to_vector())
        


        # if index == 5:
        #     ver = graph.vertices()[index]
        #     ver_prev = graph.vertices()[0]
        #     meas_np = pos_np - ver_prev.estimate().translation()
        #     meas = g2o.Isometry3d(np.identity(3), meas_np)
        #     graph.add_edge([ver, ver_prev], meas)
        return 0

    def main_loop(self):


        
        if self.odom.got_new:
            self.odom.lock = True
            self.odom.got_new = False
            if self.process_odom() == 0:
                self.index += 1
            # print("new2")
            
            self.odom.lock = False
        
        # print(self.odom.time_stamp)
        if self.pose.got_new and self.pose.time_stamp >= 0:
            self.pose.lock = True
            self.pose.got_new = False
            if self.process_pose() == 0:
                self.index += 1
            # print("new2")
            
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
    rospy.init_node('slam')
    # 
    # rospy.init_node('slam')
    # rospy.Subscriber("/matcher_pose", geometry_msgs.msg.PoseWithCovarianceStamped, callback)
    slam = Slam()
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
            slam.graph.optimize(20)
            slam.graph.save("yes_opt.g2o")
            break

    rospy.spin()





