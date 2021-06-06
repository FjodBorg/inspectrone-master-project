#!/usr/bin/env python3.7
import rospy
import numpy as np
import sensor_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf
import sensor_msgs.point_cloud2 as pc2
import math
#import tf_conversions


class PCListener:
    def __init__(self, topic_in_ply):
        self.pc = None
        self.stamp = 0
        self.n = 0
        self.init_listener(topic_in_ply)

    def init_listener(self, topic_in_ply):
        # rospy.Subscriber("/ballast_tank_ply", PointCloud2, self.callback)
        rospy.Subscriber(topic_in_ply, sensor_msgs.msg.PointCloud2, self.callback)

    def callback(self, points):
        # TODO, make sure that these variables doesn't get read and written at the same time. 
        self.pc = points
        self.stamp = points.header
        self.n = self.n + 1


class PCBroadcaster:
    def __init__(self, topic_ballast_ply, topic_scan_ply):
        self.pub_map = rospy.Publisher(topic_ballast_ply, sensor_msgs.msg.PointCloud2, queue_size=1, latch=True)
        self.pub_scan = rospy.Publisher(topic_scan_ply, sensor_msgs.msg.PointCloud2, queue_size=1, latch=True)
        self.stamp = 0

    def publish_pcds(self, pcd_scan, pcd_map):
        self.stamp = rospy.Time.now()
        ros_pcd_scan = self.open3d_to_ros(pcd_scan, frame_id="scan")
        ros_pcd_map = self.open3d_to_ros(pcd_map, frame_id="map")
        ros_pcd_map.header.stamp = self.stamp
        ros_pcd_scan.header.stamp = self.stamp
        # print(ros_pcd_map.header)
        # print(ros_pcd_scan.header)
        self.pub_scan.publish(ros_pcd_scan)
        self.pub_map.publish(ros_pcd_map)
        return self.stamp

    def publish_inital_map(self, pcd_map):
        self.stamp = rospy.Time.now()
        ros_pcd_map = self.open3d_to_ros(pcd_map, frame_id="map")
        self.pub_map.publish(ros_pcd_map)

    # Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
    def open3d_to_ros(self, open3d_cloud, frame_id="map"):
        # Set "header"
        header = std_msgs.msg.Header()
        header.stamp = self.stamp
        header.frame_id = frame_id

        points_xyz = np.asarray(open3d_cloud.points)

        return pc2.create_cloud_xyz32(header, points_xyz)

class PoseBroadcaster:
    def __init__(self, topic_pose, frame_id="scan"):
        self.p = geometry_msgs.msg.PoseWithCovarianceStamped()
        self.t = geometry_msgs.msg.TransformStamped()
        self.br = tf2_ros.TransformBroadcaster()
        self.pub = rospy.Publisher(topic_pose, geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.frame_id = frame_id
        self._covariance = self.init_covariance()
        

    def publish_transform(self, T, stamp=None, pcd_read_time=None):   
        if stamp is None:
            stamp = rospy.Time.now()
        if pcd_read_time is None:
            pcd_read_time = rospy.Time.now()
        # print(stamp)
        self.t.header.stamp = stamp  # data.header.stamp
        self.t.header.frame_id = "map"
        self.t.child_frame_id = self.frame_id  # data.header.frame_id
        self.t.transform.translation.x = T[0, 3]
        self.t.transform.translation.y = T[1, 3]
        self.t.transform.translation.z = T[2, 3]
        q = tf.transformations.quaternion_from_matrix(T)
        # tran = tf.transformations.translation_from_matruix

        self.t.transform.rotation = geometry_msgs.msg.Quaternion(*q)
        # self.t.transform.rotation.y = q[1]
        # self.t.transform.rotation.z = q[2]
        # self.t.transform.rotation.w = q[3]
        
        self.br.sendTransform(self.t)

        # TODO find another approach to get the correct transform
        # maybe lookup the unknown parts and calculate it yourself?
        rospy.sleep(rospy.Duration(0.1))  # Wait a bit before trying for the lookup
        while(True):
            try:
                rospy.sleep(0.1)
                break
            except rospy.ROSTimeMovedBackwardsException:
                pass
            except rospy.ROSInterruptException:
                # if e.g ctrl + c
                break
        # print(self.t.header)
        #self.tf_buffer.waitForTransform("map", self.t.child_frame_id, rospy.Time(), rospy.Duration(4.0))
        try:
            trans = self.tf_buffer.lookup_transform('map', self.t.child_frame_id, stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("tf Exception..")
            return 0

        self.p.header.stamp = pcd_read_time
        self.p.header.frame_id = 'map'
        self.p.pose.pose.position = trans.transform.translation
        # self.p.pose.pose.position.y = trans.transform.translation.y
        # self.p.pose.pose.position.z = trans.transform.translation.z
        # Make sure the quaternion is valid and normalized
        
        q_rot = tf.transformations.quaternion_from_euler(0, math.pi / 2.0, math.pi / 2.0)
        q_now = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        q_fix = tf.transformations.quaternion_multiply(q_now, q_rot)
        print("\n\n\n",q_rot, q_now, q_fix )

        self.p.pose.pose.orientation = geometry_msgs.msg.Quaternion(*q_fix)
        # self.p.pose.pose.orientation.y = trans.transform.rotation.y
        # self.p.pose.pose.orientation.z = trans.transform.rotation.z
        # self.p.pose.pose.orientation.w = trans.transform.rotation.w

        # rot_fix = tf.transformations.euler_matrix(0, math.pi / 2.0, math.pi / 2.0)
        # T = np.matmul(T, rot_fix)
        # print(self.p.pose.covariance)
        self.p.pose.covariance = self._covariance
        # print(self._covariance)
        self.pub.publish(self.p)

    def init_covariance(self):
        return [0.0] * 36

    def set_covariance(self, covariance):
        self._covariance = covariance

class Collect:
    def __init__(self, pcd_listener=None, pcd_broadcaster=None, pose_broadcaster=None):
        self.pcd_listener = pcd_listener
        self.pose_broadcaster = pose_broadcaster
        self.pcd_broadcaster = pcd_broadcaster

class Inits:
    def __init__(self):
        rospy.init_node("fcgf", anonymous=True, disable_signals=True) #TODO find a better solution for keyboard events not working with rospy.sleep()
        if not rospy.has_param('/fcgf/fitness_thr'):
            rospy.set_param('/fcgf/fitness_thr', 0.85)
        