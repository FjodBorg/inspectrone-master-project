#!/usr/bin/env python3.7
import rospy
import numpy as np
import sensor_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf
import sensor_msgs.point_cloud2 as pc2
#import tf_conversions


class PCListener:
    # TODO make dem all depend on config
    def __init__(self, topic_ply):
        self.pc = None
        self.n = 0
        self.init_listener(topic_ply)

    def init_listener(self, topic_ply):
        rospy.init_node("fcgf", anonymous=True, disable_signals=True) #TODO find a better solution for keyboard events not working with rospy.sleep()
        # rospy.Subscriber("/ballast_tank_ply", PointCloud2, self.callback)
        rospy.Subscriber(topic_ply, sensor_msgs.msg.PointCloud2, self.callback)

    def callback(self, points):
        self.pc = points
        self.n = self.n + 1


class PCBroadcaster:
    def __init__(self, topic_ballast_ply):
        self.pub = rospy.Publisher(topic_ballast_ply, sensor_msgs.msg.PointCloud2, queue_size=1, latch=True)

    def publish_pcd(self, pcd):
        ros_pcd = self.open3d_to_ros(pcd)
        self.pub.publish(ros_pcd)

    # Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
    def open3d_to_ros(self, open3d_cloud, frame_id="map"):
        # Set "header"
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        points_xyz = np.asarray(open3d_cloud.points)

        return pc2.create_cloud_xyz32(header, points_xyz)

  
class PoseBroadcaster:
    def __init__(self, topic_pose):
        self.p = geometry_msgs.msg.PoseStamped()
        self.t = geometry_msgs.msg.TransformStamped()
        self.br = tf2_ros.TransformBroadcaster()
        self.pub = rospy.Publisher(topic_pose, geometry_msgs.msg.PoseStamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tf_buffer)

    def publish_transform(self, T):
        time_now = rospy.Time.now()
        self.t.header.stamp = time_now  # data.header.stamp
        self.t.header.frame_id = "scan"
        self.t.child_frame_id = "map"  # data.header.frame_id
        self.t.transform.translation.x = T[0, 3]
        self.t.transform.translation.y = T[1, 3]
        self.t.transform.translation.z = T[2, 3]
        
        # source.transform(T)
        # R = T[0:3, 0:3]
        
        q = tf.transformations.quaternion_from_matrix(T)
        #q = tf.transformations.quaternion_from_matrix(R)
        #q = tf_conversions.transformations.quaternion_from_matrix(R)
    
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        
        #while True:
        self.br.sendTransform(self.t)

        rospy.sleep(rospy.Duration(0.01)) # Wait a bit before trying for the lookup

        try:
            trans = self.tfBuffer.lookup_transform('map', 'imu_link', time_now)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("tf Exception..")
            return 0

        self.p.header.stamp = time_now
        self.p.header.frame_id = 'map'
        self.p.pose.position.x = trans.transform.translation.x
        self.p.pose.position.y = trans.transform.translation.y
        self.p.pose.position.z = trans.transform.translation.z
        # Make sure the quaternion is valid and normalized
        self.p.pose.orientation.x = trans.transform.rotation.x
        self.p.pose.orientation.y = trans.transform.rotation.y
        self.p.pose.orientation.z = trans.transform.rotation.z
        self.p.pose.orientation.w = trans.transform.rotation.w
        self.pub.publish(self.p)


class Collect:
    def __init__(self, pcd_listener=None, pcd_broadcaster=None, pose_broadcaster=None):
        self.pcd_listener = pcd_listener
        self.pose_broadcaster = pose_broadcaster
        self.pcd_broadcaster = pcd_broadcaster
