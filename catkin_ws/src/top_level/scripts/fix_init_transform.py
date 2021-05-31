#!/usr/bin/python3.7
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':

    rospy.init_node('abs_correction_tf2_broadcaster')
    listener = tf.TransformListener()

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "world_offset"

    trans = rot = None

    while not rospy.is_shutdown():
        rospy.loginfo("waiting for transform...")
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/world", "/rig", now, rospy.Duration(1.0))
            # listener.waitForTransform("/turtle2", "/carrot1", now, rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform("/world", "/rig", now)
            rospy.loginfo("got transform")
            break
        except:
            pass

    #print(trans, rot)
    rospy.loginfo("got transform")
    static_transformStamped.transform.translation = geometry_msgs.msg.Vector3(*trans)
    # static_transformStamped.transform.rotation = geometry_msgs.msg.Quaternion(*rot)
    static_transformStamped.transform.rotation = geometry_msgs.msg.Quaternion(0,0,0,1)
    static_transformStamped.header.stamp = now
    rospy.loginfo(static_transformStamped)
    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()