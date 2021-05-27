#!/usr/bin/env bash
inspectrone_folder="$HOME/repos/inspectrone"
echo $inspectrone_folder
rovio="$inspectrone_folder/dockerfiles/rovio/"

echo $rovio

echo "bag file can be launched with: "
echo "rosbag play bags/Kinect/groundtruth_imu_frame.bag /points2:=/points_in --clock"

# run in subscript so we don't change the directory
(
    cd $rovio 
    ./scripts_new/start_container.sh "roslaunch rovio rovio_node_kinect.launch"
)
# echo "yes"
