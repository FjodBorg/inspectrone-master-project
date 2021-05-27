#!/usr/bin/env bash
inspectrone_folder="$HOME/repos/inspectrone"
echo $inspectrone_folder
rovio="$inspectrone_folder/dockerfiles/rovio/"

echo $rovio

echo "bag file can be launched with: "
echo "rosbag play bags/Mynteye/Localization_paper.bag --clock --loop /mynteye/points/downsampled_xyz:=/points_in -r 1"

# run in subscript so we don't change the directory
(
    cd $rovio 
    ./scripts_new/start_container.sh "roslaunch rovio rovio_node_mynteye.launch"
)
# echo "yes"