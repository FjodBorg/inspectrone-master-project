#!/usr/bin/env bash
set -e
# bash
# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source $HOME/.bashrc


# echo "${cmd_array%...}"
# echo "catkin build rovio_extras" >> $HOME/.bashrc 
# catkin build rovio_extras 
# echo "yee"

find $HOME/catkin_ws/src/rovio_extras/launch/ -maxdepth 1 -mindepth 1 -type f -exec ln -s '{}' $HOME/catkin_ws/src/rovio/launch/ \;
find $HOME/catkin_ws/src/rovio_extras/cfg/ -maxdepth 1 -mindepth 1 -type f -exec ln -s '{}' $HOME/catkin_ws/src/rovio/cfg/ \;

cmd_array=$EXTRA_OPTIONS

success=true
IFS=';' read -ra ADDR <<< "$cmd_array"
for i in "${ADDR[@]}"; do
  if [ ! -z "$i" ]; then # check if not empty string
    echo "Running command: $i"
    eval $i || echo "    please check your additional argument passed with start_container.sh" && success=false
  fi
done
 
if ! $success ; then
  exit
fi 



source $HOME/.bashrc

exec "$@"

