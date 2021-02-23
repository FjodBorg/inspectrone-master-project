# remember chmod +x <filename>

# get propper directories and files
DIR=$(dirname "$0")
return_info="$(./$DIR/container_name.sh)"
return_info=( $return_info )
container_name="${return_info[0]}"
DIR="${return_info[1]}"
repo_DIR=$(dirname "$DIR")


echo "Directory of called script: $DIR"
echo "Name of container: $container_name"


docker build -t $container_name $repo_DIR/. && 
	# ${DISPLAY} Returns newline, find fix
#docker build --build-arg DISPLAY=${DISPLAY} -t $container_name $repo_DIR/. && 
#docker build --build-arg HOME=${HOME} --build-arg repo_DIR="/${repo_DIR}" -t $container_name $repo_DIR/.
#docker run -it -d --net=host --name=$container_name -v $HOME/repos/$container_name/src/:"/home/docker/catkin_ws/src_extern/" $container_name bash
#docker run -it -d --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --name=$container_name -v $HOME/repos/$container_name/src/:"/home/docker/catkin_ws/src_extern/" $container_name bash
docker run -it -d --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --name=$container_name -v $HOME/repos/$container_name/src/:"/home/docker/catkin_ws/src_extern/" --gpus all $container_name bash

##-v="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged

#export LD_LIBRARY_PATH="/usr/lib/nvidia":${LD_LIBRARY_PATH}
#export PATH="/usr/lib/nvidia/xorg":${PATH}
#yay -S nvidia-container-toolkit 
