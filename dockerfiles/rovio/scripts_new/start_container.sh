# remember chmod +x <filename>

# get propper directories and files
DIR=$(dirname "$0")
return_info="$(./$DIR/container_name.sh)"
return_info=( $return_info )
container_name="${return_info[0]}"
DIR="${return_info[1]}"


echo "Directory of called script: $DIR"
echo "Name of container: $container_name, should be something like rovio, if not try and call it from the rovio folder"
echo ""
echo "The new start container runs it and then deletes it, so no need to use stop or remove container"
echo ""

# process arguments
input_array=("$@") 
printf -v joined_array '%s;' "${input_array[@]}"

xhost +local:"docker inspect --format='{{ .Config.Hostname }}' $container_name" 
docker run -it --net=host --rm --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="EXTRA_OPTIONS=$joined_array"  --name=$container_name -v $DIR/../src_extra/rovio_extras:"/home/docker/catkin_ws/src/rovio_extras" --gpus all $container_name bash #  && bash #/ros_entrypoint.sh
#docker start $container_name 
#docker exec --user docker -it $container_name bash
# docker exec -it $container_name bash


