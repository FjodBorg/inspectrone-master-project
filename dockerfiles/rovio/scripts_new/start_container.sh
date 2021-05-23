# remember chmod +x <filename>

# get propper directories and files
DIR=$(dirname "$0")
return_info="$(./$DIR/container_name.sh)"
return_info=( $return_info )
container_name="${return_info[0]}"
DIR="${return_info[1]}"


echo "Directory of called script: $DIR"
echo "Name of container: $container_name"
echo ""
echo "The new start container runs it and then deletes it, so no need to use stop or remove container"
echo ""

xhost +local:"docker inspect --format='{{ .Config.Hostname }}' $container_name" 
docker run -it --net=host --rm --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --name=$container_name -v $HOME/repos/$container_name/src/:"/home/docker/catkin_ws/src_extern/" --gpus all $container_name bash
#docker start $container_name 
#docker exec --user docker -it $container_name bash
docker exec -it $container_name bash


