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

xhost -local:"docker inspect --format='{{ .Config.Hostname }}' $container_name" 
docker stop $container_name
