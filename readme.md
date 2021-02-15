docker build -t inspectrone .  #build docker

#chmod +x ros_entrypoint.sh?

#docker run -it inspectrone     #run docker
#docker run -p 80:90 inspectron #run docker but with port 80 from host to port 90 in container

# -it runs it as a terminal, -d is for background, name is container name, --network host is for communication

# docker run -it -d --name test --mount type=bind,source=$HOME/repos/inspectrone/packages/,target="/app/" inspectrone #mount host folder to target  (host:container)

docker run -it -d --network host --name test -v $HOME/repos/inspectrone/packages/:"/app/" inspectrone #mount host folder to target(host:container) /app/ is over written with contents from packages


docker container stop test #stop container
docker container start test #start container
docker container rm test #remove container 


docker ps -l # print current docker sessions
docker exec -it test bash #open image with name test and in bash

docker commit [container id] [new image name] #Saves the current container as a new image


### Communication
#run roscore on host

### Add this to container
#export ROS_MASTER=http://$(hostname --ip-address):11311
#export ROS_HOSTNAME=$(hostname --ip-address)

### Display visuals
# --env="DISPLAY=$(hostname --ip-address)$DISPLAY"
# xhost local:root # maybe not safe


### Currently used:
docker run -it -d --net=host --name test7 --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -v $HOME/repos/inspectrone/packages/:"/app/" inspectrone
xhost +local:"docker inspect --format='{{ .Config.Hostname }}' test7" 
docker start test7
docker exec -it test7 bash

ln -s /app/extra rovio/.

### decompressd!
<node name="republish" type="republish" pkg="image_transport" output="screen" args="compressed in:=/axis_camera raw out:=/axis_camera/image_raw" />

sudo apt install ros-melodic-image-transport-plugins ros-melodic-image-pipeline

<node name="republish_left" type="republish" pkg="image_transport" output="screen" args="compressed in:=/mynteye/left/image_color/ raw out:=/cam0/image_raw" />
<node name="republish_right" type="republish" pkg="image_transport" output="screen" args="compressed in:=/mynteye/right/image_color/ raw out:=/cam1/image_raw" />

#perhaps add it as a seperate package?
# use localization_paper.bag (mynteye)
# rosbags needs --clock and rosparam set use_sim_time true 

### Debugging
# rosrun rqt_tf_tree rqt_tf_tree 
