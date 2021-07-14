# inspectrone
Readme not done yet

In short 
Catkin workspace contains everything with the actual algorithm and data needed for it
scripts containts small scripts for visualizaing, test, plotting etc.
Dockerfiles has the dockerfile for ROVIO and scripts to install them

install_cuda_ubuntuda_18.sh checks for cuda and installs the versionen need if not found
install_jetson.sh installs the rest of the needed programs, is made to work on jetson, but can also be used for ubuntu, but without some smart cpu function (arm is simpler)
