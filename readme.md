#TODO , remember to add the short links to docker file
# TODO get xserver working with nvidia http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2
# remove --gpu-all from run if you don't have nvidia instaleld also from docker
# #nvidia-container-toolkit
# nvidia-docker 
# watch -n 0.1 nvidia-smi

docker build -t inspectrone .  #build docker
yay -S nvidia-container-toolkit python-catkin_tools orocos-kdl-python 
pacman -S ogre 
modify pkgbuild for ros-melodic-rviz


#getting arm64 docker to work on amd64 machine: #IMPORTANT
apt-get install qemu binfmt-support qemu-user-static
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
# docker run -v $(pwd)/qemu-aarch64-static:/usr/bin/qemu-aarch64-static -it arm64v8/debian:stretch-slim bash


#chmod +x ros_entrypoint.sh?

#docker run -it inspectrone     #run docker
#docker run -p 80:90 inspectron #run docker but with port 80 from host to port 90 in container

# -it runs it as a terminal, -d is for background, name is container name, --network host is for communication

# docker run -it -d --name test --mount type=bind,source=$HOME/repos/inspectrone/packages/,target="/app/" inspectrone #mount host folder to target  (host:container)
# rosrun rqt_tf_tree rqt_tf_tree 
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

## TEASER++
yay -S python-scikit-learn python-open3d
https://teaser.readthedocs.io/en/latest/installation.html
# Remember to run OMP_NUM_THREADS=12 otherwise segmentation fault
#install dependecies for pcl (repos/f_arch_setup) -> makepkg -Acs in the folder with PKGBUILD (CUDA enabled) -> sudo pacman -U x.pkg.tar.xz
# install all things in my f_arch_repos
yay -S python-typing_extensions

###building 
cd $HOME/repos/
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. && make
sudo make install

### C++ example
cd $HOME/repos/
cd .. && cd examples/teaser_cpp_ply && mkdir build && cd build
cmake .. && make
./teaser_cpp_ply

### PYTHON example
cd TEASER-plusplus && mkdir build && cd build
cmake -DTEASERPP_PYTHON_VERSION=3.9.1 .. && make teaserpp_python
cd python && pip install .
cd ../.. && cd examples/teaser_python_ply 
python teaser_python_ply.py

### example
cd TEASER-plusplus && mkdir build && cd build
cmake -DTEASERPP_PYTHON_VERSION=3.9.1 .. && make teaserpp_python
cd python && pip install .
cd ../.. && cd examples/teaser_python_3dsmooth 
python teaser_python_3dsmooth.py


### FCGF 

# DO NOT INSTALL CUDA 11.0, it seems to be broken for many people and cuda 11.1++ is not avaialbe on conda
# Cuda 11.2 doesn't seem to work either (Not enough memory error)
# install cuda 10.2 and use that for compiling minkowskiengine
# USE THIS COMBINATION, THE OTHERS have drastically more memory usage or doesn't work
source /opt/anaconda/bin/activate root 
conda create -n py3-fcgf python=3.7
conda activate py3-fcgf
conda install pytorch=1.5.1 cudatoolkit=10.2 torchvision=0.6.1 -c pytorch
#conda install -c conda-forge pytorch 
conda install openblas-devel -c anaconda
# install minkowsky manually
export CXX=g++; 
export CC=gcc;
export CUDA_HOME=/opt/cuda-10.2; 
pip install -U MinkowskiEngine=0.5 --install-option="--blas=openblas" -v --no-deps
#pip install git+https://github.com/NVIDIA/MinkowskiEngine.git


# install FCGF
git clone https://github.com/chrischoy/FCGF.git
cd FCGF
pip install -r requirements.txt
# fix benchmark_3dmatch.py, change "_gt.log" to "-evaluation/gt.log"
# --batch-size 1, everything above is too much for 4GB ram




### Smoothnet 
# instlla from source prefered
#source /opt/anaconda/bin/activate root 
#conda create -n py3-smooth python=3.7
#conda activate py3-smooth
#conda install cudatoolkit=10.0 cudnn=7
#export CXX=g++; 
#export CC=gcc;
#export CUDA_HOME=/opt/cuda-10.2; 
#git clone https://github.com/tensorflow/tensorflow.git
#cd tensorflow
#./configure #remember jdk java and bazen 0.4
#bazel build --config=v1 --config=cuda //tensorflow/tools/pip_package:build_pip_packagebazel build --config=v1 --#config=cuda //tensorflow/tools/pip_package:build_pip_package
#READ THIS https://github.com/tensorflow/tensorflow/issues/45861
# https://github.com/grpc/grpc/pull/20048/commits/de6255941a5e1c2fb2d50e57f84e38c09f45023d

#dont install glob3 from requirements list, and change tensforflow version to 1.13.1 
# find tensorflow from github, choose 1.13 - 1.15.5
source /opt/anaconda/bin/activate root 
conda create -n py3-smooth python=3.7
conda activate py3-smooth
conda install cudatoolkit=10.0 cudnn=7
#export CXX=g++; 
#export CC=gcc;
#export CUDA_HOME=/opt/cuda-10.2; 
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/cuda-10.2/lib64
git clone https://github.com/zgojcic/3DSmoothNet
cd 3DSmoothNet
gedit CMakeLists.txt
pip install -r requirements.txt 

# add #include <pcl/impl/point_types.hpp> to core/core.h and change pcl version in cmakelists 
cmake -DCMAKE_BUILD_TYPE=Release .
make



###INSTALL teaser for conda enviroment of FCGF

### Add the libraries to path
export PYTHONPATH=/opt/ros/melodic/lib/python3.9/site-packages
export PYTHONPATH=$PYTHONPATH:$HOME/repos/FCGF
export PYTHONPATH=$PYTHONPATH:$HOME/.conda/envs/py3-fcgf/lib/python3.7/site-packages/
export PYTHONPATH=$PYTHONPATH:$HOME/repos/DeepGlobalRegistration
export PYTHONPATH=$PYTHONPATH:$HOME/repos/TEASER-plusplus/examples/teaser_python_fpfh_icp

#export PYTHONPATH=$PYTHONPATH:$HOME/repos/TEASER-plusplus/python/

### still in conda enviroment 
git clone https://github.com/chrischoy/DeepGlobalRegistration.git
cd DeepGlobalRegistration
python -m pip install -r requirements.txt

### setup paths  for external python package for conda
conda develop $HOME/repos/FCGF/
conda develop $HOME/repos/DeepGlobalRegistration/
cd ~/repos/
ln -s inspectrone/src/packages/extra/smooth_net/scripts/helpers.py inspectrone/demo_files/



add these two lines to the cmakefile of ply publisher just below the definition of the project
```
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

### other dependencies
git clone https://github.com/eric-wieser/ros_numpy.git
cd ros_numpy
python setup.py install






source /opt/anaconda/bin/activate root 
conda activate py3-fcgf

export ROS_PYTHON_VERSION=3
catkin build -DPYTHON_EXECUTABLE=$(which python) # on first build

source ~/repos/inspectrone/catkin_ws/devel/setup.bash 
source /opt/anaconda/bin/activate root 
conda activate py3-fcgf

export ROS_PYTHON_VERSION=3

