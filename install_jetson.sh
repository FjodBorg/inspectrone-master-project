#DOCKER install 
#sudo groupadd docker
#sudo usermod -aG docker ${USER}
#newgrp docker 
#docker run hello-world
#su -s ${USER}


#cd inspectron/dockerfiles/jetson #Fjodors repo
#docker build -t jetson .
#sudo docker run  --gpus all -it --privileged --rm --net=host --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix jetson 

#Native install
sudo apt update
sudo apt install -y python3.7 python3.7-dev openssl libssl-dev  ninja-build gfortran \
cmake libeigen3-dev libboost-all-dev libopenblas-dev build-essential \
libjpeg-dev zlib1g-dev git
mkdir $HOME/repos/

#source these (possibly with bashrc)
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# increase swap partition (32gb)
#sudo swapoff -a
#sudo dd if=/dev/zero of=/swapfile bs=1G count=32
#sudo chmod 600 /swapfile
#sudo mkswap /swapfile
#sudo swapon /swapfile
# add "/swapfile none swap sw 0 0" to /etc/fstab and remove the old swap partition


# Open3D dependencies
sudo apt-get update 
sudo apt-get install -y apt-utils build-essential git cmake \
 python3 python3-dev python3-pip \
 xorg-dev libglu1-mesa-dev \
libblas-dev liblapack-dev liblapacke-dev \
 libsdl2-dev libc++-7-dev libc++abi-7-dev libxi-dev \
 llvm-7 clang-7 libclang-7-dev libc++-7-dev libc++abi-7-dev 

# install newer cmake
cd $HOME/repos/
wget https://github.com/Kitware/CMake/releases/download/v3.18.6/cmake-3.18.6.tar.gz
tar -zxvf cmake-3.18.6.tar.gz
rm cmake-3.18.6.tar.gz
cd cmake-3.18.6
./bootstrap; make -j6; sudo make install

# install open3d for arm (tested with 0.12.0)(is not available on pip3 for arm)
cd $HOME/repos/
git clone --recursive https://github.com/intel-isl/Open3D
cd Open3D
git checkout tags/v0.12.0
mkdir build; cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_CUDA_MODULE=ON \
    -DBUILD_GUI=ON \
    -DBUILD_TENSORFLOW_OPS=OFF \
    -DBUILD_PYTORCH_OPS=OFF \
    -DBUILD_UNIT_TESTS=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local/ \
    -DPYTHON_IN_PATH=$(which python3.7) \
    -DPYTHON_EXECUTABLE=$(which python3.7) 
make -j$(nproc)
sudo make install
sudo rm $HOME/repos/Open3D/build/lib/python_package/pip_package -r; sudo make install-pip-package -j$(nproc)
python3.7 -m pip install $(sudo find $HOME/repos/Open3D/ -name "open3d-0.12.0*.whl")

#test if it works
python3.7 -c "import open3d; print(open3d)"


# install teaser
cd $HOME/repos
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus 
git checkout tags/v2.0
mkdir build
cd build


# python
PYTHON_VER=$(python3.7 -c "import platform; print(platform.python_version())")
PYTHON_BIN=$(which python3.7)

cd $HOME/repos/TEASER-plusplus/build
cmake  -DTEASERPP_PYTHON_VERSION=$PYTHON_VER  -DPYTHON_EXECUTABLE=$PYTHON_BIN ..
make teaserpp_python -j6
cd python && python3.7 -m pip install .

# c++
cd $HOME/repos/TEASER-plusplus/build
make -j6
sudo make install
cd .. && cd examples/teaser_cpp_ply && mkdir build && cd build
cmake .. && make

# fix executables
export LD_LIBRARY_PATH="/usr/local/lib"
sudo ldconfig

# test cpp
#cd $HOME/repos/TEASER-plusplus/examples/teaser_cpp_ply/build
#$HOME/repos/TEASER-plusplus/examples/teaser_cpp_ply/build/./teaser_cpp_ply

#test python
#cd $HOME/repos/TEASER-plusplus/examples/teaser_python_ply
#python3.7 $HOME/repos/TEASER-plusplus/examples/teaser_python_ply/teaser_python_ply.py



#install pytorch 1.7? for arm and python 3.7 (pytorch 1.7+ is broken on ubuntu 18 for arm when installing minkowski engine)
cd $HOME/repos
git clone --recursive --branch 1.7 http://github.com/pytorch/pytorch

cd pytorch
python3.7 -m pip install -U setuptools
python3.7 -m pip install Cython
python3.7 -m pip install -r requirements.txt

export USE_CUDA=1
sudo python3.7 setup.py build # took 2+ hours
sudo python3.7 setup.py install

#install minkowski
 
cd $HOME/repos/
python3.7 -m pip install numpy
git clone https://github.com/NVIDIA/MinkowskiEngine.git
cd MinkowskiEngine
git checkout tags/v0.5.0
sed -e '/pip/ s/^#*/#/' -i setup.py # remove instance of pip (not needed if you pip links to python3.7)
export CXX=g++; 
export CC=gcc;
export CUDA_HOME=/usr/local/cuda; 
export MAX_JOBS=6;
sudo python3.7 setup.py install


#install FCGF
cd $HOME/repos
git clone https://github.com/chrischoy/FCGF.git
cd FCGF
#remove open3d and minkowski from requirements.txt since it does not exist for arm and is compiled locally
sed -e '/open3d/ s/^#*/#/' -i requirements.txt 
sed -e '/MinkowskiEngine/ s/^#*/#/' -i requirements.txt 

python3.7 -m pip install pybind11
python3.7 -m pip install scipy # takes roughly 1 hour :)
python3.7 -m pip install pillow
python3.7 -m pip install sklearn
python3.7 -m pip install -r requirements.txt
#verify that it works:
#python3.7 demo.py


#install deepglobalregistration
cd $HOME/repos/
git clone https://github.com/chrischoy/DeepGlobalRegistration.git
cd DeepGlobalRegistration
# Comment out torch, open3d and MinkowskiEngine in requirements.txt
sed -e '/open3d/ s/^#*/#/' -i requirements.txt 
sed -e '/torch/ s/^#*/#/' -i requirements.txt 
sed -e '/MinkowskiEngine/ s/^#*/#/' -i requirements.txt 

python3.7 -m pip install -r requirements.txt

# INSTALL ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-melodic-desktop
echo "source /opt/ros/melodic/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update

sudo apt install -y ros-melodic-pcl-ros python-catkin-tools ros-melodic-tf2-sensor-msgs rviz

# build ros packages
cd $HOME/repos/inspectrone/catkin_ws/
catkin build -DPYTHON_EXECUTABLE=$(which python3.7) # on first build

