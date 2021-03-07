sudo apt install -y python3.7 python3.7-dev openssl libssl-dev  ninja-build gfortran
sudo apt install -y cmake libeigen3-dev libboost-all-dev libopenblas-dev build-essential
sudo apt-get install -y libjpeg-dev zlib1g-dev

#source these (possibly with bashrc)
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# increase swap partition (32gb)
sudo swapoff -a
sudo dd if=/dev/zero of=/swapfile bs=1G count=32
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
# add "/swapfile none swap sw 0 0" to /etc/fstab and remove the old swap partition

# setup local ssh


# Open3D dependencies
sudo apt-get update -y
sudo apt-get install -y apt-utils build-essential git cmake
sudo apt-get install -y python3 python3-dev python3-pip
sudo apt-get install -y xorg-dev libglu1-mesa-dev
sudo apt-get install -y libblas-dev liblapack-dev liblapacke-dev
sudo apt-get install -y libsdl2-dev libc++-7-dev libc++abi-7-dev libxi-dev
sudo apt-get install -y llvm-7 clang-7 libclang-7-dev libc++-7-dev libc++abi-7-dev


# install newer cmake

cd $HOME/repos/; mkdir cmake-3-18; cd cmake-3-18; 
wget https://github.com/Kitware/CMake/releases/download/v3.18.6/cmake-3.18.6.tar.gz
tar -zxvf cmake-3.18.6.tar.gz
cd cmake-3-18.6; ./bootstrap
make -j4; sudo make install



# install open3d for arm (tested with 0.12.0)(is not available on pip3 for arm)
git clone --recursive https://github.com/intel-isl/Open3D
cd Open3D
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
sudo make install-pip-package -j$(nproc)
python3.7 -m pip install /home/f/repos/Open3D/build/lib/python_package/pip_package/open3d-0.12.0+b379dcc9-cp37-cp37m-linux_aarch64.whl # might have a different name on yours
python3.7 -c "import open3d; print(open3d)"

# install teaser
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build

# python
cd $HOME/repos/TEASER-plusplus/build
cmake  -DTEASERPP_PYTHON_VERSION=3.7.5  -DPYTHON_EXECUTABLE=/usr/bin/python3.7 -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.7m.so .. && make teaserpp_python -j4

# c++
make -j4
sudo make install
cd .. && cd examples/teaser_cpp_ply && mkdir build && cd build
cmake .. && make

# fix executables
export LD_LIBRARY_PATH="/usr/local/lib"
sudo ldconfig


# test cpp
$HOME/repos/TEASER-plusplus/examples/teaser_cpp_ply/build/./teaser_cpp_ply
#test python
python $HOME/repos/TEASER-plusplus/examples/teaser_python_ply/teaser_python_ply.py

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
# IMPORTANT: on fresh install comment out every line that has pip in it or make sure pip links to python3.7 and not python2 or python3.6.9
export CXX=g++; 
export CC=gcc;
export CUDA_HOME=/usr/local/cuda; 
export MAX_JOBS=6;
sudo python3.7 setup.py install


#install FCGF
cd $HOME/repos/FCGF
git clone https://github.com/chrischoy/FCGF.git
cd FCGF
#remove open3d and minkowski from requirements.txt since it does not exist for arm and is compiled locally
python3.7 -m pip install pybind11
python3.7 -m pip install scipy # takes roughly 1 hour :)
python3.7 -m pip install pillow
python3.7 -m pip install sklearn
python3.7 -m pip install -r requirements.txt
#verify that it works:
python3.7 demo.py


#install deepglobalregistration
cd $HOME/repos/
git clone https://github.com/chrischoy/DeepGlobalRegistration.git
cd DeepGlobalRegistration
pip install -r requirements.txt


### install ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
