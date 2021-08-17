# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ubuntu:bionic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt update && apt install curl -y

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic
ENV HOME /home/docker

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-core=1.4.1-0* curl iputils-ping git openssh-server \ 
    python-catkin-tools software-properties-common libeigen3-dev libopencv-dev \ 
    libyaml-cpp-dev ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-tf \ 
    ros-$ROS_DISTRO-image-transport \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y 
RUN sudo add-apt-repository universe
RUN apt-get update && apt install -y build-essential curl


# vins-fusion
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev \
    libsuitesparse-dev

# setup inspectrone files
RUN mkdir -p $HOME/catkin_ws/src

# environment setup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc


WORKDIR "$HOME/catkin_ws/src"
RUN git clone https://github.com/ethz-asl/kindr 
RUN git clone https://github.com/ethz-asl/rovio
RUN git clone https://github.com/catkin/catkin_simple
RUN cd rovio && git submodule update --init --recursive
RUN git clone https://ceres-solver.googlesource.com/ceres-solver

# fix imu_frame to imu instead of /imu
RUN sudo apt-get install -y sed
RUN cd rovio/include/rovio/ && sed -i 's@imu_frame_ = "/imu"@imu_frame_ = "imu_new"@g' RovioNode.hpp
RUN cd rovio/include/rovio/ && grep -iRn -e "imu_frame_"

RUN . /opt/ros/melodic/setup.sh && mkdir ceres-bin && cd ceres-bin && cmake ../ceres-solver && make -j4 && make test -j4 && make install -j4 \ 
&& catkin build -w $HOME/catkin_ws kindr \
&& catkin build catkin_simple \
&& catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release

# RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
# RUN . /opt/ros/melodic/setup.sh && catkin build vins

# RUN sudo /bin/bash -c /opt/ros/melodic/setup.bash && 

RUN apt-get update && apt-get install rviz gedit ros-melodic-image-pipeline ros-melodic-image-transport-plugins  -y

RUN echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc

# with nvidia/cuda drivers
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR "$HOME/catkin_ws"
RUN catkin build  --cmake-args -DCMAKE_BUILD_TYPE=Release

# setup entrypoint
COPY .	/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
