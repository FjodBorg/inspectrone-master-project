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

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic

# with nvidia/cuda drivers
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    

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

# environment setup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# setup inspectrone files
RUN mkdir -p ~/catkin_ws/src

WORKDIR "~/catkin_ws/src"
RUN git clone https://github.com/ethz-asl/kindr 
RUN git clone https://github.com/ethz-asl/rovio
RUN git clone https://github.com/catkin/catkin_simple
RUN cd rovio && git submodule update --init --recursive
RUN git clone https://ceres-solver.googlesource.com/ceres-solver

RUN . /opt/ros/melodic/setup.sh && mkdir ceres-bin && cd ceres-bin && cmake ../ceres-solver && make -j4 && make test -j4 && make install -j4 \ 
&& catkin build -w /~/catkin_ws kindr \
&& catkin build catkin_simple \
&& catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
RUN . /opt/ros/melodic/setup.sh && catkin build vins

# RUN sudo /bin/bash -c /opt/ros/melodic/setup.bash && 

RUN apt-get install rviz gedit -y

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
