#!/bin/bash
# This is Dockerfile for inspectrone enviroment on jetson
arg PLATFORM=linux/arm64
FROM --platform=$PLATFORM nvcr.io/nvidia/l4t-base:r32.4.4

#change shell
SHELL ["/bin/bash", "-c"]

# set up basic variables
ARG ROS_PKG=ros-melodic-ros-base 
ENV ROS_DISTRO=melodic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}


# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# add the ROS deb repo to the apt sources list
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ROS packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		$ROS_PKG \
		ros-melodic-image-transport \
		ros-melodic-vision-msgs \
          python-rosdep \
          python-rosinstall \
          python-rosinstall-generator \
          python-wstool \
          sudo \
    && rm -rf /var/lib/apt/lists/*

#install dependencies
RUN apt-get update && apt-get install -y software-properties-common apt-utils
RUN  add-apt-repository universe

ARG DEBIAN_FRONTEND=noninteractive #fix for packages below
RUN  apt install -y python3.7 python3.7-dev openssl libssl-dev  ninja-build gfortran dialog
RUN  apt install -y cmake libeigen3-dev libboost-all-dev libopenblas-dev build-essential
RUN  apt install -y libjpeg-dev zlib1g-dev dirmngr

# more enviroment
ARG USER="jetson"
ENV HOME=/home/${USER}


# setup basic user
RUN useradd -ms /bin/bash $USER && usermod -aG sudo $USER && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# init/update rosdep
RUN apt-get update && \
    apt-get -y install python-catkin-tools &&\ 
    cd ${ROS_ROOT} && \
    rosdep init 
    
USER $USER 
RUN rosdep update
RUN echo $'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}\n\
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}\n\
source /opt/ros/$ROS_DISTRO/setup.bash' >>  $HOME/.bashrc 

USER root 
RUN rm -rf /var/lib/apt/lists/*

#extras
RUN apt update && apt install -y nano ros-melodic-rviz ros-melodic-pcl-ros
#RUN apt update && apt install -y nano 

# with nvidia/cuda drivers
#ENV NVIDIA_VISIBLE_DEVICES \
#    ${NVIDIA_VISIBLE_DEVICES:-all}
#ENV NVIDIA_DRIVER_CAPABILITIES \
#    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    

WORKDIR $HOME


#ENTRYPOINT ["/entrypoint_setup.sh"]
CMD ["bash"]
