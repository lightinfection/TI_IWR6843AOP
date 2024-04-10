FROM ubuntu:jammy

ARG ROS_DISTRO=humble \
    ROS_WORKSPACE=/ros_ws
    
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

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONIOENCODING=utf-8 \
    ROS_PYTHON_VERSION=3 \
    ROS_VERSION=2 \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_ROOT=/opt/ros/${ROS_DISTRO} \
    ROS_WORKSPACE=${ROS_WORKSPACE} \
    ROS_PACKAGE_PATH=/opt/ros/${ROS_DISTRO}:${ROS_WORKSPACE}/install 

WORKDIR ${ROS_WORKSPACE}
ENV ROS_DISTRO humble

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    zip \
    unzip \
    vim \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 base packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ["ros_entrypoint.sh", "/ros_entrypoint.sh"]
RUN chmod +x /ros_entrypoint.sh && \
    sed -n '/set -e/,/exec "\$@"/p' /ros_entrypoint.sh | sed '1d;$d' >> /root/.bashrc

# Install object_detection and python serial dependecies
RUN apt-get update && \
    apt-get install --no-install-recommends -y libpcl-dev &&\
    apt-get install --no-install-recommends -y libeigen3-dev &&\
    apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros && \
    apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-pcl-ros && \
    apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-octomap && \
    apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-visualization-msgs && \
    apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-backward-ros && \
    apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-rviz2 && \
    apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-cartographer-rviz && \
    apt-get install --no-install-recommends -y ros-${ROS_DISTRO}-octomap-rviz-plugins && \
    pip3 install scipy PyQt5 pyserial matplotlib && \
    apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN if [ ! -d ${ROS_WORKSPACE} ]; then mkdir ${ROS_WORKSPACE}; fi
COPY src.zip ${ROS_WORKSPACE}/

RUN . ${ROS_ROOT}/setup.sh && \
    cd ${ROS_WORKSPACE}/ && \
    unzip src.zip && \
    colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    rm -rf src build log && \
    apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]