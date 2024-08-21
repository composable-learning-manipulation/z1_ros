ARG BASE_IMG='nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04'

FROM ${BASE_IMG}
LABEL org.opencontainers.image.authors="texnoman@itmo.com"

EXPOSE 11311
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND noninteractive
ENV GUI false
WORKDIR '/ros_ws'


# Timezone Configuration
ENV TZ=Europe/Moscow
ENV ROS_DISTRO=noetic
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install dependencies for ROS
RUN apt-get update && apt-get upgrade -y &&\
    apt-get install --no-install-recommends -y \
    git curl wget unzip tmux\
    net-tools vim nano lsb-release \
	build-essential gcc g++ cmake make \
	libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
	libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    yasm libatlas-base-dev gfortran libpq-dev \
    libxine2-dev libglew-dev libtiff5-dev zlib1g-dev libavutil-dev libpostproc-dev \
    python3-dev python3-pip libx11-dev tzdata apt-utils mesa-utils gnupg2 \
    && rm -rf /var/lib/apt/lists/* && apt autoremove && apt clean

# Install ROS, see (http://wiki.ros.org/${ROS_DISTRO}/Installation/Ubuntu)
RUN apt-get update \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update && apt-get install -y ros-${ROS_DISTRO}-desktop-full \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc\
    && apt-get install -y python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-catkin-tools \
        python-lxml \
        build-essential \
    && rosdep init && rosdep update \
    && rm -rf /var/lib/apt/lists/* && apt autoremove && apt clean

RUN mkdir -p /ros_ws/src && cd /ros_ws \
    && catkin config --extend /opt/ros/${ROS_DISTRO} \
    && catkin build && echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

# OpenCV and other libraries Installation
RUN apt-get update \
    && libpcl-dev libeigen3-dev \
    && apt-get install -q -y openssh-client \
    && apt-get install -q -y ros-${ROS_DISTRO}-cv-bridge \
    && apt-get install -q -y python3-opencv \
    && apt-get update -y || true && apt-get install -y \
	&& apt-get install -y --no-install-recommends libopencv-dev \
    && rm -rf /var/lib/apt/lists/* && apt autoremove && apt clean

RUN apt-get update \
    && apt-get install -y ros-${ROS_DISTRO}-realsense2-description \
        ros-${ROS_DISTRO}-realsense2-camera \
    && rm -rf /var/lib/apt/lists/* && apt autoremove && apt clean


# Install ROS packages
RUN apt-get update \
    && apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-control \
        ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers \
        ros-${ROS_DISTRO}-position-controllers \
        ros-${ROS_DISTRO}-velocity-controllers \
        ros-${ROS_DISTRO}-joint-state-controller \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander \
        ros-${ROS_DISTRO}-moveit-visual-tools ros-${ROS_DISTRO}-combined-robot-hw \
    && rm -rf /var/lib/apt/lists/* && apt autoremove && apt clean

# Install ROS packages required for calibration
RUN python3 -m pip install --upgrade pip
RUN apt-get update \
    && apt-get install -y python3-pybind11 \
    && apt-get install -y ros-${ROS_DISTRO}-pinocchio \
        ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    && sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen \
    && rm -rf /var/lib/apt/lists/* && apt autoremove && apt clean

RUN mkdir src/z1_ros
COPY ./ src/z1_ros

RUN catkin build

