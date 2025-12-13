# Dockerfile for VLA (Vision-Language-Action) testing environment
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ISAAC_SIM_PYTHON_VERSION=3.10

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y ros-humble-desktop \
    ros-humble-ros-base \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade pip \
    && pip3 install \
    openai \
    speechrecognition \
    pyaudio \
    opencv-python \
    numpy \
    rospkg \
    catkin-tools

# Install additional ROS packages
RUN apt-get update \
    && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros-gz \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav2-bringup \
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace directory
WORKDIR /workspace
RUN mkdir -p /workspace/src

# Copy project files (if any)
COPY . /workspace/

# Source ROS environment by default
ENV ROS_LOCAL_INSTALL=/opt/ros/humble
ENV PYTHONPATH=${ROS_LOCAL_INSTALL}/lib/python3.10/site-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
ENV PKG_CONFIG_PATH=/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH

# Default command
CMD ["bash"]