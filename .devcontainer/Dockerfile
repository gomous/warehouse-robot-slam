FROM ros:humble

ARG USERNAME=quardrupedROS
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip

# Install useful tools for development
RUN apt-get install -y \
    git \
    wget \
    curl \
    vim \
    nano \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    bash-completion \
    lsof \
    net-tools \
    psmisc

# Initialize rosdep
RUN rosdep init || true && rosdep update

# IMPORTANT: Install Gazebo Classic 11 (NOT Ignition)
# Install ROS 2 Gazebo packages
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-dev


# Install ROS 2 Gazebo Classic packages
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-ros2-control

# Install ROS 2 core packages for robotics
RUN apt-get update && apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins

# Install navigation and SLAM dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-perception-pcl \
    ros-humble-velodyne \
    ros-humble-velodyne-description \
    ros-humble-teleop-twist-keyboard

# Install PCL and dependencies for LIO-SAM
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    libomp-dev

# Install MoveIt 2 packages (optional - only if needed)
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros \
    ros-humble-moveit-planners \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-servo \
    ros-humble-moveit-common \
    ros-humble-moveit-core \
    ros-humble-moveit-msgs \
    ros-humble-moveit-resources \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-setup-assistant \
    ros-humble-ompl

# Install GTSAM dependencies
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libtbb2-dev \
    libmetis-dev

# Build and install GTSAM 4.2a9 from source
RUN cd /tmp && \
    git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 4.2a9 && \
    mkdir build && cd build && \
    cmake .. \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && rm -rf /tmp/gtsam

# Clean up apt cache to reduce image size
RUN apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Set environment variables for Gazebo
ENV GAZEBO_MODEL_PATH=/home/$USERNAME/.gazebo/models
ENV DISPLAY=:0

ENV SHELL /bin/bash

# [Optional] Set the default user
USER $USERNAME
WORKDIR /home/$USERNAME

CMD ["/bin/bash"]