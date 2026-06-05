FROM nvidia/cuda:12.1.0-base-ubuntu22.04

WORKDIR /home

# Timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Python deps
RUN apt-get update && apt-get install -y python3-pip python3-tk wget curl nano tmux git && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    numpy==1.24.3 \
    matplotlib \
    pandas \
    pyqtgraph==0.13.3 \
    PyQt5==5.15.9

RUN pip3 install torch==2.0.1 torchvision==0.15.2 --index-url https://download.pytorch.org/whl/cu118

# Gazebo Classic (gazebo11 compatible with Ubuntu 22.04)
RUN apt-get update && apt-get install -y gazebo libgazebo-dev && \
    rm -rf /var/lib/apt/lists/*

# Gazebo models
ENV GAZEBO_MODEL_DATABASE_URI ""
RUN mkdir -p /root/.gazebo/models
RUN wget -q https://raw.githubusercontent.com/osrf/gazebo_models/master/ground_plane/model.sdf -P /root/.gazebo/models/ground_plane && \
    wget -q https://raw.githubusercontent.com/osrf/gazebo_models/master/ground_plane/model.config -P /root/.gazebo/models/ground_plane && \
    wget -q https://raw.githubusercontent.com/osrf/gazebo_models/master/sun/model.sdf -P /root/.gazebo/models/sun && \
    wget -q https://raw.githubusercontent.com/osrf/gazebo_models/master/sun/model.config -P /root/.gazebo/models/sun

# ROS2 Humble
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    python3-argcomplete \
    python3-rosdep \
    ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/turtlebot3_drlnav

RUN echo "\n\
source /opt/ros/humble/setup.bash\n\
export ROS_DOMAIN_ID=1\n\
export DRLNAV_BASE_PATH='/home/turtlebot3_drlnav'\n\
source \$DRLNAV_BASE_PATH/install/setup.bash\n\
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$DRLNAV_BASE_PATH/src/turtlebot3_simulations/turtlebot3_gazebo/models\n\
export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:\$DRLNAV_BASE_PATH/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/obstacle_plugin/lib\n\
" >> ~/.bashrc

CMD ["bash"]
