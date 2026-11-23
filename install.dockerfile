FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV DISABLE_ROS1_EOL_WARNING=1

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    ros-noetic-dynamixel-sdk \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3 \
    ros-noetic-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Clone and compile the turtlebot3_simulations package catkin workspace
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    git clone -b noetic-devel https://github.com && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make"


# Configure the environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /workspace
