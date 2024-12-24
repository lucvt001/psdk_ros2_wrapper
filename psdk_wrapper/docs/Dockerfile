# Use the official ROS Humble base image
FROM ros:humble-perception

# Install common dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    libboost-all-dev \
    libopus-dev \
    vim \
    net-tools \
    iproute2 \
    iputils-ping \
    python3-pip \
    python3-argcomplete \
    && apt-get remove -y ros-humble-compressed-depth-image-transport ros-humble-theora-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Add ROS setup to .bashrc
RUN echo "unset AMENT_PREFIX_PATH" >> /root/.bashrc
RUN echo "unset COLCON_PREFIX_PATH" >> /root/.bashrc
RUN echo "unset CMAKE_PREFIX_PATH" >> /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Set the default workspace
WORKDIR /ros2_ws

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]