FROM osrf/ros:noetic-desktop-full

# Install Gazebo and other necessary packages
RUN apt-get update && apt-get install -y \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    nano \
    git \
    python3 \
    python-yaml \
    python-is-python3 \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a non-root user with sudo privileges
RUN useradd -ms /bin/bash rosuser && echo "rosuser:a" | chpasswd && \
    usermod -aG sudo rosuser && \
    echo 'rosuser ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to the new user
USER rosuser
WORKDIR /home/rosuser/gem_ws

# Set up the workspace and clone the repository
RUN mkdir -p ~/gem_ws/src && \
    cd ~/gem_ws/src && \
    git clone https://github.com/Gowresh7/POLARIS_GEM_e2.git && \
    git clone https://github.com/Gowresh7/path_tracking.git

# Build the workspace
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Switch back to root to modify the entrypoint script
USER root
RUN chmod +x /ros_entrypoint.sh && \
    sed -i '/^exec "\$@"/i source /opt/ros/noetic/setup.bash' /ros_entrypoint.sh && \
    sed -i '/^exec "\$@"/i source /home/rosuser/gem_ws/devel/setup.bash' /ros_entrypoint.sh && \
    chmod +x /home/rosuser/gem_ws/src/path_tracking/src/tf_publisher.py && \
    chmod +x /home/rosuser/gem_ws/src/path_tracking/src/path_publisher.py 

# Switch back to rosuser for the default user
USER rosuser
WORKDIR /home/rosuser/gem_ws
