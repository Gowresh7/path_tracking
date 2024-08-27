#!/bin/bash

# Set the DISPLAY environment variable
export DISPLAY=${DISPLAY}

# Run the Gazebo container with ROS and additional commands
docker run -it \
  --env="DISPLAY=${DISPLAY}" \
  --gpus all \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev/dri:/dev/dri \
  --name gazebo_container \
  gazebo \
  bash -c "source /opt/ros/noetic/setup.bash && \
           source ~/gem_ws/devel/setup.bash && \
           roslaunch path_tracking path_tracking_sim.launch"
