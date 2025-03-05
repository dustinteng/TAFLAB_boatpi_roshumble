# Use a base image with ROS2 Humble
FROM ros:humble-ros-base

# Install necessary dependencies for building ROS2 workspaces
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    # python3-rpi.gpio \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory inside the container
WORKDIR /ros2_ws

# Copy the ROS2 workspace into the container
COPY ./src ./src

# Install dependencies and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    (rosdep init || echo "rosdep already initialized") && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Copy the entrypoint script to /ros2_ws
COPY ./ros_entrypoint.sh /ros2_ws/ros_entrypoint.sh

# Change permissions of the entrypoint script
RUN chmod +x /ros2_ws/ros_entrypoint.sh

# Set the working directory
WORKDIR /ros2_ws

# Set the entrypoint
ENTRYPOINT ["/ros2_ws/ros_entrypoint.sh"]
CMD ["bash"]
