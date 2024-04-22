# Use the official ROS 2 Foxy Fitzroy base image
FROM ros:iron-ros-base

# Install dependencies for TurtleBoat
RUN apt-get update && apt-get install -y

# Set the working directory
WORKDIR /ros2_ws

# Copy the TurtleBoat package to the workspace
COPY . /src/TurtleBoat

# Build the workspace
RUN . /opt/ros/iron/setup.sh && \
    colcon build

# Set the entry point to start the TurtleBoat node
CMD . /opt/ros/iron/setup.sh && \
    ros2 launch TurtleBoat turtleboat_launch.py