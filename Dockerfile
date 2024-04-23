# Use the official ROS 2 Iron image as the base image
FROM ros:iron-ros-base

# Install dependencies for TurtleBoat
RUN apt-get update && apt-get install -y

# Set the working directory to the root
WORKDIR /

# Copy the TurtleBoat package to the workspace
COPY . /ros2_ws/src/TurtleBoat
# Set the working directory to the root
WORKDIR /ros2_ws

# Build the workspace
RUN . /opt/ros/iron/setup.sh && \
    colcon build

# Set the entry point to start the TurtleBoat node
CMD . /opt/ros/iron/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    export ROS_DOMAIN_ID=69 && \
    ros2 run turtleboat turtleboatmain