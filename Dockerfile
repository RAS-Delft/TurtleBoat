# Use the official ROS 2 Iron image as the base image
FROM ros:iron-ros-base

# Define the optional build argument VESSEL_ID with a default value of "myvessel1"
ARG VESSEL_ID=myvessel1

# Install dependencies for TurtleBoat
RUN apt-get update && apt-get install -y \
    net-tools \
    iputils-ping

# Set the working directory to the root
WORKDIR /

# Copy the TurtleBoat package to the workspace
COPY . /ros2_ws/src/TurtleBoat

# Set the working directory
WORKDIR /ros2_ws

# Build the workspace
RUN . /opt/ros/iron/setup.sh && \
    colcon build

# Set tasks to be run upon container startup
CMD . /opt/ros/iron/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    export ROS_DOMAIN_ID=69 && \
    echo "my vesselID is '${VESSEL_ID}'" && \
    ros2 run turtleboat turtleboatmain --ros-args --remap __ns:=/${VESSEL_ID}