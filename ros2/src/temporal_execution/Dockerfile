FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Create logs dir
RUN mkdir -p /ros2_ws/.ros/log

# Copy temporal_execution package
COPY . /ros2_ws/src/temporal_execution

# Install dependencies
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --merge-install

# Source the workspace, install dependencies and run the node
CMD . /opt/ros/jazzy/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    ros2 run temporal_execution loop_node > /dev/null 2>&1
