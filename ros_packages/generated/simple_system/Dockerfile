FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Create logs dir
RUN mkdir -p /ros2_ws/.ros/log

# Copy simple_system package
COPY . /ros2_ws/src/simple_system

# Install dependencies
RUN . /opt/ros/jazzy/setup.sh && colcon build --merge-install

# New entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]