FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY . /ros2_ws/temporal_execution

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --merge-install

CMD . /opt/ros/jazzy/setup.sh && \
    . /ros2_ws/install/setup.sh && \
    ros2 run temporal_execution loop_node