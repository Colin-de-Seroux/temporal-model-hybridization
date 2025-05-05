FROM ros:latest

RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener_launch.py"]
